#general
import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys
from datetime import datetime
import time
from inspect import getsourcefile
from os.path import abspath

#ros
import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs


#Specifics
import utils.TTSExec
import utils.ExpressionExec
import utils.GestureExec
import utils.HeadGazeGesExec



#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        # print("Config file loaded")
    return config_data

#Create Logger
def setupLogger(file_log_level, terminal_log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(msecs)03d %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(file_log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(terminal_log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel( min(file_log_level,terminal_log_level) )#set to lowest

    return logger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()



class BehavExec:


    def __init__(self):
        #miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))
        path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0))) + "/config/config.yaml"
        self.__config_data = loadConfig(path)
        self.__nh = rospy.init_node(self.__config_data['Ros']['node_name'])

        
        #For tts
        self.__tts_exec = utils.TTSExec.TTSExec(self.__config_data,self.__logger)

        #For arm gesture
        self.__gesture_exec = utils.GestureExec.GestureExec(self.__config_data,self.__logger)

        #For expressions
        self.__expression_exec = utils.ExpressionExec.ExpressionExec(self.__config_data,self.__logger)

        #For gaze & head gestures
        self.__head_gaze_exec = utils.HeadGazeGesExec.HeadGazeGesExec(self.__config_data,self.__logger)

        #For behaviour execution service
        self.__end_of_conv_sub = rospy.Subscriber(      
                                        self.__config_data['Ros']['end_of_conv_topic'], 
                                        std_msgs.msg.Bool, 
                                        self.__endOfConvCallback, 
                                        queue_size=self.__config_data['Ros']['queue_size'])
        self.__behavior_server = rospy.Service(
                                        self.__config_data['Ros']['grace_behavior_service'],
                                        grace_attn_msgs.srv.GraceBehavior, 
                                        self.__handleGraceBehaviorServiceCall)

        self.__speak_event_pub = rospy.Publisher(      
                                        self.__config_data['Ros']['speak_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Ros']['queue_size'])
        self.__hum_event_pub = rospy.Publisher(      
                                        self.__config_data['Ros']['hum_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Ros']['queue_size'])
        self.__nod_event_pub = rospy.Publisher(      
                                        self.__config_data['Ros']['nod_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Ros']['queue_size'])
        self.__gaze_event_pub = rospy.Publisher(      
                                        self.__config_data['Ros']['gaze_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Ros']['queue_size'])


    def __allBehavStop(self, req = None, res = None):

        #Cutoff any on-going composite behavior
        self.__compBehavStop()

        #Neutral gaze & head
        self.__head_gaze_exec.goToNeutral()
        self.__gaze_event_pub.publish( self.__config_data['General']['gaze_neutral_event_name'] )
        self.__nod_event_pub.publish( self.__config_data['General']['stop_nodding_event_name'] )

        #Return evecution results if necessary
        if(res != None):
            res.result = self.__config_data['General']['behav_all_stopped_string']
            return res
    
    def __compBehavStop(self):
        #Cutoff any on-going tts
        self.__tts_exec.stopTTS()

        #Reset to neutral arm-pose and facial expression
        self.__goToNeutralComp()

        #Robot behav state events
        self.__speak_event_pub.publish( self.__config_data['General']['stop_speaking_event_name'] )
        self.__hum_event_pub.publish( self.__config_data['General']['stop_humming_event_name'] ) 
        
    def __endOfConvCallback(self, msg):
        self.__allBehavStop()



    '''
        Composite Behavior Service handling
    '''
    def __compositeExec(self, req, res):
        #We don't need auto-generated expressions and gestures anymore
        edited_text = self.__tts_exec.postEditTTSText(req.utterance)
        
        #Get total duration of tts
        dur_total = self.__tts_exec.parseTTSDur(self.__tts_exec.getTTSData(edited_text,req.lang))

        #Arrange expressions and gestures in physical time
        expression_seq = self.__arrangeCompExecSeq(dur_total, req.expressions, req.exp_start, req.exp_end, req.exp_mag)
        gesture_seq = self.__arrangeCompExecSeq(dur_total, req.gestures, req.ges_start, req.ges_end, req.ges_mag)

        #Prepare two threads for executing expression and gestures
        exp_thread = threading.Thread(target=lambda: self.__compExecBySeq(expression_seq, self.__expression_exec.triggerExpressionFixedDur), daemon=False)
        ges_thread = threading.Thread(target=lambda: self.__compExecBySeq(gesture_seq, self.__gesture_exec.triggerArmAnimationFixedDur), daemon=False)


        #Initiate tts, gesture, expression execution and start polling execution completion
        self.__behav_service_thread_keep_alive = True
        self.__behavior_exec_start_time = rospy.get_time()
        self.__tts_exec.say(edited_text, req.lang)
        exp_thread.start()
        ges_thread.start()
        rate = rospy.Rate(self.__config_data['CompositeBehavior']['comp_behav_exec_rate'])
        while True:
            rate.sleep()

            #Nobody said anything, check the tts state
            if(self.__tts_exec.receivedTTSEndEvent()):#TTS is over
                self.__logger.info('Execution completed.')
                
                #Stop gesture and expressions
                self.__goToNeutralComp()

                #Report successful completion of the behaviour execution
                res.result = self.__config_data['General']['behav_succ_string']

                #Break the loop and finish the service
                break
            else:#TTS still going
                pass#Do nothing

        return res

    def __arrangeCompExecSeq(self, total_dur, names, start_portion, end_portion, magnitude):
        num_behav = len(names)


        behav_seq = [None] * num_behav
        for i in range(num_behav):
            behav_seq[i] = [None] * 4 


        for i in range(num_behav):
            behav_seq[i][0] = names[i]
            behav_seq[i][1] = start_portion[i] * total_dur
            behav_seq[i][2] = end_portion[i] * total_dur
            behav_seq[i][3] = magnitude[i]
        return behav_seq

    def __compExecBySeq(self, behav_seq, exec_fnc):
        
        num_behav = len(behav_seq)
        
        rate = rospy.Rate(self.__config_data['CompositeBehavior']['comp_behav_exec_rate'])

        #The behaviour to be executed
        exec_cnt = 0
        #Total time since the start of thie performance command
        elapsed_time = 0


        while self.__behav_service_thread_keep_alive:
            #Update the elapsed time
            elapsed_time = rospy.get_time() - self.__behavior_exec_start_time

            if( exec_cnt < num_behav):# Start executing this behavior
                if( elapsed_time >= behav_seq[exec_cnt][1]):
                    self.__logger.info("Executing behavior %d: %s" % (exec_cnt , behav_seq[exec_cnt][0]))

                    self.__logger.debug(type(behav_seq[exec_cnt][0]))
                    exec_fnc(behav_seq[exec_cnt][0], behav_seq[exec_cnt][2] - behav_seq[exec_cnt][1], behav_seq[exec_cnt][3])
                    
                    exec_cnt = exec_cnt + 1 
            else:#Nothing more to execute
                break

            rate.sleep()

    def __goToNeutralComp(self):
        #Kill any on-going behaviour service thread
        self.__behav_service_thread_keep_alive = False

        #Reset to a neutral arm pose
        self.__gesture_exec.triggerArmAnimationFixedDur(
            self.__config_data['CompositeBehavior']['Predefined']['neutral_pose_info']['name'],
            self.__config_data['CompositeBehavior']['Predefined']['neutral_pose_info']['dur'],
            self.__config_data['CompositeBehavior']['Predefined']['neutral_pose_info']['magnitude'])

        #Reset to a neutral expression
        self.__expression_exec.triggerExpressionFixedDur(
            self.__config_data['CompositeBehavior']['Predefined']['neutral_expression_info']['name'],
            self.__config_data['CompositeBehavior']['Predefined']['neutral_expression_info']['dur'],
            self.__config_data['CompositeBehavior']['Predefined']['neutral_expression_info']['magnitude'])



    '''
    Interface
    '''
    def mainLoop(self):
        rate = rospy.Rate(0.1)
        while(True):
            # self.__triggerExpressionFixedDur('happy',3,0.8)
            rate.sleep()

    def __handleGraceBehaviorServiceCall(self, req):
        #Prepare response object
        res = grace_attn_msgs.srv.GraceBehaviorResponse()

        if(req.command == self.__config_data['General']['all_behav_stop_cmd']):
            res = self.__allBehavStop(req,res)

        elif(req.command == self.__config_data['General']['utterance_behav_stop_cmd']):
            self.__compBehavStop()
            res = self.__config_data['General']['utterance_stopped_string']
  

        elif(req.command == self.__config_data['General']['comp_behav_exec_cmd']):
            self.__compBehavStop()#stop previous ones
            self.__speak_event_pub.publish( self.__config_data['General']['start_speaking_event_name'] )
            res = self.__compositeExec(req,res)
            self.__speak_event_pub.publish( self.__config_data['General']['stop_speaking_event_name'] )
        
        elif(req.command == self.__config_data['General']['hum_behav_exec_cmd']):
            self.__compBehavStop()#stop previous ones
            self.__hum_event_pub.publish( self.__config_data['General']['start_humming_event_name'] )
            res = self.__compositeExec(req,res)
            self.__hum_event_pub.publish( self.__config_data['General']['stop_humming_event_name'] )   
        
        elif(req.command == self.__config_data['General']['nod_cmd']):
            self.__nod_event_pub.publish( self.__config_data['General']['start_nodding_event_name'] )
            self.__head_gaze_exec.nodOnce()
            res.result = self.__config_data['General']['behav_succ_string']
            self.__nod_event_pub.publish( self.__config_data['General']['stop_nodding_event_name'] )

        elif(req.command == self.__config_data['General']['head_gaze_follow']):
            self.__gaze_event_pub.publish( self.__config_data['General']['start_following_event_name'] )
            self.__head_gaze_exec.startFollowing()
            res.result = self.__config_data['General']['behav_succ_string']

        elif(req.command == self.__config_data['General']['head_gaze_avert']):
            self.__gaze_event_pub.publish( self.__config_data['General']['start_aversion_event_name'] )
            self.__head_gaze_exec.startAverting()
            res.result = self.__config_data['General']['behav_succ_string']

        else:
            self.__logger.error("Unexpected behavior command %s." % req.command)
        return res





if __name__ == '__main__':
    behav_executor = BehavExec()
    behav_executor.mainLoop()

































