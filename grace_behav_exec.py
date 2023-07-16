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
file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
sys.path.append(file_path)
import utils.TTSExec
import utils.ExpressionExec
import utils.GestureExec
import utils.NoddingExec
import utils.GazeBehavExec
from enum import Enum

#Misc
sys.path.append(os.path.join(file_path, '..'))
from CommonConfigs.grace_cfg_loader import *
from CommonConfigs.logging import setupLogger


#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()


class ExecFnc(Enum):
    GAZE = 0
    NOD = 1
    HUM = 2
    SPEECH = 3

class BehavExec:


    def __init__(self, config_data, service_mode = True, instance_fnc = None):
        #Miscellaneous
        signal(SIGINT, handle_sigint)

        #Config
        self.__config_data = config_data

        #Operation mode
        self.__service_mode = service_mode
        self.__instance_fnc = instance_fnc

        #Executor uses its own logger and output to its own dir
        logger_name = self.__class__.__name__
        if(not self.__service_mode):
            self.__instance_string = '[' + str(self.__instance_fnc) + ']'
            #In instance mode,we better not use class name alone as logger name
            logger_name = self.__instance_string + logger_name

        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    logger_name,
                    os.path.join(file_path,"./logs/log_") + datetime.now().strftime(self.__config_data['Custom']['Logging']['time_format']))

        #Ros node initialization
        if(self.__service_mode):
            self.__nh = rospy.init_node(self.__config_data['BehavExec']['Ros']['node_name'])

        if(self.__service_mode or self.__instance_fnc == ExecFnc.HUM or self.__instance_fnc == ExecFnc.SPEECH):
            #For tts
            self.__tts_exec = utils.TTSExec.TTSExec(self.__config_data,self.__logger)

            #For arm gesture
            self.__gesture_exec = utils.GestureExec.GestureExec(self.__config_data,self.__logger)

            #For expressions
            self.__expression_exec = utils.ExpressionExec.ExpressionExec(self.__config_data,self.__logger)


        #For nodding
        if(self.__service_mode or self.__instance_fnc == ExecFnc.NOD):
            self.__nod_exec = utils.NoddingExec.NoddingExec(self.__config_data,self.__logger)



        #For gaze (and head) attention
        if(self.__service_mode or self.__instance_fnc == ExecFnc.GAZE):
            self.__gaze_exec = utils.GazeBehavExec.GazeBehavExec(self.__config_data,self.__logger)


        #For flow control
        self.__end_of_conv_sub = rospy.Subscriber(      
                                        self.__config_data['Custom']['Flow']['topic_stop_all'], 
                                        std_msgs.msg.Bool, 
                                        self.__endOfConvCallback, 
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])

        #For behaviour execution 
        if(self.__service_mode):
            #ros-service interface
            self.__behavior_server = rospy.Service(
                                            self.__config_data['Custom']['Behavior']['grace_behavior_service'],
                                            grace_attn_msgs.srv.GraceBehavior, 
                                            self.__handleGraceBehaviorServiceCall)
        else:
            #Threading interface
            self.__behav_thread = None


        #For robot behavior state synchronization
        self.__speak_event_pub = rospy.Publisher(      
                                        self.__config_data['Custom']['Behavior']['Event']['speak_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__hum_event_pub = rospy.Publisher(      
                                        self.__config_data['Custom']['Behavior']['Event']['hum_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__nod_event_pub = rospy.Publisher(      
                                        self.__config_data['Custom']['Behavior']['Event']['nod_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__gaze_event_pub = rospy.Publisher(      
                                        self.__config_data['Custom']['Behavior']['Event']['gaze_event_topic'], 
                                        std_msgs.msg.String,
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])


    def __allBehavStop(self, req = None, res = None):
        if(self.__service_mode or self.__instance_fnc == ExecFnc.HUM or self.__instance_fnc == ExecFnc.SPEECH):
            #Cutoff any on-going composite behavior
            self.__compBehavStop()

        if(self.__service_mode or self.__instance_fnc == ExecFnc.GAZE):
            #Neutral gaze & head
            self.__gaze_exec.goToNeutral()
            self.__gaze_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['gaze_neutral_event_name'] )
            self.__nod_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_nodding_event_name'] )

        #Nothing to "stop" for nodding

        #Return evecution results if necessary
        if(res != None):
            res.result = self.__config_data['BehavExec']['General']['behav_all_stopped_string']
            return res
    
    def __compBehavStop(self):
        #Cutoff any on-going tts
        self.__tts_exec.stopTTS()

        #Reset to neutral arm-pose and facial expression
        self.__goToNeutralComp()

        #Robot behav state events
        self.__speak_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_speaking_event_name'] )
        self.__hum_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_humming_event_name'] ) 

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
        rate = rospy.Rate(self.__config_data['BehavExec']['CompositeBehavior']['comp_behav_exec_rate'])
        while True:
            rate.sleep()

            #Nobody said anything, check the tts state
            if(self.__tts_exec.receivedTTSEndEvent()):#TTS is over
                self.__logger.info('Execution completed.')
                
                #Stop gesture and expressions
                self.__goToNeutralComp()

                #Report successful completion of the behaviour execution
                res.result = self.__config_data['BehavExec']['General']['behav_succ_string']

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
        
        rate = rospy.Rate(self.__config_data['BehavExec']['CompositeBehavior']['comp_behav_exec_rate'])

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
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_pose_info']['name'],
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_pose_info']['dur'],
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_pose_info']['magnitude'])

        #Reset to a neutral expression
        self.__expression_exec.triggerExpressionFixedDur(
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_expression_info']['name'],
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_expression_info']['dur'],
            self.__config_data['BehavExec']['CompositeBehavior']['Predefined']['neutral_expression_info']['magnitude'])



    '''
    Interface
    '''
    def mainLoop(self):
        rate = rospy.Rate(0.1)
        while(True):
            # self.__triggerExpressionFixedDur('happy',3,0.8)
            rate.sleep()

    def __handleGraceBehaviorRequest(self, req):
        #Prepare response object
        res = grace_attn_msgs.srv.GraceBehaviorResponse()

        if(req.command == self.__config_data['BehavExec']['General']['all_behav_stop_cmd']):
            res = self.__allBehavStop(req,res)

        elif(req.command == self.__config_data['BehavExec']['General']['utterance_behav_stop_cmd']):
            self.__compBehavStop()
            res = self.__config_data['BehavExec']['General']['utterance_stopped_string']
  

        elif(req.command == self.__config_data['BehavExec']['General']['comp_behav_exec_cmd']):
            self.__compBehavStop()#stop previous ones
            self.__speak_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['start_speaking_event_name'] )
            res = self.__compositeExec(req,res)
            self.__speak_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_speaking_event_name'] )
        
        elif(req.command == self.__config_data['BehavExec']['General']['hum_behav_exec_cmd']):
            self.__compBehavStop()#stop previous ones
            self.__hum_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['start_humming_event_name'] )
            res = self.__compositeExec(req,res)
            self.__hum_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_humming_event_name'] )   
        
        elif(req.command == self.__config_data['BehavExec']['General']['nod_cmd']):
            self.__nod_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['start_nodding_event_name'] )
            self.__nod_exec.nodOnce()
            res.result = self.__config_data['BehavExec']['General']['behav_succ_string']
            self.__nod_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['stop_nodding_event_name'] )

        elif(req.command == self.__config_data['BehavExec']['General']['head_gaze_follow']):
            self.__gaze_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['start_following_event_name'] )
            self.__gaze_exec.startFollowing()
            res.result = self.__config_data['BehavExec']['General']['behav_succ_string']

        elif(req.command == self.__config_data['BehavExec']['General']['head_gaze_avert']):
            self.__gaze_event_pub.publish( self.__config_data['BehavExec']['BehavEvent']['start_aversion_event_name'] )
            self.__gaze_exec.startAverting()
            res.result = self.__config_data['BehavExec']['General']['behav_succ_string']

        else:
            self.__logger.error("Unexpected behavior command %s." % req.command)
        return res

    def __handleGraceBehaviorServiceCall(self, req):
        #Old ros-service interface
        return self.__handleGraceBehaviorRequest(req)

    def initiateBehaviorThread(self, req):
        #This function doest NOT check thread safety, i.e., if another thread is running
        #the behavior call is assumed to be handled either
        #(1) Near instantaneously: gaze, nod, stop command
        #or (2) By terminating previous ones: humming and speaking will call the stop-comp method first
        self.__behav_thread = threading.Thread(
                            target = self.__handleGraceBehaviorRequest, 
                            args = [req],
                            daemon = False)
        self.__behav_thread.start()



if __name__ == '__main__':
    grace_config = loadGraceConfigs()
    behav_executor = BehavExec(grace_config)
    behav_executor.mainLoop()

































