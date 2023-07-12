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
from enum import Enum
import numpy
import copy

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

class GazeBehavior(Enum):
    NEUTRAL = 0
    FOLLOW = 1
    AVERSION = 2

class GazeBehavExec:


    def __init__(self, config_data, logger):
        #Misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #ROS IO
        self.__hr_people_pub = rospy.Publisher(
                                        self.__config_data['HR']['GazeHead']['hr_people_perception_topic'],
                                        hr_msgs.msg.People, 
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__hr_face_target_pub = rospy.Publisher(
                                        self.__config_data['HR']['GazeHead']['hr_face_target_topic'], 
                                        hr_msgs.msg.Target, 
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__hr_ATTN_cfg_client = dynamic_reconfigure.client.Client(
                                        self.__config_data['HR']['GazeHead']['hr_ATTN_cfg_server'],
                                        timeout=self.__config_data['Custom']['Ros']['dynam_config_timeout'])
        self.__target_person_sub = rospy.Subscriber(
                                        self.__config_data['Custom']['Behavior']['final_target_person_topic'], 
                                        hr_msgs.msg.Person, 
                                        self.__targetPersonCallback, 
                                        queue_size=self.__config_data['Custom']['Ros']['queue_size'])


        #Following & aversion specs
        self.gaze_state = GazeBehavior.FOLLOW

        self.__follow_timeout = self.__config_data['BehavExec']['HeadGazeGes']['follow_timeout']

        self.__gaze_thread_rate = self.__config_data['BehavExec']['HeadGazeGes']['gaze_thread_rate']
        self.__dummy_target_id = self.__config_data['BehavExec']['HeadGazeGes']['dummy_target_id']
        self.__aversion_target_expiry_stamp = -1


        self.__aversion_target_min_life_expand = self.__config_data['BehavExec']['HeadGazeGes']['aversion_target_min_life_expand']
        self.__aversion_mean_life_expand = self.__config_data['BehavExec']['HeadGazeGes']['aversion_mean_life_expand']
        self.__aversion_loc_range = self.__config_data['BehavExec']['HeadGazeGes']['aversion_loc_range']

        self.__no_target_string = self.__config_data['BehavExec']['HeadGazeGes']['no_target_string']
        self.__target_person_msg = hr_msgs.msg.Person()
        self.__target_person_msg.id = self.__no_target_string 



        #Fake gaze target
        self.__neutral_gaze_dummy_target = hr_msgs.msg.Target()
        self.__neutral_gaze_dummy_target.x = self.__config_data['BehavExec']['HeadGazeGes']['neutral_gaze_dummy_target'][0]
        self.__neutral_gaze_dummy_target.y = self.__config_data['BehavExec']['HeadGazeGes']['neutral_gaze_dummy_target'][1]
        self.__neutral_gaze_dummy_target.x = self.__config_data['BehavExec']['HeadGazeGes']['neutral_gaze_dummy_target'][2]
        self.__neutral_gaze_dummy_target.speed = self.__config_data['BehavExec']['HeadGazeGes']['neutral_gaze_dummy_target'][3]
        

        #Gaze behavior thread
        self.gaze_behav_thread = threading.Thread(target = self.__gazeBehavThread,daemon = False)
        self.gaze_behav_thread.start()
                
    def __targetPersonCallback(self, msg):
        self.__target_person_msg = msg

    def __configureGraceATTN(self, target_id = None):
        if(target_id != None):
            #Look at target given by id
            self.__hr_ATTN_cfg_client.update_configuration(
                {
                    "enable_flag": True,
                    "look_at_face":target_id, 
                    "look_at_start":True, 
                    "look_at_time":self.__follow_timeout
                })
        else:
            #No id input -- turn off look-at function
            self.__hr_ATTN_cfg_client.update_configuration(
                {
                    "enable_flag": False,
                    "look_at_face":'', 
                    "look_at_start":False
                })

    def __publishGazeAversionTarget(self, dummy_person):
        dummy_people = hr_msgs.msg.People()
        dummy_people.people.append(dummy_person)
        self.__hr_people_pub.publish(dummy_people)

    def __generateGazeAversionTarget(self):
        #Copy the target person message
        gaze_aversion_target_msg = copy.deepcopy(self.__target_person_msg)

        #Hack the id to indicate that this is a dummy target for aversion
        gaze_aversion_target_msg.id = self.__dummy_target_id

        #Generate a random position in space
        gaze_aversion_target_msg.body.location.x = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.__aversion_loc_range[0],self.__aversion_loc_range[1])
        gaze_aversion_target_msg.body.location.y = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.__aversion_loc_range[0],self.__aversion_loc_range[1])
        gaze_aversion_target_msg.body.location.z = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.__aversion_loc_range[0],self.__aversion_loc_range[1])
            
        return gaze_aversion_target_msg


    def __gazeNeutral(self):
        #Neutral target
        self.__hr_face_target_pub.publish(self.__neutral_gaze_dummy_target)
        #Turn off lookat
        self.__configureGraceATTN()

    def __gazeFollowing(self):
        #(Turn on attention and) use the latest tracked person as target
        self.__configureGraceATTN(self.__target_person_msg.id)

    def __gazeAversion(self):
        current_timestamp = time.time()
        if(current_timestamp >= self.__aversion_target_expiry_stamp):
            #Decide on a new aversion duration
            interval = max(
                self.__aversion_target_min_life_expand,
                numpy.random.exponential(self.__aversion_mean_life_expand))
            #Apply a new aversion target
            aversion_target_msg = self.__generateGazeAversionTarget()
            self.__publishGazeAversionTarget(aversion_target_msg)
            time.sleep(0.05)
            self.__configureGraceATTN(aversion_target_msg.id)

            self.__aversion_target_expiry_stamp = interval + time.time() 
            self.__logger.info("Next aversion target in %f seconds." % interval)

    def resetAversionExpiryStamp(self):
        self.__aversion_target_expiry_stamp = -1

    def __gazeBehavThread(self):
        rate = rospy.Rate(self.__gaze_thread_rate)

        while(True):
            rate.sleep()
            if(self.gaze_state == GazeBehavior.NEUTRAL):
                self.__gazeNeutral()
            elif(self.gaze_state == GazeBehavior.FOLLOW):
                self.__gazeFollowing()
            elif(self.gaze_state == GazeBehavior.AVERSION):
                self.__gazeAversion()
