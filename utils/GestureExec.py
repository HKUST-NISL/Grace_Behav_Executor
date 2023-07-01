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




class GestureExec:

    def __init__(self, config_data, logger):
        #misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #Ros io
        self.__arm_animation_pub = rospy.Publisher(self.__config_data['Ros']['arm_animation_topic'], hr_msgs.msg.SetAnimation, queue_size=self.__config_data['Ros']['queue_size'])
        self.__arm_animation_normal_length = rospy.ServiceProxy(self.__config_data['Ros']['arm_animation_normal_length_service'], hr_msgs.srv.GetAnimationLength)
        self.__arm_animation_reconfig_client = dynamic_reconfigure.client.Client(self.__config_data['Ros']['arm_animation_motor_speed_reconfig']) 

        #Gesture execution configs
        self.__min_arm_anim_key_frame_transition = self.__config_data['Behavior']['arm_anim_min_motor_transition_time']
        self.__default_arm_anim_key_frame_transition = self.__config_data['Behavior']['arm_anim_motor_transition_time']

        #Initialize
        self.__configAnimationMotorSpeed()





    '''
        Helpers
    '''
    def __configAnimationMotorSpeed(self, state_transition_dur = self.__default_arm_anim_key_frame_transition):
        #This is the state transition time for the motor to achieve when going from one key frame to another key frame

        #Compare with min state transition for safety
        if(state_transition_dur < self.__min_arm_anim_key_frame_transition ):
            state_transition_dur_rectified = self.__min_arm_anim_key_frame_transition 
        else:
            state_transition_dur_rectified = state_transition_dur

        #Reconfigure HRSDK
        params = { 'arm_animation_transition': state_transition_dur_rectified } 
        self.__arm_animation_reconfig_client.update_configuration(params)

    def __queryArmAnimationNormalLength(self, name):
        #Compose a request
        req = hr_msgs.srv.GetAnimationLengthRequest()
        req.animation = name

        res = self.__arm_animation_normal_length(req)

        #Return the normal playback duration at playback speed 1
        return res.length

    def __calcArmAnimationPlaybackSpeed(self, normal_dur, dur_in):
        #The playback duration should be NO LESS than motor state transition duration
        if(dur_in < self.__min_arm_anim_key_frame_transition ):
            dur_rectified = self.__min_arm_anim_key_frame_transition
        else:
            dur_rectified = dur_in
        #Compute playback speed that will achieve this duration
        playback_speed_ratio = normal_dur / dur_rectified

        return playback_speed_ratio

    def __triggerArmAnimation(self, name, speed, magnitude):
        #Compose a message
        msg = hr_msgs.msg.SetAnimation()
        msg.name = name
        msg.speed = speed
        msg.magnitude = magnitude

        #Publish
        self.__arm_animation_pub.publish(msg)

    '''
        Interface
    '''
    def triggerArmAnimationFixedDur(self, name, dur, magnitude):
        #Arm gesture execution is open-loop

        #Normal duration with playback speed 1 - NOT MOTOR SPEED
        normal_dur = self.__queryArmAnimationNormalLength(name)

        #Compute the speed to reach the animation duration specified in the input
        playback_speed_ratio = self.__calcArmAnimationPlaybackSpeed(normal_dur, dur)

        self.__triggerArmAnimation(str(name), playback_speed_ratio, magnitude)



