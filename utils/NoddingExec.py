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
import numpy

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

class NoddingExec:

	def __init__(self, config_data, logger):
		#Misc
		self.__config_data = config_data
		self.__logger = logger.getChild(self.__class__.__name__)

		'''
			ROS io
		'''
		self.__hr_head_gesture_pub = rospy.Publisher(
										self.__config_data['HR']['GazeHead']['hr_head_gesture_topic'], 
										hr_msgs.msg.SetAnimation, 
										queue_size=self.__config_data['Custom']['Ros']['queue_size'])

		'''
			Head Gesture
		'''
		#Nodding
		self.__nod_prefix_string = self.__config_data['HR']['GazeHead']['hr_nod_common_string']
		self.__nod_type_strings = self.__config_data['BehavExec']['HeadGazeGes']['nod_type_strings']
		self.__nod_mag_range = self.__config_data['BehavExec']['HeadGazeGes']['nod_mag_range']


	'''
		#Interface
	'''
	def nodOnce(self):
		#Compose and publish a nodding command
		#For now the parameter of this nodding gesture is randomized locally
		nodding_gesture_cmd = hr_msgs.msg.SetAnimation()
		nodding_gesture_cmd.name = self.__nod_prefix_string + str(numpy.random.randint(self.__nod_type_strings[0],self.__nod_type_strings[1]+1))
		nodding_gesture_cmd.repeat = 1
		nodding_gesture_cmd.speed = 1.0
		nodding_gesture_cmd.magnitude = numpy.random.uniform(self.__nod_mag_range[0],self.__nod_mag_range[1])
		self.__hr_head_gesture_pub.publish(nodding_gesture_cmd)
		#Dummy sleep
		time.sleep(self.__config_data['BehavExec']['HeadGazeGes']['nod_dummy_dur'])
