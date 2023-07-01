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

#Specific
from . import GazeBehavExec

class HeadGazeGesExec:

	def __init__(self, config_data, logger):
		#Misc
		self.__config_data = config_data
		self.__logger = logger.getChild(self.__class__.__name__)

		'''
			ROS io
		'''
		self.__hr_head_gesture_pub = rospy.Publisher(self.__config_data['Ros']['hr_head_gesture_topic'], hr_msgs.msg.SetAnimation, queue_size=self.__config_data['Ros']['queue_size'])



		'''
			Head Gesture
		'''
		#Nodding
		self.__nod_prefix_string = self.__config_data['Ros']['hr_nod_common_string']
		self.__nod_type_strings = self.__config_data['HeadGazeGes']['nod_type_strings']
		self.__nod_mag_range = self.__config_data['HeadGazeGes']['nod_mag_range']


		'''
			Gaze
		'''
		self.__gaze_exec = GazeBehavExec.GazeBehavExec(self.__config_data, self.__logger)




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


	def startFollowing(self):
		self.__gaze_exec.gaze_state = GazeBehavExec.GazeBehavior.FOLLOW

	def startAverting(self):
		self.__gaze_exec.gaze_state = GazeBehavExec.GazeBehavior.AVERSION

	def goToNeutral(self):
		self.__gaze_exec.gaze_state = GazeBehavExec.GazeBehavior.NEUTRAL
