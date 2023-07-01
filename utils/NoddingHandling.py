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

class NoddingHandling:


	#Nodding
	hr_nod_variants_range = [1,7] #8 and 9 use lower case "s" in SHORT
	hr_nod_magnitude_range = [0.25,1.0]
	# #Deprecated: now nodding is controlled directly by the dialogue system
	# nodding_mean_interval = 12#Seconds
	# nodding_minimal_interval = 3.0#Seconds
	# noding_thread_rate = 5#Hz
	# pend_nod_text = "Next nod in "
	# no_nod_text = "Nodding disabled."


    def __init__(self, config_data, logger):
        #misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)


	def __toggleNodding(self, enabled):
		if(enabled):
			#do nogging once
			self.__nodOnce()
			LOGGER.info("Nodding")

		#Deprecated: now nodding is controlled directly by the dialogue system
		# self.nodding_enabled = enabled
		# if(self.nodding_enabled):
		# 	self.__setupNoddingTime(time.time())
		# 	LOGGER.info("Nodding Enabled.")
		# else:
		# 	LOGGER.info("Nodding Disabled.")

	def __toggleNoddingMsgCallback(self, msg):
		#Nodding is a sub function of attention
		self.__toggleNodding(self.vision_enabled and msg.data)

	def __nodOnce(self):
		#Compose and publish a nodding command
		nodding_gesture_cmd = hr_msgs.msg.SetAnimation()
		nodding_gesture_cmd.name = self.__config_data['Ros']['hr_nod_type_string'] + str(numpy.random.randint(self.hr_nod_variants_range[0],self.hr_nod_variants_range[1]+1))
		nodding_gesture_cmd.repeat = 1
		nodding_gesture_cmd.speed = 1.0
		nodding_gesture_cmd.magnitude = numpy.random.uniform(self.hr_nod_magnitude_range[0],self.hr_nod_magnitude_range[1])
		self.hr_head_gesture_pub.publish(nodding_gesture_cmd)


