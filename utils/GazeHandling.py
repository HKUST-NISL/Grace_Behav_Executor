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

class GazeHandling:

	#Aversion
	aversion_enabled = False #Whether the random aversion is enabled
	is_gaze_averting = False #Whether it's averting at the moment
	gaze_aversion_start_time = None
	gaze_aversion_stop_time = None
	gaze_aversion_target_msg = None
	aversion_target_id = "gaze_aversion_target"
	aversion_loc_range = [0.1,0.3]
	aversion_mean_interval = 10#seconds
	aversion_duration_range = [1,2]#seconds
	aversion_minimal_interval = 3.0#Seconds
	aversion_thread_rate = 5#Hz
	tracking_text = "Tracking. "
	no_aversion_text = "Aversion disabled."
	averting_text = "Averting for"




    def __init__(self, config_data, logger):
        #Misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #ROS IO
		self.hr_face_target_pub = rospy.Publisher(self.__config_data['Ros']['hr_face_target_topic'], hr_msgs.msg.Target, queue_size=self.__config_data['Ros']['queue_size'])
		self.hr_head_gesture_pub = rospy.Publisher(self.__config_data['Ros']['hr_head_gesture_topic'], hr_msgs.msg.SetAnimation, queue_size=self.__config_data['Ros']['queue_size'])
		self.toggle_aversion_pub = rospy.Publisher(self.__config_data['Ros']['toggle_aversion_topic'], std_msgs.msg.Bool, queue_size=self.__config_data['Ros']['queue_size'])


        #Fake gaze target
		self.fake_target_in_space = hr_msgs.msg.Target()
		self.fake_target_in_space.x = 1.5
		self.fake_target_in_space.y = 0
		self.fake_target_in_space.x = 1.5
		self.fake_target_in_space.speed = 1.0

		self.dynamic_ATTN_cfg_client = dynamic_reconfigure.client.Client(self.__config_data['Ros']['hr_ATTN_cfg_server'], timeout=self.__config_data['Ros']['dynamic_reconfig_request_timeout'], config_callback=self.__configureGraceATTNCallback)


		#Start the gaze aversion thread
		self.grace_aversion_thread = threading.Thread(target = self.__aversionThread,daemon = False)
		self.grace_aversion_thread.start()

		if(person.id == self.aversion_target_id):
			return #Skip the people message containing fake aversion target
				

	def __aversionThread(self):
		rate = rospy.Rate(self.aversion_thread_rate)
		while(True):
			if(self.aversion_enabled):
				t = time.time()

				if(self.is_gaze_averting == False):
					if(t >= self.gaze_aversion_start_time):
						#Create a new aversion target
						self.__createGazeAversionTarget()
						#Indicate the averting state
						self.is_gaze_averting = True
					else:
						#text state display
						try:
							aversion_state_text_msg = std_msgs.msg.String(self.tracking_text + "Next aversion in " + f"{max(0,self.gaze_aversion_start_time - t):.2f}")
							self.aversion_text_pub.publish(aversion_state_text_msg)
						except Exception as e:
							LOGGER.debug(e)
				else:
					if(t >= self.gaze_aversion_end_time):
						#Stop the current averting act
						self.is_gaze_averting = False
						#Decide the timing for the next aversion
						self.__setupAversionTime(t)


				if(self.is_gaze_averting):
					#text state display
					try:
						aversion_state_text_msg = std_msgs.msg.String(self.averting_text + f"{max(0,self.gaze_aversion_end_time - t):.2f}")
						self.aversion_text_pub.publish(aversion_state_text_msg)		
					except Exception as e:
						LOGGER.debug(e)
			else:
				self.is_gaze_averting = False
				#text state display
				try:
					text = ''
					if(self.vision_enabled):
						text = self.tracking_text + self.no_aversion_text
					else:
						text = 'Attention Disabled'
					aversion_state_text_msg = std_msgs.msg.String(text)
					self.aversion_text_pub.publish(aversion_state_text_msg)	
				except Exception as e:
					LOGGER.debug(e)


			rate.sleep()

	def __setupAversionTime(self, ref_time):
		#Start time of aversion
		self.gaze_aversion_start_time = ref_time + max(self.aversion_minimal_interval,numpy.random.exponential(self.aversion_mean_interval))
		#Duration of aversion
		dur = numpy.random.uniform(self.aversion_duration_range[0],self.aversion_duration_range[1])
		#End time of aversion
		self.gaze_aversion_end_time = self.gaze_aversion_start_time + dur
		LOGGER.info("New aversion in %f seconds." % (self.gaze_aversion_start_time - ref_time))

	def __publishGazeAversionTarget(self):
		hr_people = hr_msgs.msg.People()
		hr_people.people.append(self.gaze_aversion_target_msg)
		self.hr_people_pub.publish(hr_people)

	def __createGazeAversionTarget(self):
		if(self.attn_id_now is not None):
			try:
				# # Offset around a person message
				self.gaze_aversion_target_msg = deepcopy(self.target_person_msg)
				self.gaze_aversion_target_msg.id = self.aversion_target_id

				#Perturb the location of the person to create a fake person for aversion
				old_point = self.target_person_msg.body.location
				new_point = geometry_msgs.msg.Point()
				new_point.x = old_point.x + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				new_point.y = old_point.y + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				new_point.z = old_point.z + 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
				# new_point.y = 0.5
				# new_point.x = 0.5
				# new_point.z = 0.5

				self.gaze_aversion_target_msg.body.location = new_point
			except Exception as e:
				LOGGER.error(e)
		else:
			self.gaze_aversion_target_msg = hr_msgs.msg.Person()
			self.gaze_aversion_target_msg.id = self.aversion_target_id
			
			new_point = geometry_msgs.msg.Point()
			new_point.x = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			new_point.y = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			new_point.z = 2 * (numpy.random.binomial(1, 0.5) - 0.5) * numpy.random.uniform(self.aversion_loc_range[0],self.aversion_loc_range[1])
			
			self.gaze_aversion_target_msg.body.location = new_point

	def __configureGraceATTN(self):
		if(self.vision_enabled):
			#If attention is enabled, do attention or aversion
			if(self.is_gaze_averting == False):
				#Attend to the target person normally
				#choose target
				if( self.attn_id_now is not None ):
					try:
						self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.attn_id_now, "look_at_start":True, "look_at_time":self.__config_data['Ros']['hr_ATTN_timeout']})
						LOGGER.debug("Configuring grace attention on target %s." % (self.attn_id_now))
					except Exception as e:
						LOGGER.error(e)
			else:
				#Look at aversion target
				self.__publishGazeAversionTarget()
				try:
					self.dynamic_ATTN_cfg_client.update_configuration({"look_at_face":self.gaze_aversion_target_msg.id, "look_at_start":True, "look_at_time":self.__config_data['Ros']['hr_ATTN_timeout']})
				except Exception as e:
					LOGGER.error(e)

				LOGGER.debug("Gaze Averting.")
		else:
			#If attention is disabled, manually fix the gaze of grace
			try:
				self.hr_face_target_pub.publish(self.fake_target_in_space)
			except Exception as e:
				LOGGER.debug(e)


	def __toggleAversion(self,enabled):
		self.aversion_enabled = enabled
		if(self.aversion_enabled):
			#Setup the first time stamp for aversion
			self.__setupAversionTime(time.time())
			LOGGER.info("Aversion Enabled.")
		else:
			#Disable the aversion mechanism
			LOGGER.info("Aversion Disabled.")




