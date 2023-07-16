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



class TTSExec:

    def __init__(self, config_data, logger):
        #misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #Ros io
        self.__tts_data_client = rospy.ServiceProxy(
                                            self.__config_data['HR']['TTS']['tts_data_service'], hr_msgs.srv.TTSData)
        self.__tts_control_pub = rospy.Publisher(
                                            self.__config_data['HR']['TTS']['tts_control_topic'], 
                                            std_msgs.msg.String, 
                                            queue_size=self.__config_data['Custom']['Ros']['queue_size'])
        self.__tts_say_client = rospy.ServiceProxy(
                                            self.__config_data['HR']['TTS']['tts_say_service'], 
                                            hr_msgs.srv.TTSTrigger)
        self.__tts_event_sub = rospy.Subscriber(
                                            self.__config_data['HR']['TTS']['tts_event_topic'], 
                                            std_msgs.msg.String, 
                                            self.__ttsEventCallback, 
                                            queue_size=self.__config_data['Custom']['Ros']['queue_size'])

        #TTS execution configs
        self.__tts_language_code = self.__config_data['BehavExec']['TTS']['tts_language_code']
        self.__tts_stop_cmd = self.__config_data['HR']['TTS']['tts_stop_cmd']
        self.__tts_end_event_string = self.__config_data['HR']['TTS']['tts_end_event_string']
        self.__tts_pure_token = self.__config_data['HR']['TTS']['tts_pure_token']
        self.__latest_tts_event_string = ''


    '''
        Helpers
    '''
    def __pubTTSCtrlCmd(self, cmd):
        #Compose a message
        msg = std_msgs.msg.String()
        msg.data = cmd

        #Publish
        self.__tts_control_pub.publish(msg)

    def __ttsEventCallback(self, msg):
        self.__latest_tts_event_string = msg.data
        self.__logger.debug('Latest TTS event is \"%s\".' % self.__latest_tts_event_string)



    '''
        Interface
    '''
    def stopTTS(self):
        self.__pubTTSCtrlCmd(self.__tts_stop_cmd)

    def postEditTTSText(self, raw_text):
        #We don't need auto-generated expressions and gestures
        edited_text = self.__tts_pure_token + raw_text
        return edited_text

    def getTTSData(self, text, lang):
        #Compose a request
        req = hr_msgs.srv.TTSDataRequest()
        req.txt = text
        req.lang = lang

        #Call the service
        res = self.__tts_data_client(req)
        return res

    def parseTTSDur(self, tts_data_response):
        #Only one occurence by default
        dur_text = re.search('\"duration\": [0123456789.]+,', tts_data_response.data)
        dur = float(re.search('[0123456789.]+',dur_text.group()).group())
        return dur

    def say(self, text, lang):
        #Clear event string
        self.__latest_tts_event_string = ''

        #Compose a request
        req = hr_msgs.srv.TTSTriggerRequest()
        req.text = text
        req.lang = lang

        #Call the service
        return self.__tts_say_client(req)

    def receivedTTSEndEvent(self):
        return (self.__latest_tts_event_string == self.__tts_end_event_string)