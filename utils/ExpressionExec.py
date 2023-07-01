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

class ExpressionExec:

    def __init__(self, config_data, logger):
        #misc
        self.__config_data = config_data
        self.__logger = logger.getChild(self.__class__.__name__)

        #Ros io
        self.__expression_pub = rospy.Publisher(self.__config_data['Ros']['expression_topic'], hr_msgs.msg.SetExpression, queue_size=self.__config_data['Ros']['queue_size'])



    def triggerExpressionFixedDur(self, name, dur, magnitude):
        #Compose a message
        msg = hr_msgs.msg.SetExpression()
        msg.name = str(name)
        msg.duration = rospy.Duration(dur,0)
        msg.magnitude = float(magnitude)

        #Publish
        self.__expression_pub.publish(msg)



