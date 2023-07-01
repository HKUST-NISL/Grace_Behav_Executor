import yaml
import rospy
import os
import re
import threading

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



#Load configs
def loadConfigs():
    #Load configs
    with open("./config/config.yaml", "r") as config_file:
        configs = yaml.load(config_file, Loader=yaml.FullLoader)
        print("Read successful")
    return configs

configs = loadConfigs()


if __name__ == '__main__':
    #Ros routine
    rospy.init_node("exec_test")
    grace_behavior_client = rospy.ServiceProxy(
        configs['Ros']['grace_behavior_service'], 
        grace_attn_msgs.srv.GraceBehavior)

    #Prepare a service request
    req = grace_attn_msgs.srv.GraceBehaviorRequest()
    req.command = configs['General']['comp_behav_exec_cmd']
    req.utterance = configs['CompositeBehavior']['Predefined']['Sample']['txt']
    req.lang = configs['CompositeBehavior']['Predefined']['Sample']['lang']

    req.expressions = configs['CompositeBehavior']['Predefined']['Sample']['expressions']
    req.exp_start = configs['CompositeBehavior']['Predefined']['Sample']['exp_start']
    req.exp_end = configs['CompositeBehavior']['Predefined']['Sample']['exp_end']
    req.exp_mag = configs['CompositeBehavior']['Predefined']['Sample']['exp_mag']

    req.gestures = configs['CompositeBehavior']['Predefined']['Sample']['gestures']
    req.ges_start = configs['CompositeBehavior']['Predefined']['Sample']['ges_start']
    req.ges_end = configs['CompositeBehavior']['Predefined']['Sample']['ges_end']
    req.ges_mag = configs['CompositeBehavior']['Predefined']['Sample']['ges_mag']

    #Call the service
    print("Service call response is:\n %s" % grace_behavior_client(req))






