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
import sys
import os
from inspect import getsourcefile



if __name__ == '__main__':

    file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
    sys.path.append(os.path.join(file_path, '..'))
    from CommonConfigs.grace_cfg_loader import *
    configs = loadGraceConfigs()

    #Ros routine
    rospy.init_node("exec_test")
    grace_behavior_client = rospy.ServiceProxy(
        configs['Custom']['Behavior']['grace_behavior_service'], 
        grace_attn_msgs.srv.GraceBehavior)





    # #(1) Test Following, Aversion
    # req = grace_attn_msgs.srv.GraceBehaviorRequest()
    # req.command = configs['BehavExec']['General']['head_gaze_avert']
    # # req.command = configs['BehavExec']['General']['head_gaze_follow']
    # print("Service call response is:\n %s" % grace_behavior_client(req))
    



    #(2) Test Nodding
    req = grace_attn_msgs.srv.GraceBehaviorRequest()
    req.command = configs['BehavExec']['General']['nod_cmd']
    print("Service call response is:\n %s" % grace_behavior_client(req))




    # #(3) Test Composite
    # req = grace_attn_msgs.srv.GraceBehaviorRequest()
    # req.command = configs['BehavExec']['General']['comp_behav_exec_cmd']
    # # req.command = configs['BehavExec']['General']['hum_behav_exec_cmd']
    # req.utterance = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['txt']
    # req.lang = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['lang']

    # req.expressions = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['expressions']
    # req.exp_start = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['exp_start']
    # req.exp_end = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['exp_end']
    # req.exp_mag = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['exp_mag']

    # req.gestures = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['gestures']
    # req.ges_start = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['ges_start']
    # req.ges_end = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['ges_end']
    # req.ges_mag = configs['BehavExec']['CompositeBehavior']['Predefined']['Sample']['ges_mag']

    # print("Service call response is:\n %s" % grace_behavior_client(req))





