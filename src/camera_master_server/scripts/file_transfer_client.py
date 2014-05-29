#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 10:52:39 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_controler')
import rospy
import actionlib
from camera_controler.msg import *

def print_feedback(feedback):
    rospy.loginfo(str(feedback))

if __name__ == '__main__':
    rospy.init_node('sftp_client')
    client = actionlib.SimpleActionClient('sftp', CameraDownloadAction)
    client.wait_for_server()

    goal = CameraDownloadGoal()
    goal.dowload_frequency_s = 0
    rospy.loginfo("Goal Sent!")
    client.send_goal(goal,feedback_cb = print_feedback)

    client.wait_for_result()