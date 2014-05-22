#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 10:18:13 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_controler')
import rospy
import actionlib
from camera_controler.msg import *

def print_feedback(feedback):
    rospy.loginfo(str(feedback))

if __name__ == '__main__':
    rospy.init_node('timelaps_client')
    client = actionlib.SimpleActionClient('/pi0/timelaps', CameraControlAction)
    client.wait_for_server()

    goal = CameraControlGoal()
    goal.picture_qty = 4
    goal.inter_picture_delay_s = 5
    rospy.loginfo("Goal Sent!")
    client.send_goal(goal,feedback_cb = print_feedback)
    client.wait_for_result()