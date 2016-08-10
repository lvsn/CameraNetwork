#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 13:05:43 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_master_server')
import rospy
from camera_network_msgs.srv import *
from camera_network_msgs.msg import *
import actionlib
import math

from scripts.Util.miscellaneous import *


class theta_timelapse_action:
    def __init__(self):
        rospy.loginfo("Initialising Theta timlapse action")
        self.optionsPublisher = rospy.Publisher("/theta_options", ThetaOptions, queue_size=1)
        rospy.sleep(0.5)    # let time for connections
        self.action = actionlib.SimpleActionServer('theta_timelapse', ThetaControlAction,
                                                   self.execute, False)
        self.action.start()
        
        self.picture_count = 0
        self.optionsMsg = ThetaOptions()
        
    def execute(self, goal):
        self.optionsMsg.ISO = goal.ISO
        self.optionsMsg.shutterspeed = goal.shutterspeed
        self.optionsMsg.whitebalance = goal.whitebalance
        self.optionsMsg.exposurecompensation = goal.exposurecompensation
        self.optionsMsg.exposureprogram = goal.exposureprogram
        self.optionsPublisher.publish(self.optionsMsg)




if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    t = network_capture_server()
    rospy.spin()
