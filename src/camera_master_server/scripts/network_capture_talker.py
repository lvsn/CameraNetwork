#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 13:05:43 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_master_server')
import rospy
from camera_network_msgs.msg import *

class network_capture_talker:
    
    def __init__(self):
        rospy.loginfo("Publish Capture message")
        pub = rospy.Publisher("/network_capture_chatter", Capture)
        msg = Capture()
        msg.isHdr = False
        pub.publish(msg)
        while not rospy.is_shutdown():
            #pub.publish(msg)
            rospy.sleep(3.0)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    t = network_capture_talker()