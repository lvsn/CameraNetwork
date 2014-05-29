#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 11:08:33 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_controler')
import rospy
import camera_handler as ch
from camera_network_msgs.msg import *

class network_capture_listener:
    
    def __init__(self):
        self.cam_handler = ch.CameraHandler()
        pass
    
    def callback(self,data):
        # Simply print out values in our custom message.
        if data.isHdr:
            rospy.loginfo("Taking hdr picture")
            self.cam_handler.takeHDRPicture(0)
        else:
            rospy.loginfo("Taking single picture")
            self.cam_handler.takeSinglePicture(0)

    def listen(self):
        ns = rospy.get_namespace()
        rospy.loginfo(ns+ " is listening on network_capture_chatter")
        
        rospy.Subscriber('/network_capture_chatter', Capture, self.callback)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_listener')
    # Go to the main loop.
    listener = network_capture_listener()
    rospy.loginfo("listening")
    listener.listen()
    rospy.spin()