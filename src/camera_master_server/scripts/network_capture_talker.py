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
    
    def __init__(self,freq_hz = 0,isHdr = False):
        rospy.loginfo("Publish Capture message")
        self.publisher = rospy.Publisher("/network_capture_chatter", Capture)
        rospy.sleep(0.5) #let time for connections
        self.msg = Capture()
        self.msg.isHdr = isHdr
        self.frequency_hz = freq_hz
        

        
    def set_hdr(self,isHdr):
        self.msg.isHdr = isHdr

    def set_frequency(self, freq_hz):
        self.frequency_hz = freq_hz
        
        
    def publish(self):
        if self.frequency_hz <= 0:
            self._publish_once()
        else:
            self._publish_loop()
            
    def _publish_once(self):
        self.publisher.publish(self.msg)
        
    def _publish_loop(self):
        r = rospy.Rate(self.frequency_hz) # hz
        while not rospy.is_shutdown():
            self.publisher.publish(self.msg)
            r.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    t = network_capture_talker(freq_hz = 1)
    
    t.publish()