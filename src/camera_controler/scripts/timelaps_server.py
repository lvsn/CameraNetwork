#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 21 14:59:35 2014

@author: mathieugaron
"""
import roslib; roslib.load_manifest('camera_controler')
import rospy
import actionlib
import camera_handler as ch
from camera_controler.msg import *

class TimelapsServer:
    
    def __init__(self):
        rospy.loginfo("Setting up server")
        
        self.picture_count = 0
        self.server = actionlib.SimpleActionServer('timelaps',CameraControlAction,
                                                   self.execute,False)                                               
        self.server.start()
        self.cam_handler = ch.CameraHandler()
    
    
                        
        
    def execute(self,goal):

        feedback_msg = CameraControlActionFeedback
        r = rospy.Rate(1/goal.inter_picture_delay_s) # hz
        self.picture_count = 0
        while self.picture_count < goal.picture_qty:
            self.picture_count += 1
            self.cam_handler.takeHDRPicture(self.picture_count)
            feedback_msg.picture_taken = 'Picture taken:' + str(self.picture_count)
            self.server.publish_feedback(feedback_msg)
            if self.server.is_preempt_requested():
                break
            r.sleep()
        

        
        self.server.set_succeeded()



if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    server = TimelapsServer()
    rospy.spin()