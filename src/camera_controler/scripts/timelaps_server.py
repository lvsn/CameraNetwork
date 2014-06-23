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
from camera_network_msgs.msg import *


class TimelapsServer:
    
    def __init__(self,cam_handler):
        rospy.loginfo("Setting up server")
        
        self.picture_count = 0
        self.server = actionlib.SimpleActionServer('timelaps',CameraControlAction,
                                                   self.execute,False)                                               
        self.server.start()
        self.cam_handler = cam_handler
    
    
    def execute(self,goal):

        feedback_msg = CameraControlActionFeedback
        hz = 0
        try:
            hz = 1/goal.inter_picture_delay_s
            rospy.loginfo("Capturing at a rate of "+str(hz)+" hz")
        except ZeroDivisionError:
            hz = 1
            rospy.logwarn("Can't set a 0 delay for Capture")
            
        r = rospy.Rate(hz) # hz 
        if goal.picture_qty < 0:
            picture_goal = float('inf')
        else:
            picture_goal = goal.picture_qty
            
        self.picture_count = 0
        while self.picture_count < picture_goal:
            self.picture_count += 1
            self.cam_handler.takeHDRPicture(self.picture_count)
            feedback_msg.picture_taken = 'Picture taken:' + str(self.picture_count)
            self.server.publish_feedback(feedback_msg)
            r.sleep()
            if self.server.is_preempt_requested() or not self.server.is_active():
                break
        

        succes_msg = CameraControlActionResult
        succes_msg.total_picture = 'Total Picture : ' + str(self.picture_count)
        self.server.set_succeeded(succes_msg)



if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    camH = ch.CameraHandler()
    server = TimelapsServer(camH)
    rospy.spin()