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
        
        self.picture_qty = 0
        self.inter_picture_delay_s = 0
        self.server = actionlib.SimpleActionServer('timelaps',CameraControlAction,
                                                   self.execute,False)
        self.server.start()
        self.cam_handler = ch.CameraHandler()
    
    
                        
    
    def execute(self,goal):

        self.picture_qty = goal.picture_qty
        self.inter_picture_delay_s = goal.inter_picture_delay_s


        feedback_msg = CameraControlActionFeedback
        count = 1
        while self.picture_qty > 0:
            self.cam_handler.takeSinglePicture()
            feedback_msg.picture_taken = 'Picture taken:' + str(count)
            self.server.publish_feedback(feedback_msg)
            rospy.sleep(self.inter_picture_delay_s)
            count += 1
            self.picture_qty -= 1
        

        
        self.server.set_succeeded()



if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    server = TimelapsServer()
    rospy.spin()