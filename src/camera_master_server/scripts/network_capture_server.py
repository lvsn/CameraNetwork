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

class network_capture_server:
    
    def __init__(self):
        rospy.loginfo("Initialising Network timlaps server")
        self.publisher = rospy.Publisher("/network_capture_chatter", Capture, queue_size=1)
        rospy.sleep(0.5) #let time for connections
        self.server = actionlib.SimpleActionServer('network_timelaps',CameraControlAction,
                                                   self.execute,False)                                               
        self.server.start()
        
        self.picture_count = 0
        self.msg = Capture()
        self.msg.isHdr = True
        

        
    def execute(self,goal):
        feedback_msg = CameraControlActionFeedback
        hz = 0
        singleShotFlag = False
        try:
            hz = 1/goal.inter_picture_delay_s
            rospy.loginfo("Capturing at a rate of "+str(hz)+" hz")
        except ZeroDivisionError:
            hz = 1
            singleShotFlag = True
            rospy.logwarn("Can't set a 0 delay for Capture")
            
        r = rospy.Rate(hz) # hz 
        if goal.picture_qty < 0:
            picture_goal = float('inf')
        else:
            picture_goal = goal.picture_qty
            
        self.picture_count = 0
        while self.picture_count < picture_goal:
            self.picture_count += 1
            self.publisher.publish(self.msg)
            feedback_msg.picture_taken = 'Picture taken:' + str(self.picture_count)
            self.server.publish_feedback(feedback_msg)
            if self.server.is_preempt_requested() or singleShotFlag:
                break
            r.sleep()
        

        succes_msg = CameraControlActionResult
        succes_msg.total_picture = 'Total Picture : ' + str(self.picture_count)
        self.server.set_succeeded(succes_msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    t = network_capture_server()
    rospy.spin()
