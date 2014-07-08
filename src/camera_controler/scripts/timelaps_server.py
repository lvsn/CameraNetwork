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
import math


class TimelapsServer:
    
    def __init__(self,cam_handler):
        rospy.loginfo("Setting up server")
        
        self.picture_count = 0
        self.server = actionlib.SimpleActionServer('timelaps',CameraControlAction,
                                                   self.execute,False)                                               
        self.server.start()
        self.cam_handler = cam_handler
    
    
    def execute(self,goal):
        
        hz = self._sec_to_hz(goal.inter_picture_delay_s)
        picture_goal = self._get_frame_qty(goal.picture_qty)
        self.picture_count = 0
        r = rospy.Rate(hz)
            
        while self.picture_count < picture_goal:
            self.picture_count += 1
            self._take_picture(goal.is_hdr,self.picture_count)
            self._send_feedback(self.picture_count,picture_goal,hz)
            r.sleep()
            if self.server.is_preempt_requested() or not self.server.is_active() or hz <= 0:
                break
        

        succes_msg = CameraControlActionResult
        succes_msg.total_picture = 'Total Picture : ' + str(self.picture_count)
        self.server.set_succeeded(succes_msg)

    def _sec_to_hz(self,Tsec):
        try:
            hz = math.fabs(1/Tsec)
            rospy.loginfo("Frequency set to " + str(hz) + " hz.")
        except ZeroDivisionError:
            hz = -1
            rospy.logwarn("Can not set 0 as frequency... setting frequency to 1 hz")
        return hz
            
    def _get_frame_qty(self,Qty):
        if Qty < 0:
            frame_qty = float('inf')
        else:
            frame_qty = Qty
        return frame_qty
        
    def _take_picture(self,isHdr,pictureId):
        if(isHdr == True):
            self.cam_handler.takeHDRPicture(pictureId)
        else:
            self.cam_handler.takeSinglePicture(pictureId)
            
    def _send_feedback(self,count,goal,frequency):
        feedback_msg = CameraControlActionFeedback
        feedback_msg.picture_taken = 'Picture taken:' + str(count) \
            + '/' + str(goal) + ' (' + str(frequency) + 'Hz)'
        self.server.publish_feedback(feedback_msg)
        
            

if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    camH = ch.CameraHandler()
    server = TimelapsServer(camH)
    rospy.spin()