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
        #self.publisher = rospy.Publisher("/network_capture_chatter", Capture, queue_size=1)
        rospy.sleep(0.5)    # let time for connections
        self.action = actionlib.SimpleActionServer('theta_timelapse', ThetaControlAction,
                                                   self.execute, False)
        self.action.start()
        
        self.picture_count = 0
        #self.msg = Capture()
        #self.msg.mode = True
        
    def execute(self, goal):
        #self.msg.mode = goal.mode
        #self.msg.download = goal.download
        #self.msg.time = goal.time
        #period = goal.inter_picture_delay_s
        #hz = self._sec_to_hz(period)
        #picture_goal = self._get_frame_qty(goal.picture_qty)
        #self.picture_count = 0

        #while self.picture_count < picture_goal:
        #    timestamp = rospy.get_time() + period
        #    self.picture_count += 1
#
 #           self.publisher.publish(self.msg)

  #          self._send_feedback(self.picture_count, picture_goal, hz)
   #         interupt = self._sleep(timestamp)
    #        if interupt:
     #           break
        #succes_msg = CameraControlActionResult
       # succes_msg.total_picture = 'Total Picture : ' + str(self.picture_count)
      #  self.action.set_succeeded(succes_msg)

    def _sleep(self, timestamp):
        """
        Sleep until the timestamp (in seconds) if there is an interuption to the action, it return True.
        it give a resolution of 2 sec.
        :param timestamp:
        :return bool:
        """
        while timestamp > rospy.get_time():
                rospy.sleep(2)
                if self.action.is_preempt_requested(
                ) or not self.action.is_active():
                    return True
        return False

    @staticmethod
    def _sec_to_hz(Tsec):
        try:
            hz = math.fabs(1/Tsec)
            rospy.loginfo("Frequency set to " + str(hz) + " hz.")
        except ZeroDivisionError:
            hz = -1
            rospy.logwarn("Can not set 0 as frequency... setting frequency to 1 hz")
        return hz

    @staticmethod
    def _get_frame_qty(Qty):
        if Qty < 0:
            frame_qty = float('inf')
        else:
            frame_qty = Qty
        return frame_qty
            
    def _send_feedback(self, count, goal, frequency):
        feedback_msg = CameraControlActionFeedback
        feedback_msg.picture_taken = 'Picture taken:' + str(count) \
            + '/' + str(goal) + ' (' + str(frequency) + 'Hz)'
        self.action.publish_feedback(feedback_msg)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('network_capture_talker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    t = network_capture_server()
    rospy.spin()
