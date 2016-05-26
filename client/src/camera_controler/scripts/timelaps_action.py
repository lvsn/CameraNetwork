#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 21 14:59:35 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Object that implement a Timelapse action : use a camera handler to take picture
in a timelapse manner with HDR capabilities.
"""
import math, datetime

import roslib
roslib.load_manifest('camera_controler')
import rospy
import actionlib
import pysolar

from camera_network_msgs.srv import *
import camera_handler as ch
from camera_network_msgs.msg import *
from scripts.Util.command import *
from scripts.Util.convert import *
from scripts.Util.constant import *
from scripts.Util.miscellaneous import *

class TimelapsAction:

    def __init__(self, cam_handler):
        rospy.loginfo("Setting up Timelapse Action")

        self.picture_count = 0
        self.action = actionlib.SimpleActionServer(
            'timelaps',
            CameraControlAction,
            self.execute,
            False)
        self.action.start()
        self.cam_handler = cam_handler
        self.time_type = 0
        rospy.loginfo("... Timelapse Action Done ...")

    def execute(self, goal):
        self.time_type = goal.time
        periode = abs(goal.inter_picture_delay_s)
        hz = self._sec_to_hz(periode)
        picture_goal = self._get_frame_qty(goal.picture_qty)
        self.picture_qty = goal.picture_qty
        self.picture_count = 0
        self.cam_handler.cam_dl = goal.download

        while self.picture_count < picture_goal:
            timestamp = rospy.get_time() + periode
            self.picture_count += 1

            if self.time_type == 0:
                self._take_picture(goal.mode, self.picture_count)
            elif self.time_type == 1:
                if is_it_day():
                    self._take_picture(goal.mode, self.picture_count)
            elif self.time_type == 2:
                if not is_it_day():
                    self._take_picture(goal.mode, self.picture_count)

            self._send_feedback(self.picture_count, picture_goal, hz)
                        
            if self.cam_handler.cam_dl and ENABLE_PROGRESSIVE_DL:
                self.cam_handler.progressive_dl_cb(True)
            
            if self._sleep(timestamp):
                break

        success_msg = CameraControlActionResult
        success_msg.total_picture = 'Total Picture : ' + str(self.picture_count)
        self.action.set_succeeded(success_msg)

    def _sleep(self, timestamp):
        """
        Sleep until the timestamp (in seconds) if there is an interruption to the action, it return True.
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
            hz = math.fabs(1 / Tsec)
        except ZeroDivisionError:
            hz = -1
        rospy.loginfo("Frequency set to {:.4f} hz.".format(hz))
        return hz

    @staticmethod
    def _get_frame_qty(Qty):
        if Qty < 0:
            frame_qty = float('inf')
        else:
            frame_qty = Qty
        return frame_qty

    def _take_picture(self, mode, pictureId):
        """
        1 - take picture with specific mode
        :param mode: int - 0: single | 1: HDR | 2: shell
        :param pictureId:
        """
        if mode == 1:
            self.cam_handler.takeHDRPicture(pictureId, loadCamera=False)
        elif mode == 2:
            for cmdLine in self.cam_handler.shell_config.splitlines():
                rospy.loginfo('Cmd execute: {}'.format(cmdLine))
                Command.run(cmdLine)
        else:
            self.cam_handler.takeSinglePicture(pictureId, loadCamera=False)

    def _progressive_downloading(self):
        if self.cam_handler.cam_dl:
            rospy.loginfo('downloading begin !')

            self.cam_handler.download_data(0)
        else:
            rospy.loginfo('no downloading !')

    def _send_feedback(self, count, goal, frequency):
        feedback_msg = CameraControlActionFeedback
        feedback_msg.picture_taken = 'Picture taken:' + str(count) \
            + '/' + str(goal) + ' (' + str(frequency) + 'Hz)'
        self.action.publish_feedback(feedback_msg)


if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    camH = ch.CameraHandler()
    action = TimelapsAction(camH)
    rospy.spin()
