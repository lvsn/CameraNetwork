#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu September 05 015:49:30 2014

@author: Mathieu Garon
"""

import roslib
roslib.load_manifest('camera_drivers')
import rospy
import os
import rospkg

import camera_driver as cd
import BlueToothKeyboardSimulator as bt


class mobile_server(cd.camera_driver):

    """Handles the requests by ROS to the mobile phone."""

    def __init__(self):
        # ROS functions
        if not os.geteuid() == 0:
            rospy.logfatal('Node must be launched as root!')
        super(mobile_server, self).__init__()
        self.bluetooth = bt.BluetoothKeyBoard(
            7,
            rospkg.RosPack().get_path('camera_drivers') +
            "/scripts/sdp_record.xml")
        self.bluetooth.listen()
        rospy.spin()

    def capture_image_cb(self, req):
        self.bluetooth.raise_volume()
        return []

    def capture_video_cb(self, req):
        rospy.logwarn('Mobile driver does not support video capture.')
        return []

    def calibrate_picture_cb(self, req):
        rospy.logwarn('Mobile driver does not support calibration.')
        return []

    def _copy_picture_from_device_to_standard_directory(self, filename):
        rospy.logwarn('Mobile driver does not support load.')
        return 'error'

    def _delete_picture_from_device(self):
        pass

    def set_camera_cb(self, req):
        rospy.logwarn('Mobile driver does not support set camera.')
        return []

    def get_camera_cb(self, req):
        return {
            'iso': self._add_Choice('', 'None'),
            'imageformat': self._add_Choice('', 'None'),
            'aperture': self._add_Choice('', 'None'),
            'shutterspeed': self._add_Choice('', 'None')}

if __name__ == "__main__":
    rospy.init_node('mobile_driver')
    server = mobile_server()
