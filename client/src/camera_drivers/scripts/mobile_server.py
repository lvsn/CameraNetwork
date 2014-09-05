#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu September 05 015:49:30 2014

@author: Mathieu Garon
"""

import roslib
roslib.load_manifest('camera_drivers')
import rospy

import camera_driver as cd



class mobile_server(cd.camera_driver):

    """Handles the requests by ROS to the mobile phone."""

    def __init__(self):
        # ROS functions
        super(mobile_server, self).__init__()
        rospy.spin()

    def __del__(self):
        pass

    def capture_image_cb(self, req):
        pass

    def stream_video_cb(self, req):
        pass

    def capture_video_cb(self, req):
        pass

    def calibrate_picture_cb(self, req):
        pass

    def load_camera_cb(self, req):
        pass

    def set_camera_cb(self, req):
        pass

    def get_camera_cb(self, req):
        pass

if __name__ == "__main__":
    rospy.init_node('mobile_driver')
    server = mobile_server()