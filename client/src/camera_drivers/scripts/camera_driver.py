#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Abstract class of camera_driver
"""
from abc import ABCMeta, abstractmethod

import rospy
import std_srvs.srv
from CameraParameterHandler import *
from camera_network_msgs.srv import *


class camera_driver(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        self.cameraModel = None
        rospy.Service('capture_camera', CaptureService, self.capture_image_cb)
        rospy.Service('get_camera', OutCameraData, self.get_camera_cb)
        rospy.Service('set_camera', InCameraData, self.set_camera_cb)
        rospy.Service('load_camera', Load, self.load_camera_cb)
        rospy.Service('capture_video', Uint32, self.capture_video_cb)
        rospy.Service(
            'calibrate_picture',
            std_srvs.srv.Empty,
            self.calibrate_video_cb)

    def __del__(self):
        if self.cameraModel:
            try:
                rospy.delete_param("camera_model")
            except KeyError:
                rospy.logerr("Unable to delete parameter (not set)")

    @abstractmethod
    def capture_image_cb(self, req):
        pass

    @abstractmethod
    def capture_video_cb(self, req):
        pass

    @abstractmethod
    def load_camera_cb(self, req):
        pass

    @abstractmethod
    def set_camera_cb(self, req):
        pass

    @abstractmethod
    def get_camera_cb(self, req):
        pass

    @abstractmethod
    def calibrate_video_cb(self, req):
        pass

    def _set_camera_model(self, camera):
        self.cameraModel = camera
        rospy.set_param("camera_model", self.cameraModel)
