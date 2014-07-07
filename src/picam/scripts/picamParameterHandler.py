#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu June 05 12:53:30 2014

@author: Mathieu Garon
"""
import rospy
import picamera
import os

class PicameraParameterHandler:

    def __init__(self):
        self.configLoadedtoRos = False
        self.cameraModel = 'PiCam'
        self.pictureFormat = 'jpeg'
        
        self.supportedFormatList = ['jpeg','png','gif','bmp','yuv','rgb','rgba','bgr','bgra']

        #todo look if camera is here
        rospy.loginfo("Found PiCam")

    def __del__(self):
        if self.configLoadedtoRos:
            try:
                rospy.delete_param("camera_model")
            except KeyError:
                rospy.logerr("Unable to delete parameter (not set)")

    def set_camera_parameters(self):
        rospy.set_param("camera_model",self.cameraModel)
        self.configLoadedtoRos = True


    def set_format(self,formatString):
        if formatString in self.supportedFormatList:
            self.pictureFormat = formatString
        else:
            rospy.logwarn("Format " + formatString + " not supported by Picam")

    def get_format(self):
        return self.pictureFormat

