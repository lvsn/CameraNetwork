#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Abstract class of camera_driver
"""

from abc import ABCMeta, abstractmethod

class camera_driver(object):
    __metaclass__ = ABCMeta


    @abstractmethod
    def capture_image_cb(self,req):
        pass
        
    @abstractmethod
    def capture_video_cb(self,req):
        pass
        
    @abstractmethod
    def load_camera_cb(self,req):
        pass
        
    @abstractmethod
    def set_camera_cb(self,req):
        pass
        
    @abstractmethod
    def get_camera_cb(self,req):
        pass
        
    @abstractmethod
    def calibrate_video_cb(self,req):
        pass