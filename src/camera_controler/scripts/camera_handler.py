#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
"""
import rospy
from gphoto_cam.srv import *  #TODO make it generic (another package)

class CameraHandler:
    
    def __init__(self):
        rospy.loginfo("Setting up camera Handler")        
        rospy.wait_for_service('set_camera')
        rospy.wait_for_service('capture_camera')
        
        self.capture_camera_service = rospy.ServiceProxy('capture_camera', Capture)
        self.set_camera_service = rospy.ServiceProxy('set_camera', InCameraData)
        self.updateCameraSetting()
        
                        
    def updateCameraSetting(self,configDict = {}):
        '''
        update parameter server
        configDict must contain the supported key to update parameter server:
            ex: if only 'iso' is present, only the iso parameter will be updated
        '''
        #TODO Look for race condition!
        
        if configDict.has_key('iso'):
            rospy.set_param('camera_actual_settings/iso',configDict['iso'])
            
        if configDict.has_key('imageformat'):
            rospy.set_param('camera_actual_settings/imageformat',configDict['imageformat'])
            
        if configDict.has_key('shutterspeed'):
            rospy.set_param('camera_actual_settings/shutterspeed',configDict['shutterspeed'])
            
        if configDict.has_key('aperture'):
            rospy.set_param('camera_actual_settings/aperture',configDict['aperture'])
            
        setting = rospy.get_param('camera_actual_settings') 
        self.set_camera_service(setting['iso'],setting['imageformat'],
                        setting['aperture'],setting['shutterspeed'])
        
    def takeSinglePicture(self):
        settingList = rospy.get_param('camera_capture_settings')
        pictureSetting = settingList[0]
        self.updateCameraSetting(pictureSetting)
        self.capture_camera_service(True)
        
    def takeHDRPicture(self):
        settingList = rospy.get_param('camera_capture_settings')
        for setting in settingList:
            self.updateCameraSetting(setting)
            self.capture_camera_service(True)
        
        