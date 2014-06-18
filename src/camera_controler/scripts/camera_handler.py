#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
"""
import rospy
from camera_network_msgs.srv import * 

class CameraHandler:
    
    def __init__(self):
        rospy.loginfo("Setting up camera Handler")        
        rospy.wait_for_service('set_camera')
        rospy.wait_for_service('capture_camera')
        rospy.wait_for_service('load_camera')
         
        self.capture_camera_service = rospy.ServiceProxy('capture_camera', CaptureService)
        self.set_camera_service = rospy.ServiceProxy('set_camera', InCameraData)
        self.load_camera_service = rospy.ServiceProxy('load_camera', Load)
        self.updateCameraSetting()
        
        
    #def __del__(self):
        #rospy.delete_param('camera_setting')
        #rospy.delete_param('file')
        
                        
    def updateCameraSetting(self,configDict = {}):
        '''
        update parameter server
        configDict must contain the supported key to update parameter server:
            ex: if only 'iso' is present, only the iso parameter will be updated
        '''
        
        if 'iso' in configDict:
            rospy.set_param('camera_setting/iso',configDict['iso'])
            
        if 'imageformat' in configDict:
            rospy.set_param('camera_setting/imageformat',configDict['imageformat'])
            
        if 'shutterspeed' in configDict:
            rospy.set_param('camera_setting/shutterspeed',configDict['shutterspeed'])
            
        if 'aperture' in configDict:
            rospy.set_param('camera_setting/aperture',configDict['aperture'])
            
        setting = rospy.get_param('camera_setting')
        try:
            self.set_camera_service(setting['iso'],setting['imageformat'],
                        setting['aperture'],setting['shutterspeed'])
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s",e)
        
    def takeSinglePicture(self,pictureId,setCamera = True, loadCamera = True):
        settingList = rospy.get_param('camera_setting/captureSequence')
        pictureName = str(pictureId)
        #picture path ex : pictureId-n_23May14_10h30m00s.jpg  (n depend on camera's picture qty)
        picturePath = self._generatePictureName(pictureName)
        pictureSetting = settingList[0]
        if setCamera:
            self.updateCameraSetting(pictureSetting)
        self.capture_camera_service('dummy')
        if loadCamera:
            self.load_camera_service(picturePath)
        
    def takeHDRPicture(self,pictureId,setCamera = True, loadCamera = True):
        settingList = rospy.get_param('camera_setting/captureSequence')
        pictureName = str(pictureId)
        #picture path ex : pictureId-n_23May14_10h30m00s.jpg  (n depend on camera's picture qty)
        picturePath = self._generatePictureName(pictureName)   
        for setting in settingList:
            if setCamera:
                self.updateCameraSetting(setting)
            self.capture_camera_service('dummy')
        if loadCamera:
            self.load_camera_service(picturePath)
            
    def takePreview(self,configDict={}):
        self.updateCameraSetting(configDict)
        picturePath = 'preview/send.%C'
        self.capture_camera_service('dummy')
        self.load_camera_service(picturePath)
     
    def _generatePictureName(self,name):
        return '%B/' + name + '-%n_%d%B%y_%Hh%Mm%Ss.%C' 
        
