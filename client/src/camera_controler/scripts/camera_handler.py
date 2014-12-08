#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Object Facade object for the Camera Driver. Make the use of it easier or
controler's services.
"""
import rospy
import std_srvs.srv
from camera_network_msgs.srv import *


class CameraHandler:
    """
    CameraHandler manipulate low level driver service.
    It give a set of high level function for the user.
    """
    def __init__(self):
        rospy.loginfo("Setting up camera Handler")
        rospy.wait_for_service('get_camera')
        rospy.wait_for_service('set_camera')
        rospy.wait_for_service('capture_camera')
        rospy.wait_for_service('load_camera')
        rospy.wait_for_service('capture_video')
        rospy.wait_for_service('calibrate_picture')
        rospy.wait_for_service('configure_parameter_queue')
        rospy.wait_for_service('preview_camera_driver')

        self.get_camera_service = rospy.ServiceProxy(
            'get_camera',
            OutCameraData)
        self.capture_camera_service = rospy.ServiceProxy(
            'capture_camera',
            CaptureService)
        self.preview_camera_service = rospy.ServiceProxy(
            'preview_camera_driver',
            std_srvs.srv.Empty
        )
        self.set_camera_service = rospy.ServiceProxy(
            'set_camera',
            InCameraData)
        self.load_camera_service = rospy.ServiceProxy('load_camera', Load)
        self.capture_video_service = rospy.ServiceProxy(
            'capture_video',
            Uint32)
        self.calibrate_picture_service = rospy.ServiceProxy(
            'calibrate_picture',
            std_srvs.srv.Empty)
        self.configure_parameter_queue_service = rospy.ServiceProxy(
            'configure_parameter_queue',
            ParameterQueue)
        self.updateCameraSetting()

    def updateCameraSetting(self, configDict={}):
        '''
        update parameter server
        configDict must contain the supported key to update parameter server:
            ex: if only 'iso' is present, only the iso parameter will be updated
        '''

        if 'iso' in configDict:
            rospy.set_param('camera_setting/iso', configDict['iso'])

        if 'imageformat' in configDict:
            rospy.set_param(
                'camera_setting/imageformat',
                configDict['imageformat'])

        if 'shutterspeed' in configDict:
            rospy.set_param(
                'camera_setting/shutterspeed',
                configDict['shutterspeed'])

        if 'aperture' in configDict:
            rospy.set_param('camera_setting/aperture', configDict['aperture'])

        setting = rospy.get_param('camera_setting')
        try:
            self.set_camera_service(
                setting['iso'],
                setting['imageformat'],
                setting['aperture'],
                setting['shutterspeed'])
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)

    def updateParameterQueue(self, configDict, flush=False):
        if 'iso' not in configDict:
            configDict['iso'] = ''
        if 'imageformat' not in configDict:
            configDict['imageformat'] = ''
        if 'aperture' not in configDict:
            configDict['aperture'] = ''
        if 'shutterspeed' not in configDict:
            configDict['shutterspeed'] = ''
        try:
            self.configure_parameter_queue_service(
                configDict['iso'],
                configDict['imageformat'],
                configDict['aperture'],
                configDict['shutterspeed'],
                flush)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)


    def takeSinglePicture(self, pictureId, setCamera=True, loadCamera=False):
        pictureName = str(pictureId)
        # picture path ex : pictureId-n_23May14_10h30m00s.jpg  (n depend on
        # camera's picture qty)
        picturePath = self._generatePictureName(pictureName)
        if setCamera:
            self.updateCameraSetting()
        self.capture_camera_service(0)
        if loadCamera:
            self.load_camera_service(picturePath)

    def takeHDRPicture(self, pictureId, loadCamera=False):
        settingList = rospy.get_param('camera_setting/captureSequence')
        pictureName = str(pictureId)
        picturePath = self._generatePictureName(pictureName)
        for setting in settingList:
            self.updateParameterQueue(setting)
        self.capture_camera_service(0)
        if loadCamera:
            self.load_camera_service(picturePath)

    def takeVideo(self, time):
        self.capture_video_service(time)
        picture = self._generatePictureName('video')
        self.load_camera_service(picture)

    def takePreview(self):
        self.preview_camera_service()

    def calibrate(self):
        self.calibrate_picture_service()
        setting = self.get_camera_service(False)
        settingDict = {}
        settingDict['iso'] = setting.iso
        settingDict['shutterspeed'] = setting.shutterspeed
        settingDict['aperture'] = setting.aperture
        self.updateCameraSetting(settingDict)

    def _generatePictureName(self, name):
        return '%B/{name}-%n_%Y-%m-%d_%Hh%Mm%Ss.%f.%C'.format(**locals())
