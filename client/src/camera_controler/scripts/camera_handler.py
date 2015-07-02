#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Object Facade object for the Camera Driver. Make the use of it easier or
controler's services.
"""
import os
import rospy, subprocess, threading
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
                # --- shell option --- #jb
        rospy.Service(
            'save_config_shell',
            CommandOption,
            self.save_settings_shell_cb)
        rospy.Service(
            'get_config_shell',
            std_srvs.srv.Trigger(),
            self.get_settings_shell_cb)
        self.shell_config = ''

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

    def takeAEBPicture(self, timelapse=120.0):
        rospy.loginfo('*** Stop all AEB process ***')
        self.stopAEBProcess()

        rospy.loginfo('Launch timelaps: {}'.format(timelapse))
        threading.Timer(timelapse, self.captureAEBPicture).start()

        rospy.loginfo('Capturing with AEB')
        self.captureAEBPicture()

    def captureAEBPicture(self):

        rospy.loginfo('*** Take AEB picture ***')

        rospy.loginfo('Lock process')
        subprocess.call('mkdir {}lock'.format(os.path.realpath(__file__)))

        rospy.loginfo('Preset the ISO, fileformat, etc.')
        subprocess.call('gphoto2 --reset', shell=True)
        subprocess.call('gphoto2 --set-config imageformat=32', shell=True)
        rospy.loginfo('\_ Keep photo in SD card')
        subprocess.call('gphoto2 --set-config capturetarget=1', shell=True)
        subprocess.call('gphoto2 --set-config iso=1', shell=True)
        subprocess.call('gphoto2 --set-config autopoweroff="0"', shell=True)

        rospy.loginfo('Taking AEB pictures')
        subprocess.call('gphoto2 --set-config /main/capturesettings/aeb="+/- 3" --set-config /main/capturesettings/aperture="16" --set-config /main/capturesettings/shutterspeed="1/1000" --set-config /main/actions/eosremoterelease=2 --wait-event=1s', shell=True)
        subprocess.call('gphoto2 --set-config /main/capturesettings/aperture=4 --set-config /main/capturesettings/shutterspeed="1/30" --set-config /main/actions/eosremoterelease=2 --wait-event=1s', shell=True)
        subprocess.call('gphoto2 --set-config /main/capturesettings/aeb=0 --set-config /main/capturesettings/shutterspeed="1" --capture-image', shell=True)

        rospy.loginfo('Unlock process')
        subprocess.call('rmdir {}lock'.format(os.path.realpath(__file__)))

    def stopAEBProcess(self):
        rospy.loginfo('AEB stopping begin...')
        str_sub = subprocess.check_output('/bin/ps aux | /bin/grep python | /bin/grep launch_aeb.py', shell=True).splitlines()
        if len(str_sub) > 1:
            pid = str_sub[0].split()[1]
            rospy.loginfo('\_ AEB process present with PID: {}'.format(pid))
            subprocess.call('kill {}'.format(pid), shell=True)
            rospy.loginfo('\_ AEB finished.')
        else:
            rospy.loginfo('\_ AEB not running.')

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

    def save_settings_shell_cb(self, req):
        # --- save_config_shell ---
        rospy.loginfo('Saving new settings shell: {}'.format(req.option))
        self.shell_config = req.option
        return {}

    def get_settings_shell_cb(self, req):
        # --- get_config_shell ---
        if not self.shell_config:
            rospy.logwarn('No current shell settings')
            return {'success': 0, 'message': 'No current shell settings'}
        else:
            rospy.loginfo('Getting shell config')
            return {'success': 1, 'message': self.shell_config}
