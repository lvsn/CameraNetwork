#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Object Facade object for the Camera Driver. Make the use of it easier or
controler's services.
"""
import os, sys, envoy
import rospy, subprocess, threading
import std_srvs.srv
from camera_network_msgs.srv import *
from time import sleep

GREP_CAMERA_PATTERN = '[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{1}[[:alpha:]]{1}[[:digit:]]{4}.CR2'

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

        rospy.loginfo('Setting up CameraHandler Service')
        rospy.Service(
            'save_config_shell',
            CommandOption,
            self.save_settings_shell_cb)

        # rospy.Service(
        #     'get_config_shell',
        #     std_srvs.srv.Trigger(),
        #     self.get_config_shell_cb)

        rospy.Service(
            'download_data',
            std_srvs.srv.Empty(),
            self.download_data_cb)

        self.shell_config = ''
        self.lock = False
        rospy.loginfo('... CameraHandler set up done ...')

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

    def save_settings_shell_cb(self, req):
        # --- save_config_shell ---
        if not self.lock:
            rospy.loginfo('Saving new settings shell: {}'.format(req.option))
            self.shell_config = req.option
        else:
            rospy.logwarn('Camera locked. You cant save your shell settings')
        return {}

    # def get_config_shell_cb(self, req):
    #     # --- get_config_shell ---
    #     if not self.shell_config:
    #         rospy.logwarn('No current shell settings')
    #         return {'success': 0, 'message': 'No current shell settings'}
    #     else:
    #         rospy.loginfo('Getting shell config')
    #         return {'success': 1, 'message': self.shell_config}

    def download_data_cb(self, req):
        rospy.loginfo('*** Service: Downloading raw data')
        self.load_data_cb(10)
        rospy.loginfo('*** Service Done')
        return {}

    def load_data_cb(self, number=None):
        """
        Routine: Loads raw data from camera to local pictures folder
        1 - Waiting for camera using and lock camera
        2 - getting camera list files
        3 - downloading number raw data
        4 - delete it if downloaded and unclaim camera
        5 - unlock camera
        """
        # 1 - Waiting for camera using and claim camera
        self.waiting_for_using()
        self.lock = True

        # 2 - Getting camera list files
        # TODO Check execution time with threading.lock something like this
        try:
            camera_list = self._run_cmd('gphoto2 --list-files').splitlines()
            camera_list_files = [line.split()[1] for line in camera_list if '#' in line]
            i = 0

            if number is None or number > len(camera_list_files):
                number = len(camera_list_files)

            # 3 - Download number=X raw data
            while i < number:

                if len(camera_list_files) == 0:
                    rospy.loginfo('Camera empty')
                    break

                os.chdir('/home/jbecirovski/camnet-output/')
                folder_list_files = [file for file in os.listdir('/home/jbecirovski/camnet-output')]

                if not camera_list_files[0] in folder_list_files:
                    self._run_cmd('gphoto2 --get-file=1', 'get first file in camera')
                    rospy.loginfo('#({}/{}) '.format(i + 1, number) + camera_list_files[0] + ' downloaded to folder')

                folder_list_files = [file for file in os.listdir('/home/jbecirovski/camnet-output')]

                # 4 - Delete image
                if camera_list_files[0] in folder_list_files:
                    self._run_cmd('gphoto2 --delete-file=1 --recurse')
                    #rospy.loginfo(camera_list_files[0] + ' deleted from camera')
                    camera_list_files.pop(0)
                    i += 1
        except:
            err_type, err_tb, e = sys.exc_info()
            rospy.logerr(err_tb)

        # 5 - unlock
        self.lock = False

    def organize_raw_data(self):
        """
        Routine: Organises raw data in local pictures folder
        """
        pass

    def send_raw_data(self):
        """
        Routine: Sends raw data from local pictures folder to another folder using rsync
        """
        pass

    def waiting_for_using(self):
        """
        Routine: Waits until camera is no longer used
        """
        while True:
            if self.lock:
                sleep(1)
            else:
                break

    def _run_cmd(self, cmd, category='Unspecified CameraError'):
        """
        Run command and raise if error is detected.
        :param cmd: str - shell command
        :param category: str - choose your category
        :return: str - shell output
        """
        cmd_output = envoy.run(cmd)
        if cmd_output.status_code:
            rospy.logwarn(category + ':\n' + cmd)
            raise AssertionError(category + ': ' + cmd_output.std_err)
        else:
            return cmd_output.std_out
