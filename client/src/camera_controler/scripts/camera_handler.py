#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 16:21:09 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Object Facade object for the Camera Driver. Make the use of it easier or
controler's services.
"""
import os, sys
sys.path.append(os.path.expanduser('~/camera-network'))

import threading, time
import rospy
import std_srvs.srv
from camera_network_msgs.srv import *
from scripts.Util.command import *
from scripts.Util.constant import *
from scripts.Util.miscellaneous import *


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
        rospy.loginfo("Driver Launched")

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

        rospy.Service(
            'download_data',
            Uint32,
            self.download_data_cb)

        # TODO Make current_status service
        self.current_status = 'Idle'
        self.shell_config = ''
        self.lock_gphoto = False
        self.cam_threads = {'SendData': threading.Thread(target=self._send_data_thread, name='SendData')}
        self.cam_dl = False

        rospy.loginfo('Taking Environment Data')
        try:
            self.path_src = CAMNET_OUTPUT_DIR
            if self.path_src is None or not os.path.isdir(self.path_src):
                rospy.logwarn('Bad source path for sending pictures: %s' % self.path_src)
                self.path_src = HOME_DIR + '/camera-output/'
                try:
                    os.mkdir(self.path_src)
                except:
                    rospy.logwarn('Default path is already present: %s' % self.path_src)
        except :
            rospy.logerr('Taking Env Data Error')

        self.path_dst = CAMNET_SERVER_DATA_DIR
        rospy.loginfo('Destination path taken: %s' % self.path_dst)

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

    def takeVideo(self, time_period):
        self.capture_video_service(time_period)
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
        if not self.lock_gphoto:
            rospy.loginfo('Saving new settings shell: {}'.format(req.option))
            self.shell_config = req.option
        else:
            rospy.logwarn('Camera locked. You cant save your shell settings')
        return {}

    def download_data_cb(self, req):
        """
        CallBack Ros Service: Launch thread for downloading pictures from camera to server.
        Check if current thread is starting or not.
        :param req: uInt32.integer = int
        :return: empty dict
        """
        self._start_dl_all_data_thread(req)
        return {} 

    def _start_dl_all_data_thread(self, req):

        rospy.logwarn('CREATING THREAD')
        if 'DLdata' not in self.cam_threads:
            rospy.logwarn('CREATING THREAD')
            self.cam_threads['DLdata'] = threading.Thread(target=self._dl_data_thread,
                                                            name='DLdata',
                                                            args=(req,))
            self.cam_threads['DLdata'].start()
        if not self.cam_threads['DLdata'].is_alive:
            rospy.logwarn('STARTING THREAD')
            self.cam_threads['DLdata'].start()

    def _dl_data_thread(self, req):
        #TODO : progression bar
        rospy.logwarn('ENTERING THREAD')
        while True:
            number_pictures = self._number_pictures_to_download(DL_DATA_SERIE_SIZE)
            folder_list_files = [file for file in os.listdir(self.path_src) if '.CR2' in file]
            if number_pictures > 0 and len(folder_list_files) + number_pictures < MAX_PHOTOS_ON_DISK: 
                self._load_and_delete_data(number_pictures)
                self.send_data(self.path_src, self.path_dst, folder_list_files)
            else:
                break

    def progressive_dl_cb(self, req, max_pictures=DL_DATA_SERIE_SIZE):
        self._start_send_data_thread(req)
        number_pictures = self._number_pictures_to_download(max_pictures)
        folder_list_files = [file for file in os.listdir(self.path_src)]
        if number_pictures > 0 and len(folder_list_files) + number_pictures < MAX_PHOTOS_ON_DISK:
           self._load_and_delete_data(number_pictures)

    def _start_send_data_thread(self, req):
        if not self.cam_threads['SendData'].is_alive():
            self.cam_threads['SendData'] = threading.Thread(target=self._send_data_thread,
                                                            name='sendData',
                                                            args=(req,))
            self.cam_threads['SendData'].start()

    def _number_pictures_to_download(self, max_pictures=DL_DATA_SERIE_SIZE):
        with Locker(LOCK_CAMNET_CAPTURE):
            output = Command.run('gphoto2 --list-files')
        if not output:
            return 0
        camera_raw_list_pictures = output.splitlines()
        camera_list_pictures = [line.split()[1] for line in camera_raw_list_pictures if '#' in line]
        rospy.logwarn(' '.join(camera_list_pictures))
        if max_pictures < 0:
            return len(camera_list_pictures)
        else:
            return min(len(camera_list_pictures), max_pictures)


    def _load_and_delete_data(self, number):
        try:
             with Locker(LOCK_CAMNET_CAPTURE):
                os.chdir(self.path_src)
                self._download_camera_data(number)
                rospy.sleep(2)
                self._delete_camera_data(number)
        except:
            err_type, err_tb, e = sys.exc_info()
            rospy.logerr(err_tb)
    
    def _download_camera_data(self, number_pictures):
        Command.run('gphoto2 --get-file 1-{} --wait-event-and-download=6s --skip-existing'.format(number_pictures), 'get files in camera')

    def _delete_camera_data(self, number_pictures):
        Command.run('gphoto2 --delete-file 1-{} --recurse --wait-event=1s'.format(number_pictures))

    def _send_data_thread(self, req):
        while True:

            folder_list_files = [file for file in os.listdir(self.path_src) if '.CR2' in file]
            theta_list_files = [file for file in os.listdir(THETA_OUTPUT_DIR) if '.JPG' in file]
            if len(theta_list_files) > 0:
                self.send_data(THETA_OUTPUT_DIR, THETA_SERVER_DATA_DIR, theta_list_files)
            if len(folder_list_files) > 0:
                self.send_data(self.path_src, self.path_dst, folder_list_files)
                rospy.sleep(2)
            elif len(theta_list_files) == 0:
                rospy.sleep(10)


    def send_data(self, srcfolder, dstfolder, files_list):

        if len(files_list) > 0:
            try:
                # TODO Sending with rsync: Find way to show progress / checkpoint with command
                cmd = 'rsync -v --remove-source-files --no-owner --no-group -p --chmod=ugo=rwX,Dugo=rwX'
                time_out = '--timeout=10'
                files = ' '.join(srcfolder + '/' + file for file in files_list)
                # try 5 times
                for i in range(5):
                    try:
                        Command.run(' '.join([cmd, time_out, files, os.path.join(dstfolder, get_today_date()) + '/']), 'SendRawData - rsync')
                        break
                    except AssertionError as e:
                        rospy.logwarn('Rsync Error: %s' % e)
                        rospy.sleep(10)
            except:
                err_type, err_tb, e = sys.exc_info()
                rospy.logerr(err_tb)


