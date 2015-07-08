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

DL_DATA_SERIE_SIZE = 10


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
            Uint32,
            self.download_data_cb)

        self.current_status = 'Idle'
        self.shell_config = ''
        self.lock_gphoto = False
        self.cam_threads = {'LoadData': threading.Thread(target=self.download_data_sequence, name='LoadData')}

        rospy.loginfo('Taking Environment Data')
        try:
            self.path_src = os.environ.get('CAMNET_OUTPUT_DIR')
            if self.path_src is None or not os.path.isdir(self.path_src):
                rospy.logwarn('Bad source path for sending pictures: %s' % self.path_src)
                self.path_src = os.environ['HOME'] + '/camera-output/'
                try:
                    os.mkdir(self.path_src)
                except:
                    rospy.logwarn('Default path is already present: %s' % self.path_src)
        except :
            rospy.logerr('Taking Env Data Error')

        self.path_dst = os.environ.get('CAMNET_SERVER_DATA_DIR')
        if self.path_dst is None:
            self.path_dst = 'JUBEC7@victoria.gel.ulaval.ca:/home-local/yahog.extra.nobkp/www/pictures/test_pro/'
            rospy.logwarn('Bad destination path for sending pictures: %s' % self.path_dst)
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
        if not self.lock_gphoto:
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
        """
        CallBack Ros Service: Launch thread for downloading pictures from camera to server.
        Check if current thread is starting or not.
        :param req: uInt32.integer = int
        :return: empty dict
        """
        if not self.cam_threads['LoadData'].is_alive():
            self.cam_threads['LoadData'] = threading.Thread(target=self.download_data_sequence,
                                                            name='LoadData',
                                                            args=(req.integer,))
            self.cam_threads['LoadData'].start()
        else:
            rospy.logwarn('*** Loading process is currently activate ***')
        return {}

    def download_data_sequence(self, count):
        """
        Routine: == Download data sequence ==
        1 - Adjust count with pictures list
        2 - Load from camera for each group of DL_DATA_SERIE_SIZE pictures
        3 - Send with rsync to destination server for each group of DL_DATA_SERIE_SIZE pictures
        4 - Repeat it until count is over
        :param count: int - number of pictures for downloading
        """
        try:
            list_pictures = (self._run_cmd('gphoto2 --list-files')).splitlines()
            count_pictures = len([line.split()[1] for line in list_pictures if '#' in line])

            if count > count_pictures:
                count = count_pictures
            while count >= 0:
                rospy.loginfo('ServiceDownloadData: %i pictures left' % count)
                rospy.loginfo('-> Loading data from camera: %s pictures' % (DL_DATA_SERIE_SIZE if count >= DL_DATA_SERIE_SIZE else count))
                self.load_data(DL_DATA_SERIE_SIZE if count >= DL_DATA_SERIE_SIZE else count)
                rospy.loginfo('-> Sending data to destination: %s' % self.path_dst)
                self.send_data()
                rospy.loginfo('ServiceDownloadData: sequence done')
                count -= DL_DATA_SERIE_SIZE
        except:
            err_type, err_tb, e = sys.exc_info()
            rospy.logerr(err_tb)

    def load_data(self, number=-1):
        """
        Routine: Loads raw data from camera to local pictures folder
        1 - getting camera list files
        2 - downloading number raw data
        3 - delete it if downloaded
        """
        try:
            # 1 - Getting camera list files
            # TODO Check execution time with threading.lock something like that
            camera_list = self._run_cmd('gphoto2 --list-files').splitlines()
            camera_list_files = [line.split()[1] for line in camera_list if '#' in line]
            i = 0
            if number < 1 or number > len(camera_list_files):
                number = len(camera_list_files)

            # 2 - Download number raw data
            while i < number:
                if len(camera_list_files) == 0:
                    rospy.loginfo('Camera empty')
                    break

                os.chdir(self.path_src)
                folder_list_files = [file for file in os.listdir(self.path_src)]

                if not camera_list_files[0] in folder_list_files:
                    self._run_cmd('gphoto2 --get-file=1', 'get first file in camera')
                    rospy.loginfo('#({}/{}) '.format(i + 1, number) + camera_list_files[0] + ' downloaded to folder')

                folder_list_files = [file for file in os.listdir(self.path_src)]

                # 3 - Delete image
                if camera_list_files[0] in folder_list_files:
                    self._run_cmd('gphoto2 --delete-file=1 --recurse')
                    camera_list_files.pop(0)
                    i += 1
        except:
            err_type, err_tb, e = sys.exc_info()
            rospy.logerr(err_tb)

    def organize_raw_data(self):
        # TODO Make organizer pictures
        """
        Routine: Organises raw data in local pictures folder
        """
        pass

    def send_data(self):
        # TODO Make sender pictures with rsync
        """
        Routine: Sends raw data from local pictures folder to another folder using rsync
        1 - Getting path source
        2 - Getting path destination
        3 - Sending with rsync
        """
        try:
            # 1 - Getting path pictures folder
            path_src = self.path_src

            # 2 - Getting path destination
            path_dst = self.path_dst

            # 3 - Sending with rsync
            # TODO Sending with rsync: Find way to show progress / checkpoint with command
            cmd = 'rsync -vrz --progress --remove-source-files --no-owner --no-group --chmod=ugo+rwx,Dugo+rwx'
            time_out = '--timeout=10'

            while True:
                try:
                    self._run_cmd(' '.join([cmd, time_out, path_src, path_dst]), 'SendRawData - rsync')
                    break
                except AssertionError as e:
                    rospy.logwarn('Rsync Error: %s' % e)
        except:
            err_type, err_tb, e = sys.exc_info()
            rospy.logerr(err_tb)

    def waiting_for_using(self, time_out=20):
        """
        Routine: Waits until camera is no longer used.
        """
        while time_out > 0:
            if self.lock_gphoto:
                rospy.sleep(1)
            else:
                break
            time_out -= 1

        if time_out <= 0:
            rospy.logwarn('Command TimeOut: Watchdog trigger enabled')
            raise AssertionError('Timeout')

    def _run_cmd(self, cmd, category='Unspecified CameraError'):
        """
        Run command and raise if error is detected.
        :param cmd: str - shell command
        :param category: str - choose your category
        :return: str - shell output
        """
        # Check
        if 'gphoto2' in cmd:
            try:
                self.waiting_for_using()
            except:
                raise AssertionError(category + ': ' + cmd)
            self.lock_gphoto = True

        cmd_output = envoy.run(cmd)

        if 'gphoto2' in cmd:
            self.lock_gphoto = False

        if cmd_output.status_code:
            rospy.logwarn(category + ':\n' + cmd)
            raise AssertionError(category + ': ' + cmd_output.std_err)
        else:
            return cmd_output.std_out