#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Abstract class of camera_driver
"""
from abc import ABCMeta, abstractmethod

import os
from datetime import datetime
import rospy
import std_srvs.srv
from camera_network_msgs.srv import *
import sensor_msgs.msg
import envoy
from cv_bridge import CvBridge, CvBridgeError


class camera_driver(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        self.cameraModel = None
        try:
            self.homePath = os.environ["CAMNET_OUTPUT_DIR"]
        except KeyError:
            self.homePath = os.path.expanduser("~/Pictures")
        try:
            self.serverPath = os.environ["CAMNET_SERVER_DATA_DIR"]
        except KeyError:
            user = rospy.get_param("/Username/server")
            self.serverPath = os.path.expanduser("/home/" + user + "/Pictures/server")
        #should look on param server for serverusername?
        
        rospy.set_param("DownloadQty", 10)
        self.parameterQueue = []
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher("/preview", sensor_msgs.msg.Image, queue_size=1)
        rospy.Service('capture_camera', CaptureService, self.capture_image_cb)
        rospy.Service('preview_camera_driver', std_srvs.srv.Empty, self.preview_capture_cb)
        rospy.Service('get_camera', OutCameraData, self.get_camera_cb)
        rospy.Service('set_camera', InCameraData, self.set_camera_cb)
        rospy.Service('load_camera', Load, self.load_camera_cb)
        rospy.Service('capture_video', Uint32, self.capture_video_cb)
        rospy.Service('calibrate_picture', std_srvs.srv.Empty, self.calibrate_picture_cb)
        rospy.Service('configure_parameter_queue', ParameterQueue, self.set_parameter_queue_cb)
        rospy.loginfo("Camera Service Ready")

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
    def preview_capture_cb(self, req):
        pass

    @abstractmethod
    def capture_video_cb(self, req):
        pass

    def load_camera_cb(self, req):
        '''
        Template method :
        1- create standard directory path
        2- transfert picture from device(ex camera) to directory
        3- delete picture from device
        '''
        if (self._create_picture_standard_directory(req.path) == 1):
            msg = self._copy_picture_from_device_to_standard_directory(req.path)
            if not "error" in msg:
                rospy.loginfo("Deleting Pictures from camera")
                self._delete_picture_from_device()
        else:
            msg = 'Load aborted'
            rospy.logerr(msg)
        return msg

    def upload_data_to_server(self):
        """
        This method use rsync process to upload all the data from client homepath
         to user@address (ros server)'s serverPath.

        """
        user = rospy.get_param("/Username/server")
        address = rospy.get_param("/IP/server")
        rsyncCommand = "-azP " + self.homePath + "/ " + user + "@" + address + ":" + self.serverPath
        rospy.loginfo("launch rsync " + rsyncCommand)
        self._run_rsync(rsyncCommand)

    def _create_picture_standard_directory(self, directory):
        """
        Create the directory from homePath. The method make sure .. is not used
        :param directory:
        :return:
        """
        loadPath = os.path.join(
            self.homePath,
            self._filename_format(directory),
        )
        if loadPath.find('..') != -1:
            rospy.logwarn("Use of '..' is prohibited")
            return -1
        directory = os.path.dirname(loadPath)
        self._mkdir(directory)
        return 1

    @abstractmethod
    def _copy_picture_from_device_to_standard_directory(self, filename):
        pass

    @abstractmethod
    def _delete_picture_from_device(self):
        pass

    @abstractmethod
    def set_camera_cb(self, req):
        pass

    @abstractmethod
    def get_camera_cb(self, req):
        pass

    def _add_Choice(self, choicesString, newChoice):
        return choicesString + '\nChoice : ' + str(newChoice)

    @abstractmethod
    def calibrate_picture_cb(self, req):
        pass

    def set_parameter_queue_cb(self, req):
        """
        add a dictionnary of parameter to parameterQueue.
        If flush is set to True, it reset the parameterQueue
        :param req:
        """
        parameterSet = {}
        parameterSet['iso'] = req.iso
        parameterSet['shutterspeed'] = req.shutterspeed
        parameterSet['aperture'] = req.aperture
        parameterSet['imageformat'] = req.imageformat

        self.parameterQueue.append(parameterSet)
        if(req.flush):
            self.parameterQueue = []
        rospy.loginfo("ParameterQueue is now :" + str(self.parameterQueue))
        return []

    def _set_camera_model(self, camera):
        self.cameraModel = camera
        rospy.set_param("camera_model", self.cameraModel)

    def _mkdir(self, dirPath):
        if not os.path.exists(dirPath):
            os.makedirs(dirPath)

    def _filename_format(self, string_, pictureId=0, pictureFormat='jpeg'):
        """
        Produces the name of the image. string_ comes from
        camera_controller's camera_handler.py:_generatePictureName

        Will format in standard strftime
        %n is any __str__ object for unique ID
        %C is picture format
        """
        string_ = string_.replace('%C', pictureFormat)
        string_ = string_.replace('%n', str(pictureId))
        return datetime.now().strftime(string_)

    def _publish_picture(self, img):
        """
        This function publish opencv image to /preview
        :param img:
        """
        try:
            self.image_publisher.publish(
                self.bridge.cv2_to_imgmsg(
                    img,
                    "bgr8"))
        except CvBridgeError as e:
            rospy.logwarn("imagePublish error : " + e)
        rospy.loginfo("Image Published to preview")

    def _run_rsync(self, cmd):
        """
        Launch a rsync process
        :param cmd: command string ex: '--remove-source-files -azP /home/user/test/ server@192.0.0.2:/home/user/test'
        :return envoy:  return envoy object with stdout stderr etc
        """
        cmd = 'rsync ' + cmd
        r = envoy.run(cmd)
        if r.status_code:
            rospy.logerr("rsync process error : " + r.std_err)
        return r
