#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Gphoto driver, implement all basic services to make cameracontroler able to communicate
with it.
"""

import roslib
roslib.load_manifest('camera_drivers')
import rospy
from camera_network_msgs.srv import *
import tarfile
import os
import camera_driver as cd
import subprocess


join = os.path.join
gphoto2Executable = 'gphoto2'


class GPhotoServer(cd.camera_driver):

    def __init__(self):
        # Camera's configuration name
        self.isoConfig = "iso"
        self.apertureConfig = "aperture"
        self.shutterspeedConfig = "shutterspeed"
        self.imageformatConfig = "imageformat"

        self._find_camera()
        self._update_camera_parameters()

        super(GPhotoServer, self).__init__()

        rospy.spin()

    def capture_image_cb(self, req):
        rospy.loginfo("Taking Picture")
        rospy.sleep(req.timer)
        msg = self._run_gphoto(" --capture-image --wait-event=1s")
        return msg

    def capture_video_cb(self, req):
        rospy.logwarn("Gphoto2 does not support video capture at this moment!")
        return []

    def load_camera_cb(self, req):
        rospy.sleep(3)
        filename = " --filename " + join(self.homePath, req.path)
        if filename.find('..') != -1:
            rospy.logwarn("use of .. is prohibed")
            return "error"
        rospy.loginfo("Loading picture to : " + filename)

        msg = self._run_gphoto(filename + " -P")

        rospy.loginfo("Deleting camera's pictures")
        self._run_gphoto(" -D --recurse")
        return msg

    def set_camera_cb(self, req):
        """
        set camera's information. Will set data only if it contain something
        """
        rospy.loginfo("Setting camera's Configuration : " + str(req))
        backMessage = ''
        commandCall = ''
        if(req.iso != ""):
            commandCall += " --set-config " + \
                self.camParam.isoConfig + "=" + req.iso

        if(req.imageformat != ""):
            commandCall += " --set-config " + \
                self.camParam.imageformatConfig + "=" + req.imageformat

        if(req.aperture != ""):
            commandCall += " --set-config " + \
                self.camParam.apertureConfig + "=" + req.aperture

        if(req.shutterspeed != ""):
            commandCall += " --set-config " + \
                self.camParam.shutterspeedConfig + "=" + req.shutterspeed

        backMessage = self._run_gphoto(commandCall)

        return backMessage

    def get_camera_cb(self, req):
        """
        get camera's information. req contain a flag named getAllInformation.
        If true, it will return the whole gphoto's string
        If false, it will return only the value from the string
        """
        rospy.loginfo("Getting camera's Configuration")

        iso = self._run_gphoto(" --get-config " + self.camParam.isoConfig)
        imageformat = self._run_gphoto(
            " --get-config " +
            self.camParam.imageformatConfig)
        aperture = self._run_gphoto(
            " --get-config " +
            self.camParam.apertureConfig)
        shutterspeed = self._run_gphoto(
            " --get-config " +
            self.camParam.shutterspeedConfig)

        if not req.getAllInformation:
            iso = self._parse_current_value(iso)
            imageformat = self._parse_current_value(imageformat)
            aperture = self._parse_current_value(aperture)
            shutterspeed = self._parse_current_value(shutterspeed)

        return {
            'iso': iso,
            'imageformat': imageformat,
            'aperture': aperture,
            'shutterspeed': shutterspeed}

    def calibrate_picture_cb(self, req):
        rospy.logwarn("Calibrate Picture is not supported with gphoto driver!")

    def _parse_current_value(self, string):
        lineList = string.split('\n')
        for n in lineList:
            if n.find('Current') == 0:
                return n[8:]  # remove Current: from the string
        return ''

    def _find_camera(self):
        rospy.loginfo("...Looking for camera...")
        camera = ''
        r = rospy.Rate(0.25)  # retry connection every 4 seconds
        while camera == '':
            cameralist = self._run_gphoto(" --auto-detect")
            camera = self._parse_gphoto_camera_list(cameralist)
            if camera == '':
                rospy.logwarn("No Camera Found")
            r.sleep()

        self._set_camera_model(camera)

    def _update_camera_parameters(self):
        # Load specific parameter
        if self.cameraModel == 'Nikon DSC D3100 (PTP mode)':
            self.imageformatConfig = "/main/capturesettings/imagequality"
            self.apertureConfig = "/main/other/5007"
        # TODO add more model here, or load xml  MathGaron

    def _parse_gphoto_camera_list(self, string):
        lineList = string.split('\n')
        # first two lines are non important information
        if len(lineList) > 3:
            # The string contain the usb number wich is not important, and we remove all spaces at the end of
            # the camera's name.
            return lineList[2].split(' usb')[0].rstrip()
        else:
            return ''

    def _run_gphoto(self, cmd):
        cmd = gphoto2Executable + cmd

        p = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,
                             )

        stdout, stderr = p.communicate()
        ret = p.returncode

        if ret == 1:
            if 'No camera found' in stderr:
                rospy.logerror('Error talking to the camera: ' + stderr)

            return stdout

        return stdout


if __name__ == "__main__":
    rospy.init_node('gphoto_driver')
    server = GPhotoServer()
