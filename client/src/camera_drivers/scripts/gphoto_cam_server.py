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
import std_srvs.srv
from camera_network_msgs.srv import *
from CameraParameterHandler import *
import tarfile
import os
import time
import camera_driver as cd

import gphoto2_cli_caller as gphoto


join = os.path.join


class GPhotoServer(cd.camera_driver):
    def __init__(self):
        self.camParam = CameraParameterHandler()
        self.camParam.set_camera_parameters()
        
        super(GPhotoServer, self).__init__()

        rospy.loginfo("Camera Ready")
        rospy.spin()

    def capture_image_cb(self, req):
        rospy.loginfo("Taking Picture")
        rospy.sleep(req.timer)
        # TODO: Faster than 1s sleep...
        msg = gphoto.run(" --capture-image --wait-event=1s --keep")
        return msg

    def capture_video_cb(self, req):
        rospy.logwarn("Gphoto2 does not support video capture at this moment!")
        return []

    def load_camera_cb(self, req):
        rospy.sleep(3)
        try:
            rootPath = os.environ["CAMNET_OUTPUT_DIR"]
        except KeyError:
            rootPath = os.path.expanduser("~/Pictures")
        rospy.loginfo("Loading picture.")

        msg = gphoto.run("--get-all-files")

        # TODO: If previous call fails, don't delete everything!

        rospy.loginfo("Deleting camera's pictures")
        gphoto.run(" -D --recurse")
        #self._make_tarfile(time.strftime('/home/CameraNetwork/%B.tar.gz'), rootPath)
        # self._delete_directory(rootPath)
        return msg

    def _delete_directory(self, directory):
        try:
            for f in os.listdir(directory):
                os.remove(directory + f)
        except:
            rospy.logerr("Problem while deleting " + directory)

    def _make_tarfile(self, output_filename, source_dir):
        with tarfile.open(output_filename, "w:gz") as tar:
            tar.add(source_dir, arcname=os.path.basename(source_dir))

    def set_camera_cb(self, req):
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

        backMessage = gphoto.run(commandCall)

        return backMessage

    def get_camera_cb(self, req):
        rospy.loginfo("Getting camera's Configuration")

        iso = gphoto.run(" --get-config " + self.camParam.isoConfig)
        imageformat = gphoto.run(" --get-config " + self.camParam.imageformatConfig)
        aperture = gphoto.run(" --get-config " + self.camParam.apertureConfig)
        shutterspeed = gphoto.run(" --get-config " + self.camParam.shutterspeedConfig)

        if not req.getAllInformation:
            iso = self._parse_current_value(iso)
            imageformat = self._parse_current_value(imageformat)
            aperture = self._parse_current_value(aperture)
            shutterspeed = self._parse_current_value(shutterspeed)

        return {
            'iso': iso,
            'imageformat': imageformat,
            'aperture': aperture,
            'shutterspeed': shutterspeed,
        }

    def calibrate_video_cb(self, req):
        rospy.logwarn("Not supported with gphoto driver!")

    def _parse_current_value(self, string):

        lineList = string.split('\n')
        for n in lineList:
            if n.find('Current') == 0:
                return n[8:]  # remove Current: from the string
        return ''


if __name__ == "__main__":
    rospy.init_node('gphoto_cam')
    server = GPhotoServer()
