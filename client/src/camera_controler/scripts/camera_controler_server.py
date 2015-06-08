#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 16:19:15 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

This class init the camera's controler node. It implement directly the services
and instantiate timelapse action.
"""

import roslib
roslib.load_manifest("rosparam")
roslib.load_manifest("camera_controler")
import rospy
import rospkg
import rosparam
import timelaps_action as ta
import camera_handler as ch
import os
import std_msgs.msg
import std_srvs.srv
from camera_network_msgs.srv import *
from camera_network_msgs.msg import *
import subprocess


class Server:
    def __init__(self):
        rospy.init_node('timelaps_server')

        self.cam_handler = ch.CameraHandler()
        self.TimelapsAction = ta.TimelapsAction(self.cam_handler)
        rospy.Subscriber(
            '/network_capture_chatter',
            Capture,
            self.capture_listen_cb,
            queue_size=1)
        rospy.Subscriber(
            '/network_capture_video_chatter',
            std_msgs.msg.UInt32,
            self.capture_video_listen_cb,
            queue_size=1)

        # setup server to set camera init parameters
        rospy.Service(
            'save_config',
            std_srvs.srv.Empty(),
            self.save_settings_cb)
        rospy.Service(
            'preview_camera',
            std_srvs.srv.Empty(),
            self.preview_image_cb)
        rospy.Service(
            'shutdown_device',
            CommandOption,
            self.shutdown_device_cb)
        rospy.Service(
            'calibrate_device',
            std_srvs.srv.Empty(),
            self.calibrate_device_cb)
        rospy.Service(
            'update_camera',
            InCameraData,
            self.update_camera_cb)
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.cam_handler
        del self.TimelapsAction
        try:
            rospy.delete_param('camera_setting')
        except KeyError:
            pass
        try:
            rospy.delete_param('file')
        except KeyError:
            pass
        try:
            rospy.delete_param('/IP/' + os.environ['CAMERA_NAME'])
        except KeyError:
            pass

    def save_settings_cb(self, req):
        rospack = rospkg.RosPack()
        path = rospack.get_path('camera_controler')
        yamlFile = rospy.get_param('file')
        # this function overwrite yamlfile and dump camera_setting current
        # values
        rosparam.yamlmain(
            ['rosparam', 'dump', path + '/param/' + yamlFile, 'camera_setting'])
        rospy.loginfo(
            "Saved camera_setting parameters to : " +
            path +
            '/param/' +
            yamlFile)
        return {}

    def preview_image_cb(self, req):
        self.cam_handler.takePreview()
        return []

    def update_camera_cb(self, req):
        settings = {
            'iso': req.iso,
            'imageformat': req.imageformat,
            'shutterspeed': req.shutterspeed,
            'aperture': req.aperture}
        self.cam_handler.updateCameraSetting(settings)
        return ''

    def capture_listen_cb(self, req):
        # Simply print out values in our custom message.
        if req.isHdr:
            rospy.loginfo("Taking hdr picture")
            self.cam_handler.takeHDRPicture(0)
        elif req.isAEB:
            rospy.loginfo("Taking AEB picture")
            self.cam_handler.takeAEBPicutre()
        else:
            rospy.loginfo("Taking single picture")
            self.cam_handler.takeSinglePicture(0, setCamera=False)

    def capture_video_listen_cb(self, req):
        self.cam_handler.takeVideo(req.data)

    def calibrate_device_cb(self, req):
        """Calibration is the Auto feature."""
        self.cam_handler.calibrate()
        return []

    def shutdown_device_cb(self, req):
        if req.option in ['', '-h', '-r']:
            command = "/usr/bin/sudo /sbin/shutdown " + req.option + " now"
            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
            process.communicate()[0]
            rospy.loginfo(
                "Shutdown command launched with option : " +
                req.option)
        else:
            rospy.logwarn("Option Invalid")
        return []

    def _delete_file(self, filename, directory=""):
        try:
            os.remove(directory + filename)
        except:
            rospy.logerr("Problem while deleting " + directory + filename)

    def _rename_file(self, filename, directory, newName):
        try:
            os.rename(directory + filename, directory + newName + ".jpeg")
        except:
            rospy.logerr("Problem while renaming " + directory + filename)


if __name__ == "__main__":
    serverInstance = Server()
