#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  9 14:41:12 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Server that handle action/services from user.
"""
from __future__ import print_function

import os
import errno

import roslib; roslib.load_manifest('camera_master_server')
import rospy

import file_transfer_action as fta
import network_capture_action as nca
import getpass
import time
import thread

class Server:
    def __init__(self):
        self.terminateThreads = False
        rospy.init_node('master_server')
        self.setPictureDirectory()
        self.sftp = fta.sftp_action(self.imagePath)
        self.network_capture = nca.network_capture_action()
        self.setParam()
        thread.start_new_thread( Server._heart_beat, (self, ))
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def setParam(self):
        rospy.set_param('/IP/server', os.getenv("ROS_IP"))
        rospy.set_param('/Username/server', getpass.getuser())

    def setPictureDirectory(self):
        """
        Sets the self.imagePath variable according to CAMNET_OUTPUT_DIR
        environment variable. If not found, reverts to ~/Pictures.
        If the directory does not exists, it tries to create it.
        """
        try:
            self.imagePath = os.environ["CAMNET_OUTPUT_DIR"]
        except KeyError:
            self.imagePath = os.path.expanduser("~/Pictures")
            rospy.logwarn(
                "Could not find the CAMNET_OUTPUT_DIR environment variable.\n"
                "Reverting to ~/Pictures as the picture directory."
            )

        try:
            os.makedirs(self.imagePath)
        except OSError as e:
            if e.errno != errno.EEXIST:
                rospy.logfatal("Install file not executed! : "
                               "CameraNetwork path not set")

    def _heart_beat(self):
        while(not self.terminateThreads):
            connectedDevices = rospy.get_param("/IP")
            for name in connectedDevices:
                response = os.system("ping -c 1 " + connectedDevices[name] + " > /dev/null 2>&1")
                if response == 1:
                    rospy.logwarn("Lost connection with " + name)
                    rospy.delete_param("/IP/" + name)
            rospy.sleep(4)

    def shutdown(self):
        del self.sftp
        del self.network_capture


if __name__ == "__main__":
    serverInstance = Server()
