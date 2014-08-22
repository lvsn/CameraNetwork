#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  9 14:41:12 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Server that handle action/services from user.
"""

import roslib; roslib.load_manifest('camera_master_server')
import rospy
import file_transfer_action as fta
import network_capture_action as nca
import os

class server:
    
    def __init__(self):
<<<<<<< HEAD:src/camera_master_server/scripts/master_server.py
        self.imagePath = "/home/"+os.getlogin()+"/Images"
=======
        rospy.init_node('master_server')
        self.imagePath = "/home/"+os.getlogin()+"/Images"
>>>>>>> 91b6ea2bf0ca81f5d482124072d76c468dca4398:server/src/camera_master_server/scripts/master_server.py
        if not os.path.exists(self.imagePath):
            rospy.logfatal("Install file not executed! : CameraNetwork path not set")
        self.sftp = fta.sftp_action(self.imagePath)
        self.network_capture = nca.network_capture_action()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.sftp
        del self.network_capture

if __name__ == "__main__":
    
    serverInstance = server()
