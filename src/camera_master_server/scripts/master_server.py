#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  9 14:41:12 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_master_server')
import rospy
import file_transfer_server as fts
import network_capture_server as ncs
import os

class server:
    
    def __init__(self):
        self.homePath = os.path.expanduser("~")
        self.imagePath = self.homePath + '/CameraPicture/'
        if not os.path.exists(self.imagePath):
            os.makedirs(self.imagePath)
        rospy.init_node('master_server')
        self.sftp = fts.sftp_server(self.imagePath)
        self.network_capture = ncs.network_capture_server()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.sftp
        del self.network_capture

if __name__ == "__main__":
    
    serverInstance = server()
