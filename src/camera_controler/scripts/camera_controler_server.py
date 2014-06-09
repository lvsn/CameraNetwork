#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 16:19:15 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_controler')
import rospy
import timelaps_server as ts
import network_capture_listener as ncl
import camera_handler as ch
import parameter_save as ps

class server:
    
    def __init__(self):
        rospy.init_node('timelaps_server')
        self.cam_handler = ch.CameraHandler()
        self.timelapsServer = ts.TimelapsServer(self.cam_handler)
        self.listener = ncl.network_capture_listener(self.cam_handler)
        self.listener.listen()
        self.paramSaver = ps.save_server()
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.cam_handler
        del self.listener
        del self.timelapsServer
        rospy.delete_param('camera_setting')
        rospy.delete_param('file')

if __name__ == "__main__":
    
    serverInstance = server()