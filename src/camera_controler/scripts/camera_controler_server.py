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

if __name__ == "__main__":
    rospy.init_node('timelaps_server')
    cam_handler = ch.CameraHandler()
    server = ts.TimelapsServer(cam_handler)
    listener = ncl.network_capture_listener(cam_handler)
    listener.listen()
    rospy.spin()
