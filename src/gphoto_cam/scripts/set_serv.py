#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:57:07 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *
import gphoto2_cli_wrapper as gphoto


def set_camera(req):
    print "Taking Shot"
    if(req.iso != ""):
        gphoto.run(" --set-config /main/imgsettings/iso="+req.iso)
    if(req.imageformat != ""):
        gphoto.run(" --set-config /main/imgsettings/imageformat="+req.imageformat)
    if(req.aperture != ""):
        gphoto.run(" --set-config /main/capturesettings/aperture="+req.aperture)
    if(req.shutterspeed != ""):
        gphoto.run(" --set-config /main/capturesettings/shutterspeed="+req.shutterspeed)


def set_camera_server():
    rospy.init_node('set_camera')
    s = rospy.Service('set_camera', CameraData, set_camera)
    print "Ready to get Configurations."
    rospy.spin()

if __name__ == "__main__":
    set_camera_server()