#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:57:07 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *
import gphoto2_cli_wrapper as gphoto


def set_camera_cb(req):
    print " --set-config " + imageformatConfig + "=" + req.imageformat
    backMessage = ''
    if(req.iso != ""):
        isoConfig = rospy.get_param("/isoConfig")
        out = gphoto.run(" --set-config " + isoConfig + "=" + req.iso)
        if out != '':
            backMessage += 'iso Value is not supported\n'
            
    if(req.imageformat != ""):
        imageformatConfig = rospy.get_param("/imageformatConfig")
        out = gphoto.run(" --set-config " + imageformatConfig + "=" + req.imageformat)
        if out != '':
            backMessage += 'imageformat Value is not supported\n'
            
    if(req.aperture != ""):
        apertureConfig = rospy.get_param("/apertureConfig")
        out = gphoto.run(" --set-config " + apertureConfig + "=" + req.aperture)
        if out != '':
            backMessage += 'aperture Value is not supported\n'
            
    if(req.shutterspeed != ""):
        shutterspeedConfig = rospy.get_param("/shutterspeedConfig")
        out = gphoto.run(" --set-config " + shutterspeedConfig + "=" + req.shutterspeed)
        if out != '':
            backMessage += 'shutterspeed Value is not supported\n'
            
    return backMessage

def set_camera_server():
    rospy.init_node('set_camera')
    s = rospy.Service('set_camera', InCameraData, set_camera_cb)
    print "Ready to get Configurations."
    rospy.spin()

if __name__ == "__main__":
    set_camera_server()