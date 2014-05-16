#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 11:24:16 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *
import gphoto2_cli_wrapper as gphoto

def find_current_value(string):
    
    splitedString = string.split('\n')
    for n in splitedString:
        if n.find('Current') == 0:
            return n[8:]
    return ''


def get_camera_cb(req):
    print "getting Configuration"
    
    isoConfig = rospy.get_param("/isoConfig"," ")
    iso = gphoto.run(" --get-config " + isoConfig)
    iso = find_current_value(iso)
    
    imageformatConfig = rospy.get_param("/imageformatConfig"," ")
    imageformat = gphoto.run(" --get-config " + imageformatConfig)
    imageformat = find_current_value(imageformat)
    
    apertureConfig = rospy.get_param("/apertureConfig"," ")
    aperture = gphoto.run(" --get-config "+ apertureConfig)
    aperture = find_current_value(aperture)
    
    shutterspeedConfig = rospy.get_param("/shutterspeedConfig"," ")
    shutterspeed = gphoto.run(" --get-config " + shutterspeedConfig)
    shutterspeed = find_current_value(shutterspeed)
            
    return {'iso': iso,'imageformat':imageformat,'aperture':aperture,'shutterspeed':shutterspeed}

def get_camera_server():
    rospy.init_node('get_camera')
    s = rospy.Service('get_camera', OutCameraData, get_camera_cb)
    print "Ready to send Configurations."
    rospy.spin()

if __name__ == "__main__":
    get_camera_server()