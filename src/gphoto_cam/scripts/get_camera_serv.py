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

    iso = gphoto.run(" --get-config /main/imgsettings/iso")
    iso = find_current_value(iso)
    
    imageformat = gphoto.run(" --get-config /main/imgsettings/imageformat")
    imageformat = find_current_value(imageformat)
    
    aperture = gphoto.run(" --get-config /main/capturesettings/aperture")
    aperture = find_current_value(aperture)
    
    shutterspeed = gphoto.run(" --get-config /main/capturesettings/shutterspeed")
    shutterspeed = find_current_value(shutterspeed)
            
    return {'iso': iso,'imageformat':imageformat,'aperture':aperture,'shutterspeed':shutterspeed}

def get_camera_server():
    rospy.init_node('get_camera')
    s = rospy.Service('get_camera', OutCameraData, get_camera_cb)
    print "Ready to send Configurations."
    rospy.spin()

if __name__ == "__main__":
    get_camera_server()