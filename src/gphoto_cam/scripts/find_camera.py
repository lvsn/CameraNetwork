#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 16:13:10 2014

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


def get_camera_data():
    print "Ready to send Configurations."
    cameraData = gphoto.run(" --auto-detect")
    cameraData = find_current_value(cameraData)
    cameraData = 'test'
    rospy.set_param("/camera_model", cameraData)
    if cameraData == 'test':
        rospy.set_param("/isoConfig","/main/imgsettings/iso")
        rospy.set_param("/imageformatConfig","/main/imgsettings/imageformat")
        rospy.set_param("/apertureConfig","/main/capturesettings/aperture")
        rospy.set_param("/shutterspeed","/main/capturesettings/shutterspeed")

if __name__ == "__main__":
    get_camera_data()