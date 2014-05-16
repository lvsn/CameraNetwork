#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 16:13:10 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *
import gphoto2_cli_wrapper as gphoto

def find_camera_type(string):
    
    splitedString = string.split('\n')
    if len(splitedString) > 3:
        return splitedString[2].split('  ')[0]  #the name and port are separated with at least two spaces
    else:
        return ''



def get_camera_data():
    print "...Looking for Camera..."
    cameraData = gphoto.run(" --auto-detect")
    cameraData = find_camera_type(cameraData)
    if cameraData == '':
        print "No Camera Found"
    else:
        print "Found : " + cameraData
    rospy.set_param("/camera_model", cameraData)
    if cameraData == 'Nikon DSC D3100 (PTP mode)':
        rospy.set_param("/isoConfig","/main/imgsettings/iso")
        rospy.set_param("/imageformatConfig","/main/capturesettings/imagequality")
        rospy.set_param("/apertureConfig","/main/capturesettings/f-number")
        rospy.set_param("/shutterspeedConfig","/main/capturesettings/shutterspeed")
        
    #Put Other cameras here

if __name__ == "__main__":
    get_camera_data()