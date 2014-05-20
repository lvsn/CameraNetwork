#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 16:13:10 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *
import gphoto2_cli_wrapper as gphoto

    
class CameraParameterHandler:
    def __init__(self):
        self.configLoaded = False
        self.cameraModel = ''        
        
        print "...Looking for camera..."
        cameralist = gphoto.run(" --auto-detect")
        self.cameraModel = self._find_camera_type(cameralist)
        if self.cameraModel == '':
            print "No Camera Found"
        else:
            print "Found : " + self.cameraModel
        rospy.set_param("camera_model", self.cameraModel)
        
    def __del__(self):
        rospy.delete_param("camera_model") #local parameter
        if self.configLoaded:
            try:
                rospy.delete_param("~isoConfig")
                rospy.delete_param("~imageformatConfig")
                rospy.delete_param("~apertureConfig")
                rospy.delete_param("~shutterspeedConfig")
            except KeyError:
                print "Unable to delete parameter (not set)"
        
    def set_camera_parameters(self):
        if self.cameraModel == 'Nikon DSC D3100 (PTP mode)':
            rospy.set_param("~isoConfig","/main/imgsettings/iso")
            rospy.set_param("~imageformatConfig","/main/capturesettings/imagequality")
            rospy.set_param("~apertureConfig","/main/capturesettings/f-number")
            rospy.set_param("~shutterspeedConfig","/main/capturesettings/shutterspeed")
            self.configLoaded = True            
        
    
    def _find_camera_type(self,string):
    
        splitedString = string.split('\n')
        if len(splitedString) > 3:
            return splitedString[2].split('  ')[0]  #the name and port are separated with at least two spaces
        else:
            return ''
        

if __name__ == "__main__":
    get_camera_data()