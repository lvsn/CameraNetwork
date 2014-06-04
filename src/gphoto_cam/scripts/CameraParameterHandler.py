#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 16:13:10 2014

@author: mathieugaron
"""

import rospy
import gphoto2_cli_caller as gphoto

    
class CameraParameterHandler:
    def __init__(self):
        self.configLoadedtoRos = False
        self.cameraModel = ''        
        
        rospy.loginfo("...Looking for camera...")
        
        r = rospy.Rate(0.25) #retry connection every 4 seconds
        
        while self.cameraModel == '':
            cameralist = gphoto.run(" --auto-detect")
            self.cameraModel = self._parse_gphoto_camera_list(cameralist)
            if self.cameraModel == '':
                rospy.logwarn("No Camera Found")
            r.sleep()
        
        rospy.loginfo("Found : " + self.cameraModel)
        
    def __del__(self):
        if self.configLoadedtoRos:
            try:
                rospy.delete_param("camera_model") #local parameter
                rospy.delete_param("~isoConfig")
                rospy.delete_param("~imageformatConfig")
                rospy.delete_param("~apertureConfig")
                rospy.delete_param("~shutterspeedConfig")
            except KeyError:
                rospy.logerr("Unable to delete parameter (not set)")
        
    def set_camera_parameters(self):
        rospy.set_param("camera_model", self.cameraModel)
        rospy.set_param("~isoConfig","iso")
        rospy.set_param("~apertureConfig","aperture")
        rospy.set_param("~shutterspeedConfig","shutterspeed")
        
        #Load spécific parameter
        if self.cameraModel == 'Nikon DSC D3100 (PTP mode)':
            rospy.set_param("~imageformatConfig","/main/capturesettings/imagequality")
            rospy.set_param("~apertureConfig","/main/other/5007")
            
        elif self.cameraModel == 'Canon Digital Rebel XT (normal mode)':
            rospy.set_param("~imageformatConfig","/main/settings/imageformat")
            
            
        self.configLoadedtoRos = True
        rospy.loginfo("Camera's Configuration loaded")
        
    
    def _parse_gphoto_camera_list(self,string):
    
        lineList = string.split('\n')
        #first two lines are non important information
        if len(lineList) > 3:
            # The string contain the usb number wich is not important, and we remove all spaces at the end of
            # the camera's name.
            return lineList[2].split(' usb')[0].rstrip() 
        else:
            return ''
        

if __name__ == "__main__":
    get_camera_data()