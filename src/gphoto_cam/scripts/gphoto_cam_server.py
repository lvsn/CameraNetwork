#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('gphoto_cam')
import rospy
from camera_network_msgs.srv import *
from CameraParameterHandler import *

import gphoto2_cli_caller as gphoto
import os    
    
    
class gphoto_server():
    
    def __init__(self):
        #init gphoto cam
        self.camParam = CameraParameterHandler()
        self.camParam.set_camera_parameters()
        #Start services
        rospy.Service('capture_camera', CaptureService, self.capture_image_cb)
        rospy.Service('get_camera', OutCameraData, self.get_camera_cb)
        rospy.Service('set_camera', InCameraData, self.set_camera_cb)
        rospy.Service('load_camera',Load,self.load_camera_cb)
        
        rospy.loginfo("Camera Ready")
        rospy.spin()

    def capture_image_cb(self,req):
        rospy.loginfo("Taking Picture")
        rospy.sleep(req.timer)
        msg = gphoto.run(" --capture-image --wait-event=1s")
        return msg
    
    def load_camera_cb(self,req):
        rospy.sleep(3)
        rootPath = '/home/CameraNetwork/'
        filename = " --filename " + rootPath + req.path
        if filename.find('..') != -1:
            rospy.logwarn("use of .. is prohibed")
            return "error"
        rospy.loginfo("Loading picture to : " + filename)

        msg = gphoto.run(filename + " -P")
        
        rospy.loginfo("Deleting camera's pictures")
        gphoto.run(" -D --recurse")
        return msg
        
    
    def set_camera_cb(self,req):
        rospy.loginfo("Setting camera's Configuration : " + str(req))
        backMessage = ''
        commandCall = ''
        if(req.iso != ""):
            commandCall += " --set-config " + self.camParam.isoConfig + "=" + req.iso
                
        if(req.imageformat != ""):
            commandCall += " --set-config " + self.camParam.imageformatConfig + "=" + req.imageformat
                
        if(req.aperture != ""):
            commandCall += " --set-config " + self.camParam.apertureConfig + "=" + req.aperture
                
        if(req.shutterspeed != ""):
            commandCall += " --set-config " + self.camParam.shutterspeedConfig + "=" + req.shutterspeed
            
        backMessage = gphoto.run(commandCall)
                
        return backMessage
    
    
    def get_camera_cb(self,req):
        rospy.loginfo("Getting camera's Configuration")
        
        iso = gphoto.run(" --get-config " + self.camParam.isoConfig) 
        imageformat = gphoto.run(" --get-config " + self.camParam.imageformatConfig)     
        aperture = gphoto.run(" --get-config "+ self.camParam.apertureConfig)        
        shutterspeed = gphoto.run(" --get-config " + self.camParam.shutterspeedConfig)
        
        if not req.getAllInformation:
            iso = self._parse_current_value(iso)
            imageformat = self._parse_current_value(imageformat)
            aperture = self._parse_current_value(aperture)
            shutterspeed = self._parse_current_value(shutterspeed)
                
        return {'iso': iso,'imageformat':imageformat,'aperture':aperture,'shutterspeed':shutterspeed}
    
    
    def _parse_current_value(self,string):
        
        lineList = string.split('\n')
        for n in lineList:
            if n.find('Current') == 0:
                return n[8:]    #remove Current: from the string
        return ''


if __name__ == "__main__":
    rospy.init_node('gphoto_cam')
    #init gphoto cam
    server = gphoto_server()


