#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""
import roslib; roslib.load_manifest('gphoto_cam')
import rospy
from gphoto_cam.srv import *
from CameraParameterHandler import *

import gphoto2_cli_wrapper as gphoto

def find_current_value(string):
    
    splitedString = string.split('\n')
    for n in splitedString:
        if n.find('Current') == 0:
            return n[8:]
    return ''
    

def capture_image_cb(req):
    rospy.loginfo("Taking Picture")
    gphoto.run(" --capture-image")
    return 'OK'
    
def load_camera_cb(req):
    filename = " --filename " + req.path
    rospy.loginfo("Loading picture to folder" + req.path)
    gphoto.run(filename + " -P -D --recurse")
    return 'OK'    
    
def set_camera_cb(req):
    rospy.loginfo("Setting camera's Configuration")
    backMessage = ''
    commandCall = ''
    if(req.iso != ""):
        isoConfig = rospy.get_param("~isoConfig"," ")
        commandCall += " --set-config " + isoConfig + "=" + req.iso
            
    if(req.imageformat != ""):
        imageformatConfig = rospy.get_param("~imageformatConfig"," ")
        commandCall += " --set-config " + imageformatConfig + "=" + req.imageformat
            
    if(req.aperture != ""):
        apertureConfig = rospy.get_param("~apertureConfig"," ")
        commandCall += " --set-config " + apertureConfig + "=" + req.aperture
            
    if(req.shutterspeed != ""):
        shutterspeedConfig = rospy.get_param("~shutterspeedConfig"," ")
        commandCall += " --set-config " + shutterspeedConfig + "=" + req.shutterspeed
        
    backMessage = gphoto.run(commandCall)
            
    return backMessage
    
    
def get_camera_cb(req):
    rospy.loginfo("Getting camera's Configuration")
    
    isoConfig = rospy.get_param("~isoConfig"," ")
    iso = gphoto.run(" --get-config " + isoConfig)
    iso = find_current_value(iso)
    
    imageformatConfig = rospy.get_param("~imageformatConfig"," ")
    imageformat = gphoto.run(" --get-config " + imageformatConfig)
    imageformat = find_current_value(imageformat)
    
    apertureConfig = rospy.get_param("~apertureConfig"," ")
    aperture = gphoto.run(" --get-config "+ apertureConfig)
    aperture = find_current_value(aperture)
    
    shutterspeedConfig = rospy.get_param("~shutterspeedConfig"," ")
    shutterspeed = gphoto.run(" --get-config " + shutterspeedConfig)
    shutterspeed = find_current_value(shutterspeed)
            
    return {'iso': iso,'imageformat':imageformat,'aperture':aperture,'shutterspeed':shutterspeed}


if __name__ == "__main__":
    rospy.init_node('gphoto_cam')
    #log
    rospy.loginfo("gphoto_cam's URI : "+rospy.get_node_uri());
    #init gphoto cam
    camParam = CameraParameterHandler()
    camParam.set_camera_parameters()
    #Start services
    rospy.Service('capture_camera', Capture, capture_image_cb)
    rospy.Service('get_camera', OutCameraData, get_camera_cb)
    rospy.Service('set_camera', InCameraData, set_camera_cb)
    rospy.Service('load_camera',Load,load_camera_cb)
    
    rospy.loginfo("Camera Ready at " + str(rospy.get_rostime().secs))
    rospy.spin()


