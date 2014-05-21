#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""

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
    if(req.keepOnCamera):
        gphoto.run(" --capture-image")
    else:
        filename = " --filename ~/CameraPicture/%B/%d%B%y_%Hh%Mm%Ss.%C"
        gphoto.run(filename + " --capture-image-and-download")
    return 'OK'
    
    
def set_camera_cb(req):
    rospy.loginfo("Setting camera's Configuration")
    backMessage = ''
    if(req.iso != ""):
        isoConfig = rospy.get_param("/isoConfig"," ")
        out = gphoto.run(" --set-config " + isoConfig + "=" + req.iso)
        if out != '':
            backMessage += 'iso Value is not supported\n'
            
    if(req.imageformat != ""):
        imageformatConfig = rospy.get_param("/imageformatConfig"," ")
        out = gphoto.run(" --set-config " + imageformatConfig + "=" + req.imageformat)
        if out != '':
            backMessage += 'imageformat Value is not supported\n'
            
    if(req.aperture != ""):
        apertureConfig = rospy.get_param("/apertureConfig"," ")
        out = gphoto.run(" --set-config " + apertureConfig + "=" + req.aperture)
        if out != '':
            backMessage += 'aperture Value is not supported\n'
            
    if(req.shutterspeed != ""):
        shutterspeedConfig = rospy.get_param("/shutterspeedConfig"," ")
        out = gphoto.run(" --set-config " + shutterspeedConfig + "=" + req.shutterspeed)
        if out != '':
            backMessage += 'shutterspeed Value is not supported\n'
            
    return backMessage
    
    
def get_camera_cb(req):
    rospy.loginfo("Getting camera's Configuration")
    
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


def gphoto_cam_server():
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
    
    rospy.loginfo("Camera Ready at " + str(rospy.get_rostime().secs))
    rospy.spin()

if __name__ == "__main__":
    gphoto_cam_server()
