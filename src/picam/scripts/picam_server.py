#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu June 05 09:17:30 2014

@author: Mathieu Garon
"""
import roslib; roslib.load_manifest('picam')
import rospy
from camera_network_msgs.srv import *

import picamera
import os
import time
import picamParameterHandler as pph


class picam_server:
    def __init__(self):
        try:
            self.picam = picamera.PiCamera()
        except:
            rospy.logfatal("Check if the Picam is free or installed")

        self.picam.awb_mode = 'off'
        self.picam.awb_gains = 1.5


        self.camParam = pph.PicameraParameterHandler()
        self.camParam.set_camera_parameters()
        self.homePath = "/home/CameraNetwork"
        self.tmpPath = self.homePath + '/tmp'

        #creating a large id generator so pictures are not overwrited
        self.id_gen = self._id_generator()

        rospy.Service('capture_camera',CaptureService,self.capture_image_cb)
        rospy.Service('load_camera',Load,self.load_camera_cb)
        rospy.Service('get_camera',OutCameraData,self.get_camera_cb)
        rospy.Service('set_camera',InCameraData,self.set_camera_cb)
        rospy.loginfo("Camera Ready")
        rospy.spin()

    def __del__(self):
        self.picam.close()

    def capture_image_cb(self,req):
        rospy.loginfo("Taking Picture")
        if not os.path.exists( self.tmpPath):
            os.makedirs( self.tmpPath)
        pictureFileName =  self.tmpPath + '/unloaded_' + self.id_gen.next() + '.' + self.camParam.get_format() 
        self.picam.capture( pictureFileName, format=self.camParam.get_format())
        return 'image saved as ' + pictureFileName

    def load_camera_cb(self,req):
        #reset generator
        self.id_gen = self._id_generator()
        loadPath = self.homePath + "/" + self._filename_format(req.path,0,'dummy')  #to make sure it create the right path
        if loadPath.find('..') != -1:
            rospy.logwarn("use of .. is prohibed")
            return "error"
        directory = os.path.dirname(loadPath)
        rospy.loginfo("Loading Picture to folder " + directory)
        if not os.path.exists(directory):
            os.makedirs( directory)
        count = 0
        for pictureFile in os.listdir(self.tmpPath):
            fileFormat =pictureFile.split('.')[-1]
            os.rename( self.tmpPath + "/" + pictureFile, self.homePath + "/" + self._filename_format(req.path,count,fileFormat))
            count += 1
        return "Transfered " + str(count) + " files."

    def set_camera_cb(self,req):
        rospy.loginfo("Setting camera's Configuration to " + str(req))
        if(req.iso != ""):
            self.picam.ISO = int(float(req.iso))
        if(req.imageformat != ""):
            self.camParam.set_format(req.imageformat)
        if(req.aperture != ""):
            rospy.logwarn("aperture is not supported on picam")
        if(req.shutterspeed != ""):
            self.picam.shutter_speed = int(float(req.shutterspeed))

        return "Picam set"

    def get_camera_cb(self,req):
        rospy.loginfo("Getting camera's Configuration")
        iso = str(self.picam.ISO)
        imageformat = str(self.camParam.get_format())
        aperture = "not supported"
        shutterspeed = str(self.picam.shutter_speed)
        
        return {'iso':iso,'imageformat':imageformat,'aperture':aperture,'shutterspeed':shutterspeed}
    
    def _filename_format(self,string,pictureId,pictureFormat):
        string = string.replace('%C',pictureFormat)
        string = string.replace('%n', str(pictureId))
        return time.strftime(string)

    def _id_generator(self):
        for i in range(10000000):
            yield str(i)


if __name__ == "__main__":
    rospy.init_node('picam')
    server = picam_server()
