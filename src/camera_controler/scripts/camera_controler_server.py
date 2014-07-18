#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 28 16:19:15 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('camera_controler')
import rospy
import timelaps_server as ts
import network_capture_listener as ncl
import camera_handler as ch
import parameter_save as ps
import os
import std_srvs.srv
from camera_network_msgs.srv import *
import subprocess


class server:
    
    def __init__(self):
        rospy.init_node('timelaps_server')
        
        self.cam_handler = ch.CameraHandler()
        self.timelapsServer = ts.TimelapsServer(self.cam_handler)
        self.listener = ncl.network_capture_listener(self.cam_handler)
        
        self.listener.listen()
        #setup server to set camera init parameters
        self.paramSaver = ps.save_server()
        rospy.Service('preview_camera', std_srvs.srv.Empty(), self.preview_image_cb)
        rospy.Service('shutdown_device', CommandOption,self.shutdown_device_cb)
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.cam_handler
        del self.listener
        del self.timelapsServer
        rospy.delete_param('camera_setting')
        rospy.delete_param('file')
        rospy.delete_param('/IP/' + os.environ['CAMERA_NAME'])

    def preview_image_cb(self,req):
        self.cam_handler.takePreview()
        directory = "/home/CameraNetwork/preview/"
        for filename in os.listdir(directory):
            f, extention = os.path.splitext(filename)
            if extention not in [".jpg",".jpeg",".JPG",".JPEG"]:
                rospy.logwarn("Camera is not set to JPG: current format = " + extention)
                self._delete_file(filename,directory)
            else:
                rospy.loginfo("file " + f + ".jpeg is ready")
                self._rename_file(filename,directory)
        return []
    
    def shutdown_device_cb(self,req):       
        if req.option in ['','-h','-r']:
            command = "/usr/bin/sudo /sbin/shutdown " + req.option +" now"
            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
            process.communicate()[0]
            rospy.loginfo("Shutdown command launched with option : " + req.option)
        else:
            rospy.logwarn("Option Invalid")
        return []
        
    def _delete_file(self,filename,directory=""):
        try:
            os.remove(directory + filename)
        except:
            rospy.logerr("Problem while deleting " + directory+filename)
        
    def _rename_file(self,filename,directory=""):
        try:
            os.rename(directory + filename, directory+f+".jpeg")
        except:
            rospy.logerr("Problem while renaming" + directory+filename)

if __name__ == "__main__":
    serverInstance = server()
