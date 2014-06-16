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
import std_srvs.srv
import os

class server:
    
    def __init__(self):
        rospy.init_node('timelaps_server')
        self.cam_handler = ch.CameraHandler()
        self.timelapsServer = ts.TimelapsServer(self.cam_handler)
        self.listener = ncl.network_capture_listener(self.cam_handler)
        self.listener.listen()
        self.paramSaver = ps.save_server()
        rospy.Service('preview_camera', std_srvs.srv.Empty(), self.preview_image_cb)
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        del self.cam_handler
        del self.listener
        del self.timelapsServer
        rospy.delete_param('camera_setting')
        rospy.delete_param('file')
        
    def preview_image_cb(self,req):
        self.cam_handler.takePreview()
        directory = os.environ['HOME'] + "/CameraPicture/preview/"
        filelist = os.listdir(directory)
        for filename in filelist:
            f, e = os.path.splitext(filename)
            if e not in [".jpg",".jpeg",".JPG",".JPEG"]:
                rospy.logwarn("Camera is not set to JPG: current format = " + e)
                try:
                    os.remove(directory + filename)
                except:
                    rospy.logerr("Problem while deleting " + directory+filename)

            else:
                rospy.loginfo("file " + f + ".jpeg is ready")
                try:
                    os.rename(directory + filename, directory+f+".jpeg")
                except:
                    rospy.logerr("Problem while renaming" + directory+filename)
        return []
        

if __name__ == "__main__":
    
    serverInstance = server()
