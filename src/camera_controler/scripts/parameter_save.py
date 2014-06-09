#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  9 09:31:45 2014

@author: mathieugaron
"""

import roslib
roslib.load_manifest("rosparam")
roslib.load_manifest("camera_controler")
import std_srvs.srv
import rosparam
import rospkg
import rospy

class save_server():
    
    def __init__(self):
        rospy.Service('save_config', std_srvs.srv.Empty(), self.save_settings)
    
    def save_settings(self,req):
        rospack = rospkg.RosPack()
        path = rospack.get_path('camera_controler')
        yamlFile = rospy.get_param('file')
        rosparam.yamlmain(['rosparam', 'dump',path + '/param/' + yamlFile,'camera_setting'])
        rospy.loginfo("Saved camera_setting parameters to : " + path + '/param/' + yamlFile )
        return {}

if __name__ == "__main__":
    server = save_server()

