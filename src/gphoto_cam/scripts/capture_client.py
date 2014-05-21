#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""

import rospy
import roslib; roslib.load_manifest('gphoto_cam')
from gphoto_cam.srv import *

import sys
import string



def capture_image_client(keepOnCamera):
    rospy.wait_for_service('capture_camera')
    
    try:
        capture_camera = rospy.ServiceProxy('capture_camera', Capture)
        capture_camera(keepOnCamera)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s",e)

def usage():
    return "%s [bool] (keep Image On Camera Device)"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        keepOnCamera = string.atoi(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    if(keepOnCamera):
        rospy.loginfo("Picture will be saved in Camera")
    else:
        rospy.loginfo("Waiting to receive Picture")
    capture_image_client(keepOnCamera)
