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



def capture_image_client(keepOnCamera):
    rospy.wait_for_service('capture_image')
    
    try:
        capture_image = rospy.ServiceProxy('capture_image', Capture)
        capture_image(keepOnCamera)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [bool] (keep Image On Camera Device)"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        keepOnCamera = bool(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    if(keepOnCamera):
        print "Picture will be saved in Camera"
    else:
        print "Waiting to receive Picture"
    capture_image_client(keepOnCamera)
