#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('gphoto_cam')

import sys

import rospy

from gphoto_cam.srv import *


def set_client(iso,imageformat,aperture,shutterspeed):
    rospy.wait_for_service('set_camera')
    
    try:
        set_camera = rospy.ServiceProxy('set_camera', InCameraData)
        out = set_camera(iso,imageformat,aperture,shutterspeed)
        rospy.loginfo(out)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s",e)

def usage():
    return "%s [string string string string] (iso imageformat aperture shutterspeed)"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        iso = str(sys.argv[1])
        imageformat = str(sys.argv[2])
        aperture = str(sys.argv[3])
        shutterspeed = str(sys.argv[4])
    else:
        print usage()
        sys.exit(1)
    rospy.loginfo("set parameter to camera")
    set_client(iso,imageformat,aperture,shutterspeed)
