#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu May 15 11:13:38 2014

@author: mathieugaron
"""

import roslib; roslib.load_manifest('gphoto_cam')

import sys

import rospy

from gphoto_cam.srv import *


def get_client(getAllInformation):
    rospy.wait_for_service('get_camera')
    
    try:
        get_camera = rospy.ServiceProxy('get_camera', OutCameraData)
        ans = get_camera(getAllInformation)
        rospy.loginfo(ans)
    except rospy.ServiceException, e:
       rospy.logwarn("Service call failed: %s",e)

def usage():
    return "%s [bool] (get_all_information?)"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        pass
    else:
        print usage()
        sys.exit(1)
    rospy.loginfo("Get Camera's information")
    get_client(True)