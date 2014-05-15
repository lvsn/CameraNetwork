#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
"""

import rospy
from gphoto_cam.srv import *

import os.path
from datetime import datetime, timedelta
import gphoto2_cli_wrapper as gphoto


def capture_image_cb(req):
    print "Taking Shot"
    if(req.keepOnCamera):
        gphoto.run(" --capture-image")
    else:
        gphoto.run(" --capture-image-and-download")
    return 'OK'


def capture_image_server():
    rospy.init_node('capture_camera')
    s = rospy.Service('capture_camera', Capture, capture_image_cb)
    print "Ready to capture Image."
    rospy.spin()

if __name__ == "__main__":
    capture_image_server()
