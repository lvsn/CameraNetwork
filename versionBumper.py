#!/usr/bin/env python

from xml.etree import ElementTree as et
import sys

if len(sys.argv) != 2:
    print "Add Version as only argument ex: 1.0.0"
else:
    newVersion = str(sys.argv[1])    
    print "Versions bumped to :" + newVersion
    camera_controlerPath = 'src/camera_controler/package.xml'
    tree = et.parse(camera_controlerPath)
    tree.find('.//version').text = newVersion
    tree.write(camera_controlerPath)
    
    camera_controlerPath = 'src/camera_master_server/package.xml'
    tree = et.parse(camera_controlerPath)
    tree.find('.//version').text = newVersion
    tree.write(camera_controlerPath)
    
    camera_controlerPath = 'src/camera_network_msgs/package.xml'
    tree = et.parse(camera_controlerPath)
    tree.find('.//version').text = newVersion
    tree.write(camera_controlerPath)
    
    camera_controlerPath = 'src/gphoto_cam/package.xml'
    tree = et.parse(camera_controlerPath)
    tree.find('.//version').text = newVersion
    tree.write(camera_controlerPath)
    
    camera_controlerPath = 'src/picam/package.xml'
    tree = et.parse(camera_controlerPath)
    tree.find('.//version').text = newVersion
    tree.write(camera_controlerPath)
