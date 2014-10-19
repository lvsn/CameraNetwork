#!/bin/bash

export ROS_IP=$(/sbin/ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_MASTER_URI=http://victoria.gel.ulaval.ca:11311

/bin/ps aux | /bin/grep roslaunch | /bin/grep camera_controler_gphoto.launch &> /dev/null
if [ $? -ne 0 ]; then
    source /home/pi/camera-network/client/install/setup.bash
    CAMERA_NAME=SkyCam roslaunch /home/pi/camera-network/client/install/share/camera_controler/camera_controler_gphoto.launch
fi
