#!/bin/bash

export ROS_IP=$(/sbin/ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_MASTER_URI=http://victoria.gel.ulaval.ca:11311
export CAMNET_CAM_NAME=unSpecCam
CAMNET_OUTPUT_DIR=/home/pi/camera-output/*
export CAMNET_SERVER_DATA_DIR=JUBEC7@victoria.gel.ulaval.ca:/home-local/yahog.extra.nobkp/www/pictures/unprocessed/

/bin/ps aux | /bin/grep roslaunch | /bin/grep camera_controler_gphoto.launch &> /dev/null
if [ $? -ne 0 ]; then
    source ~/camera-network/client/install/setup.bash
    CAMERA_NAME=$CAMNET_CAM_NAME roslaunch ~/camera-network/client/src/camera_controler/launch/camera_controler_gphoto.launch
fi
