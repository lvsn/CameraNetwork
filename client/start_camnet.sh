#!/bin/bash

#export ROS_IP=$(/sbin/ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_IP=localhost
export ROS_MASTER_URI=http://localhost:11311
#export CAMNET_OUTPUT_DIR=/home
export CAMNET_SERVER_DATA_DIR=~/Pictures
#export CAMNET_SERVER_USER=

/bin/ps aux | /bin/grep roslaunch | /bin/grep camera_controler_gphoto.launch &> /dev/null
if [ $? -ne 0 ]; then
    source ~/camera-network/client/devel/setup.bash
    CAMERA_NAME=SkyCam roslaunch ~/camera-network/client/src/camera_controler/launch/camera_controler_gphoto.launch
fi
