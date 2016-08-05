#!/bin/bash

export ROS_IP=$(/sbin/ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_MASTER_URI=http://victoria.gel.ulaval.ca:11311
export CAMNET_CAM_NAME=Skycam
export CAMNET_OUTPUT_DIR=/home/pi/camera-output/*
#export CAMNET_SERVER_DATA_DIR=dobil18@rachmaninoff.gel.ulaval.ca:/gel/rachmaninoff/data/pictures/unprocessed/raw_data/progressiveDL/
export THETA_SERVER_DATA_DIR=dobil18@rachmaninoff.gel.ulaval.ca:/home/debussy/sibelius/thetaS/
export ENABLE_PROGRESSIVE_DL=True
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

/bin/ps aux | /bin/grep roslaunch | /bin/grep camera_controler_gphoto.launch &> /dev/null
if [ $? -ne 0 ]; then
    source ~/camera-network/client/install/setup.bash
    CAMERA_NAME=$CAMNET_CAM_NAME roslaunch ~/camera-network/client/src/camera_controler/launch/camera_controler_gphoto.launch
fi
