#!/bin/bash
source ./install/setup.bash
ROS_IP=$(/sbin/ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/') roslaunch /home-local/yahog.extra.nobkp/CameraNetwork/server/install/share/camera_master_server/launch/camera_master_server.launch
