#!/bin/bash

#################################
# Install Camera-Network Server #
#################################

ROS_SERVER_PATH=/home-local/yahog.extra.nobkp/CameraNetwork

echo '*** CAMNET SERVER INSTALLATION BEGIN ***'
set +e
cd $ROS_SERVER_PATH/server/
echo '*** CATKIN_MAKE ***'
catkin_make
echo '*** CATKIN_MAKE INSTALL ***'
catkin_make install
echo '*** INSTALL CUSTOM FILES ***'
ln -sv $ROS_SERVER_PATH/server/src/camera_master_server/launch $ROS_SERVER_PATH/server/install/share/camera_master_server/
ln -sv $ROS_SERVER_PATH/server/src/camera_master_server/scripts/*.py $ROS_SERVER_PATH/server/install/share/camera_master_server/
set -e
echo '*** CAMNET SERVER INSTALLATION DONE ***'

