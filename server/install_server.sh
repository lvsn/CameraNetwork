#!/bin/bash

#################################
# Install Camera-Network Server #
#################################

echo '*** CAMNET SERVER INSTALLATION BEGIN ***'
set +e
cd ~/camera-network/server/
echo '*** CATKIN_MAKE ***'
catkin_make
echo '*** CATKIN_MAKE INSTALL ***'
catkin_make install
echo '*** INSTALL CUSTOM FILES ***'
ln -sv ~/camera-network/server/src/camera_master_server/launch ~/camera-network/server/install/share/camera_master_server/
ln -sv ~/camera-network/server/src/camera_master_server/scripts/*.py ~/camera-network/server/install/share/camera_master_server/
set -e
echo '*** CAMNET SERVER INSTALLATION DONE ***'

