#!/bin/bash

##########################
# Install Camera-Network #
##########################

echo '*** CAMNET CLIENT INSTALLATION BEGIN ***'
set +e
cd ~/camera-network/client/
echo '*** CATKIN_MAKE ***'
catkin_make
echo '*** CATKIN_MAKE INSTALL ***'
catkin_make install
echo '*** INSTALL CUSTOM FILES ***'
ln -sv ~/camera-network/client/src/camera_controler/launch ~/camera-network/client/install/share/camera_controler/
ln -sv ~/camera-network/client/src/camera_controler/param ~/camera-network/client/install/share/camera_controler/
ln -sv ~/camera-network/client/src/camera_controler/scripts/*.py ~/camera-network/client/install/share/camera_controler/
ln -sv ~/camera-network/client/src/camera_drivers/launch ~/camera-network/client/install/share/camera_drivers/
ln -sv ~/camera-network/client/src/camera_drivers/scripts/*.py ~/camera-network/client/install/share/camera_drivers/
set -e
echo '*** CAMNET CLIENT INSTALLATION DONE ***'
