# Dependencies #

## Server Setup ##
This installation process is compatible with ubuntu distros.
The application is tested with hydro and indigo versions of ROS

### Installing Ros ###
http://wiki.ros.org/hydro/Installation/Ubuntu   
http://wiki.ros.org/indigo/Installation/Ubuntu

(from now on chande <rosversion> depending on wich one you installed)

### Installing Additionnal Ros Packages ###
```
$ sudo apt-get install ros-<rosversion>-rosbridge-server
$ sudo apt-get install ros-<rosversion>-mjpeg-server
```
### Setting up network ###
```
$  echo 'export ROS_IP=$(ip addr | grep '"'"'state UP'"'"' -A2 | tail -n1 | awk '"'"'{print $2}'"'"' | cut -f1  -d'"'"'/'"'"')' >> ~/.bashrc
$ echo export ROS_MASTER_URI=http://<MASTER'S URL>:11311 >> ~/.bashrc
```

### Python and dependencies ###

Python 2.7 is needed for the Catkin package handler. It is recommended to create a virtual environment for your Python installation. Once it's done, install the following dependencies:
```
$ pip install catkin_pkg
$ pip install empy
$ pip install pyyaml
$ pip install rospkg
$ pip install pillow
$ pip install envoy
$ pip install exifread
```

Because ROS Indigo uses an old version of PIL to work, you must setup a proxy to the new Image module as such:
```
$ echo "from PIL.Image import *" >> <your-python-site-packages>/Image.py
```

### Launching ###

TODO: Improve this.

```
$ cd <this-repo>
$ cd server/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ catkin_make install
$ source ./install/setup.bash
$ roscd camera_master_server
$ ln -s ../../../src/camera_master_server/scripts/* ./
$ ln -s ../../../src/camera_master_server/launch/* ./
$ roslaunch camera_master_server.launch
```

## Client Installation ##
The client holds the camera drivers and publishing nodes. It must be install on every system connected to a camera.

This setup takes for granted that you have already installed a Linux distribution on your system.

### ROS and packages ###
Install ROS Hydro using the following instructions: http://wiki.ros.org/ROSberryPi/Setting%20up%20Hydro%20on%20RaspberryPi

Then, install the following packages (This may take a lot of time!):
```
$ cd ~/ros_catkin_ws/src
$ roslocate info common_msgs | rosws merge -
$ roslocate info actionlib | rosws merge -
$ roslocate info class_loader | rosws merge -
$ roslocate info pluginlib | rosws merge -
$ roslocate info cv_bridge | rosws merge -
$ roslocate info image_transport | rosws merge -
$ rosinstall_generator opencv2 --deps | rosws merge -
$ rosws update
$ cd ..
$ rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
$ ./src/catkin/bin/catkin_make_isolated --install
$ sudo apt-get install python-picamera daemontools
$ echo ‘source /opt/ros/hydro/setup.bash ’ >> ~/.bashrc
```

# Camera Network Installation #

will generate camera-network folder and generate key to the server:

### setup ROS workspace ###
```
$ git clone https://github.com/lvsn/CameraNetwork.git
$ cd camera-network/<client-or-server>/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ catkin_make install
$ ln -sv ~/camera-network/client/src/camera_controler/launch ~/camera-network/client/install/share/camera_controler/
$ ln -sv ~/camera-network/client/src/camera_controler/param ~/camera-network/client/install/share/camera_controler/
$ ln -sv ~/camera-network/client/src/camera_controler/scripts/*.py ~/camera-network/client/install/share/camera_controler/
$ ln -sv ~/camera-network/client/src/camera_drivers/launch ~/camera-network/client/install/share/camera_drivers/
$ ln -sv ~/camera-network/client/src/camera_drivers/scripts/*.py ~/camera-network/client/install/share/camera_drivers/
```

### Install gphoto ###
```
$ sudo ./gphoto2-updater.sh
```

### Install Bluetooth support ###
```
$ sudo apt-get install python-gobject bluez bluez-tools python-bluez python-dev
```
In  /etc/bluetooth/main.conf, change the following line
```
#DisablePlugins = network,input
```
to :
```
DisablePlugins = network,input,audio,pnat,sap,serial
```
Restart Daemon.
```
$ sudo /etc/init.d/bluetooth restart
```

### Setup Client Launchfile ###

Configure camera_control.launch as you need:
(the param file can be changed)
(the camera control launch file can be changed to picam or gphoto)
