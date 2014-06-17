Camera Network is a camera sensor project developped with ROS (Robotic Operating System) to communicate easily with multiple camera device on the network.

For now, Camera Network is compatible with gphoto and the picamera module.

You can communicate with the network directly with ROS, or with the WebGUI (if the master-server is launched with the right webserver url)

# Dependencys #

## Master Server Setup ##
This installation process is compatible with ubuntu distros.
The application is tested with hydro and indigo versions of ROS

### Installing Ros ###
http://wiki.ros.org/hydro/Installation/Ubuntu   
http://wiki.ros.org/indigo/Installation/Ubuntu

(from now on chande <rosversion> depending on wich one you installed)

### Installing Additionnal Ros Packages ###
```
#!bash
$ sudo apt-get install ros-<rosversion>-rosbridge-server
$ sudo apt-get install ros-<rosversion>-mjpeg-server
```
### Setting up network ###
```
#!bash
$ echo 'export ROS_IP=$(ifconfig eth0 | grep "inet addr:" | cut -d: -f2 | awk "{ print $1}")' >> ~/.bashrc  
$ echo export ROS_MASTER_URI=http://<MASTER'S URL>:11311 >> ~/.bashrc   
```

### Installing Paramiko (sftp transfer) ###
```
#!bash
$ wget https://github.com/paramiko/paramiko/archive/master.zip
$ unzip master.zip
$ cd paramiko-master
$ sudo easy_install ./
```

## Raspberry Pi Installation ##
This one is a bit harder:

### Installing Debian ###
-Install Raspbian (ex : NOOBS : http://www.raspberrypi.org/help/noobs-setup/)

### ROS ###
Install ros hydro : http://wiki.ros.org/ROSberryPi/Setting%20up%20Hydro%20on%20RaspberryPi

### Ros packages ###
This part takes a lot of time!


```
#!bash

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
```


```
#!bash
$ sudo apt-get install python-picamera daemontools
$ echo ‘source /opt/ros/hydro/setup.bash ’ >> ~/.bashrc
```