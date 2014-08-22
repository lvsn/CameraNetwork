# Dependencies #

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
$ echo 'export ROS_IP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')' >> ~/.bashrc  
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


# Install Camera Network #

will generate camera-network folder:

### setup ROS workspace ###
```
#!bash
$ git clone https://MathieuGaron@bitbucket.org/MathieuGaron/camera-network.git
$ cd camera-network/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo ./install.sh
```

### Install gphoto and upstart ###
```
#!bash
$ sudo ./gphoto2-updater.sh 
$ sudo apt-get install upstart  
```

### Setup Upstart Job ###
This section create an upstart job when the desired interface start or stop:

here, set the interface of your choice, and the master's URL/IP
```
#!bash
$ rosrun robot_upstart install camera_controler/launch/camera_controler_gphoto.launch --interface wlan0 --master http://<MASTER'S URL>:11311 --setup /home/pi/ros_catkin_ws/install_isolated/setup.bash 
```
add these lines at the beginning of /etc/init/camera.conf:
```
 setuid pi  
 setgid plugdev   //for Gphoto
 setgid video      //for Picam
```

** you can't add the two setgid! **


add this line in /usr/sbin/camera-start:

```
export CAMERA_NAME=<Unique name>
```

Configure camera_control.launch as you need:
(the param file can be changed)
(the camera control launch file can be changed to picam or gphoto)
