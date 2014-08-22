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
$ sudo apt-get install ros-<rosversion>-rosbridge-server
$ sudo apt-get install ros-<rosversion>-mjpeg-server
```
### Setting up network ###
```
$ echo 'export ROS_IP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')' >> ~/.bashrc  
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
```

#### Installing Paramiko (sftp transfer) ####
```
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
$ sudo apt-get install python-picamera daemontools
$ echo ‘source /opt/ros/hydro/setup.bash ’ >> ~/.bashrc
```


# Install Camera Network #

will generate camera-network folder:

### setup ROS workspace ###
```
$ git clone https://MathieuGaron@bitbucket.org/MathieuGaron/camera-network.git
$ cd camera-network/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo ./install.sh
```

### Install gphoto and upstart ###
```
$ sudo ./gphoto2-updater.sh 
$ sudo apt-get install upstart  
```

### Setup Upstart Job ###
This section launches the camera controller upon interface connection. It is based on robot_upstart (turtlebot). Detail about this system can be found here: http://wiki.ros.org/robot_upstart .

To create a new upstart service for the device, execute:
```
$ rosrun robot_upstart install camera_controler/launch/camera_controler_gphoto.launch --interface wlan0 --master http://<MASTER'S URL>:11311 --setup /home/pi/ros_catkin_ws/install_isolated/setup.bash 
```
You will then be able to start and enable the service.

To configure the camera, add these lines at the beginning of /etc/init/camera.conf:
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
