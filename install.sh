#!/bin/bash
#"""
#Created on Wed Jun 18 09:31:27 2014
#
#@author: mathieu
#"""

mkdir /home/CameraNetwork
mkdir /home/CameraNetwork/preview
mkdir /home/CameraNetwork/log
mkdir /home/CameraNetwork/master
mkdir /home/CameraNetwork/data

chmod -R 777 /home/CameraNetwork

if [[ $1 = 'netrst' ]];
    then
	echo "ping -c4 www.google.ca > /dev/null
	if [ \$? != 0 ]
	then
	sudo /sbin/shutdown -r now
	fi" > /usr/local/bin/checkwifi.sh

	chmod +x /usr/local/bin/checkwifi.sh

	crontab -l | { cat; echo "*/5 * * * * /usr/bin/sudo -H /usr/local/bin/checkwifi.sh >> /dev/null 2>&1"; } | crontab -
	echo "Device Will reboot if no network for 5 minutes"
fi

echo "Camera Network's folder ready"
echo "use : ./install.sh netrst to set reboot when network is down"
