#!/bin/bash
#"""
#Created on Wed Jun 18 09:31:27 2014
#
#@author: mathieu
#"""


mkdir ~/Pictures/preview
mkdir ~/Pictures/log
mkdir ~/Pictures/server
mkdir ~/Pictures/data

echo "CamNet's folder ready"


ssh-keygen -b 2048 -t rsa -f ~/.ssh/id_rsa -q -N ""

echo "Enter server's user"
read user
echo "Enter server's IP or hostname"
read ip

echo "pair key to "$user@$ip

ssh-copy-id $user@$ip

echo "Installation COMPLETE"
echo "make sure the keygen is working before running camnet"

