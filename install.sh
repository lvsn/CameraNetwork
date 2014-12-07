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
echo "Enter remote's password"
ssh-copy-id user@192.168.0.2

echo "make sure the keygen is working before running camnet"

