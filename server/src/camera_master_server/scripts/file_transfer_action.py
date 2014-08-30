#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri May 23 14:27:55 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com


"""
import os
import time
import math
import traceback
from xml.dom import minidom

import actionlib
import rospy
import rospkg
import std_srvs.srv
import paramiko

from camera_network_msgs.msg import *
from camera_network_msgs.srv import *


join = os.path.join


class sftp_action:

    """Implements the picture fetching functionality with ROS."""

    def __init__(self, imagePath):
        self.imagePath = imagePath
        self.dateFolder = ''
        self.localImagePath = join(self.imagePath, 'master', self.dateFolder)
        self.localLogPath = join(self.imagePath, 'log')
        self.fileQty = 0

        rospy.loginfo('Writing files to ' + self.localImagePath)
        rospy.loginfo('Wrinting log to ' + self.localLogPath)
        rospy.loginfo('Setting up sftp Action')

        self.action = actionlib.SimpleActionServer(
            'sftp',
            CameraDownloadAction,
            self.execute,
            False)
        self.action.start()
        rospy.Service('add_user', User, self.add_user)
        rospy.Service('delete_users', std_srvs.srv.Empty(), self.delete_users)
        rospy.Service('save_users', std_srvs.srv.Empty(), self.save_users)
        rospy.Service('get_users', BackMessage, self.get_users)

        self.ipDict = {}
        self.userDict = {}
        self._parse_userXML()

        # setup logging
        self.create_dir(self.localLogPath)
        paramiko.util.log_to_file(
            self.localLogPath +
            time.strftime('%d%B%Hh') +
            '.log')

    def execute(self, goal):
        totalCount = 0

        hz = self._sec_to_hz(goal.dowload_frequency_s)
        self._sleep_until(goal.start_time)

        r = rospy.Rate(hz)
        while True:
            self.refresh_ip()
            self.refresh_date()

            try:
                pictureQty = self.download_all_images_from_network()
            except (AttributeError, ):
                # Could not iterate over self.ipDict
                pictureQty = 0
            totalCount += pictureQty
            r.sleep()
            if self.action.is_preempt_requested() or not self.action.is_active(
            ) or goal.dowload_frequency_s == 0:
                break
        succes_msg = CameraDownloadActionResult
        succes_msg.total_downloaded = 'Downloaded ' + \
            str(totalCount) + ' pictures from ' + str(len(self.ipDict)) + ' devices.'

        self.rospack = rospkg.RosPack()
        self.action.set_succeeded(succes_msg)

    def add_user(self, req):
        self.userDict[req.name] = (req.username, req.password)
        rospy.loginfo("Users : " + str(self.userDict))
        return []

    def delete_users(self, req):
        rospy.loginfo("Users deleted")
        self.userDict = {}
        return[]

    def save_users(self, req):
        rospy.loginfo("users.xml saved")
        self._create_userXML()
        return []

    def get_users(self, req):
        rospy.loginfo("Get users information")
        msg = ''
        for key in self.userDict:
            usr, passw = self.userDict[key]
            msg += key + ': Username=' + usr + ' Password=' + passw + '\n'
        return msg

    def refresh_ip(self):
        self.ipDict = rospy.get_param('/IP', {})
        rospy.loginfo("Refreshing ip dic: " + str(self.ipDict))

    def refresh_date(self):
        self.dateFolder = time.strftime("%B")
        self.localImagePath = join(self.imagePath, 'master', self.dateFolder)

    def download_all_images_from_network(self):
        """
        Open sftp session to download images of each session
        return a tuple of number of device and total transfered images
        """
        imageQty = 0
        for name, ip in self.ipDict.items():
            try:
                rospy.loginfo(
                    'start sftp session with {0} on ip {1} and port 22'
                    ''.format(name, ip)
                )
                sftp, t = self._createSession(ip, name)
                if sftp is not None and t is not None:
                    imageQty += self._downloadImageFolder(sftp, name)
                    t.close()
                    rospy.loginfo('Session with ' + ip + ' closed')
            except Exception as e:
                rospy.logerr(traceback.format_exc())
                try:
                    t.close()
                except:
                    pass
        return imageQty

    def _createSession(self, ip, name, port=22):
        try:
            rospy.loginfo("Connecting to {0}:{1}".format(ip, port))
            t = paramiko.Transport((ip, port))
            rospy.loginfo("Connecting...")
            (username, passw) = self._get_user_and_passw(name)
            t.connect(username=username, password=passw)
            rospy.loginfo("SFTP client launch...")
            sftp = paramiko.SFTPClient.from_transport(t)
        except:
            rospy.logwarn("Unable to connect to " + str(ip))
            return (None, None)
        return (sftp, t)

    def _downloadImageFolder(self, sftp, deviceName=''):
        feedback_msg = CameraDownloadActionFeedback
        username, passw = self._get_user_and_passw(deviceName)
        remote_dir = "/home/{username}/Pictures".format(**locals())

        filelist = sftp.listdir(os.path.join(remote_dir, self.dateFolder))
        rospy.loginfo('found {0} files'.format(len(filelist)))
        self.create_dir(self.localImagePath + deviceName)
        count = 1.0
        if len(filelist) == 0:
            rospy.loginfo("No file to download")
        else:
            for f in filelist:
                rospy.loginfo('Downloading {0}'.format(f))
                remoteFile = os.path.join(remote_dir, self.dateFolder, f)
                localFile = os.path.join(self.localImagePath, deviceName, f)
                rospy.logwarn("{0} -> {1}".format(remoteFile, localFile))
                try:
                    sftp.get(remoteFile, localFile)
                    sftp.remove(remoteFile)
                except:
                    rospy.logwarn(traceback.format_exc())
                feedback_msg.picture_downloaded = "{0:.2f}".format(
                    float(
                        count / len(filelist) * 100)) + '% of Device ' + deviceName
                self.action.publish_feedback(feedback_msg)
                count += 1
        return len(filelist)

    def create_dir(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def _sec_to_hz(self, Tsec):
        try:
            hz = math.fabs(1 / Tsec)
            rospy.loginfo("Download Frequency set to " + str(hz) + " hz.")
        except ZeroDivisionError:
            hz = -1
            rospy.logwarn(
                "Can not set 0 as frequency... setting frequency to 1 hz")
        return hz

    def _sleep_until(self, timestamp):
        delta = timestamp - rospy.get_time()
        if delta > 0:
            rospy.loginfo("Wainting for " + str(delta) + " seconds")
            while timestamp > rospy.get_time():
                rospy.sleep(5)
                if self.action.is_preempt_requested(
                ) or not self.action.is_active():
                    rospy.loginfo("Download timer interupted")
                    break

    def _parse_userXML(self, filename='users.xml'):
        try:
            xmldoc = minidom.parse(
                rospkg.RosPack().get_path('camera_master_server') +
                '/' +
                filename)
            for node in xmldoc.getElementsByTagName('device'):
                deviceName = node.getElementsByTagName(
                    'name')[0].childNodes[0].data
                (username, password) = node.getElementsByTagName('user')[0].childNodes[
                    0].data, node.getElementsByTagName('pass')[0].childNodes[0].data
                self.userDict[deviceName] = (username, password)
        except:
            rospy.logwarn(
                "Unable to load users.xml, will use default raspberry pi user for all devices")

    def _create_userXML(self, filename='users.xml'):
        xmldoc = minidom.Document()
        data = xmldoc.createElement('data')
        xmldoc.appendChild(data)
        for key in self.userDict:
            (username, password) = self.userDict[key]
            device = xmldoc.createElement('device')
            data.appendChild(device)

            name = xmldoc.createElement('name')
            name_content = xmldoc.createTextNode(key)
            name.appendChild(name_content)
            device.appendChild(name)

            user = xmldoc.createElement('user')
            user_content = xmldoc.createTextNode(username)
            user.appendChild(user_content)
            device.appendChild(user)

            passw = xmldoc.createElement('pass')
            passw_content = xmldoc.createTextNode(password)
            passw.appendChild(passw_content)
            device.appendChild(passw)
        with open(rospkg.RosPack().get_path('camera_master_server') + '/' + filename, 'w') as f:
            f.write(xmldoc.toprettyxml(indent="    ", encoding="utf-8"))

    def _get_user_and_passw(self, name):
        if name in self.userDict:
            return self.userDict[name]
        else:
            return ('pi', 'raspberry')


if __name__ == "__main__":
    rospy.init_node('sftp_action')
    sftp = sftp_action(os.path.expanduser("~/Pictures"))
    rospy.spin()
