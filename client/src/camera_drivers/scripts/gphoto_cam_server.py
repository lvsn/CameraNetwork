#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:00:06 2014

@author: mathieugaron
@email: mathieugaron1991@hotmail.com

Gphoto driver, implement all basic services to make cameracontroler able to communicate
with it.
"""

import roslib

roslib.load_manifest('camera_drivers')
import rospy
from camera_network_msgs.srv import *
import os, sys
sys.path.append(os.path.expanduser('~/camera-network/'))
import camera_driver as cd
import Image
import numpy
import thread
from scripts.Util.command import *

join = os.path.join
gphoto2Executable = 'gphoto2'

print("ICI!")

class GPhotoServer(cd.camera_driver):
    def __init__(self):
        self.terminateThreads = False
        rospy.loginfo("Initializing Gphoto camera driver")
        # Camera's configuration name
        self.isoConfig = "iso"
        self.apertureConfig = "aperture"
        self.shutterspeedConfig = "shutterspeed"
        self.imageformatConfig = "imageformat"
        self.camera = ''
        thread.start_new_thread(GPhotoServer._find_camera_thread, (self, ))
        #self._update_camera_parameters()

        super(GPhotoServer, self).__init__()

        rospy.spin()


    def capture_image_cb(self, req):
        rospy.loginfo("Taking Picture")
        rospy.sleep(req.timer)
        gphotoCommand = ""
        if self.parameterQueue:
            #gphotoCommand = " --capture-image --wait-event=1s --keep"
            gphotoCommand = self._build_sequence_command(self.parameterQueue)
            rospy.loginfo(gphotoCommand)
            self.parameterQueue = []
        else:
            gphotoCommand = " --capture-image --wait-event=1s --keep"
        msg = self._run_gphoto(gphotoCommand)
        return msg

    def preview_capture_cb(self, req):
        self._run_gphoto(" --capture-image --wait-event=2s")
        num = self._get_picture_qty()
        #--no-keep is not working this command simulate it
        filename = self.homePath + "/previewImage"
        self._run_gphoto(" --filename " + filename + " -R -p " + str(num) + " -d " + str(num))
        picture = Image.open(filename).resize((320, 240))
        self._publish_picture(numpy.array(picture)[:, :, ::-1])
        os.remove(filename)
        return []

    def _build_sequence_command(self, data):
        """
        Build a Gphoto2 script (string) to call a sequence of gphoto commands.
        data is a list of dictionnary
        dictionnary : iso,aperture,shutterspeed,imageformat
        :param data:
        :return commandcall:
        """
        commandcall = " --shell"
        for param in data:
            commandcall += self._create_set_command(self.isoConfig, param['iso'], prefix="--")
            commandcall += self._create_set_command(self.imageformatConfig, param['imageformat'], prefix="--")
            commandcall += self._create_set_command(self.apertureConfig, param['aperture'], prefix="--")
            commandcall += self._create_set_command(self.shutterspeedConfig, param['shutterspeed'], prefix="--")
            commandcall += "--capture-image --keep --wait-event 1s"
        return commandcall

    def capture_video_cb(self, req):
        rospy.logwarn("Gphoto2 does not support video capture at this moment!")
        return []

    def _copy_picture_from_device_to_standard_directory(self, filename):
        goalQty = rospy.get_param("DownloadQty")
        currentQty = self._get_picture_qty()
        rospy.loginfo('Found ' + str(currentQty) + ' picture(s), will download after ' + str(goalQty))
        if goalQty < 0:
            return "error : qty is negative"
            #Todo find a way to return without string with error... MathGaron
        elif currentQty > goalQty:
            filename = " --filename " + join(self.homePath, filename)
            msg = self._run_gphoto(filename + " -P")
            rospy.loginfo("Picture downloaded to " + filename)
            self.upload_data_to_server()
        else:
            msg = "error"
        return msg

    def _delete_picture_from_device(self):
        self._run_gphoto(" -DR")

    def _get_picture_qty(self):
        msg = self._run_gphoto(' -L')
        try:
            value = int(msg.split('\n')[-2].split()[0][1:])
        except ValueError:
            value = 0
        return value

    def set_camera_cb(self, req):
        """
        set camera's information. Will set data only if it contain something
        """
        rospy.loginfo("Setting camera's Configuration : " + str(req))
        commandCall = ""
        commandCall += self._create_set_command(self.isoConfig, req.iso)
        commandCall += self._create_set_command(self.imageformatConfig, req.imageformat)
        commandCall += self._create_set_command(self.apertureConfig, req.aperture)
        commandCall += self._create_set_command(self.shutterspeedConfig, req.shutterspeed)
        return self._run_gphoto(commandCall)

    def _create_set_command(self, config, string, prefix=" --"):
        command = ""
        if string != "":
            command = prefix + "set-config " + \
                      config + "=" + \
                      self._commandLine_format(string)
        return command

    def _commandLine_format(self, string):
        string = string.replace("(", "\(")
        string = string.replace(")", "\)")
        string = string.replace(" ", "\ ")
        return string.replace(",", "\ ")

    def get_camera_cb(self, req):
        """
        get camera's information. req contain a flag named getAllInformation.
        If true, it will return the whole gphoto's string
        If false, it will return only the value from the string
        """
        rospy.loginfo("Getting camera's Configuration")

        iso = self._run_gphoto(" --get-config " + self.isoConfig)
        imageformat = self._run_gphoto(
            " --get-config " +
            self.imageformatConfig)
        aperture = self._run_gphoto(
            " --get-config " +
            self.apertureConfig)
        shutterspeed = self._run_gphoto(
            " --get-config " +
            self.shutterspeedConfig)

        memorydata = self._run_gphoto(" --storage-info")
        totalspace, freespace, freeimages = self._parse_device_space(memorydata)

        if not req.getAllInformation:
            iso = self._parse_current_value(iso)
            imageformat = self._parse_current_value(imageformat)
            aperture = self._parse_current_value(aperture)
            shutterspeed = self._parse_current_value(shutterspeed)

        return {
            'iso': iso,
            'imageformat': imageformat,
            'aperture': aperture,
            'shutterspeed': shutterspeed,
            'totalSpace': totalspace,
            'freeSpace': freespace,
            'freeImages': freeimages,
        }

    def calibrate_picture_cb(self, req):
        rospy.logwarn("Calibrate Picture is not supported with gphoto driver!")

    def _parse_current_value(self, string):
        lineList = string.split('\n')
        keyword = 'Current:'
        for n in lineList:
            if n.find(keyword) == 0:
                return n[len(keyword):]
        return ''

    def _parse_device_space(self, string):
        """
        Parse a gphoto2 string (gphoto2 --storage-info) to extract important information : total memory capacity
        memory left and quantity of picture that can be still taken.
        :param string:
        :return totalspace, freespace, freeimages:
        """
        lineList = string.split('\n')
        totalspace = ''
        freespace = ''
        freeimages = ''
        totalspacekeyword = 'totalcapacity='
        freespacekeyword = 'free='
        freeimageskeyword = 'freeimages='

        for n in lineList:
            if n.find(totalspacekeyword) == 0:
                totalspace = n[len(totalspacekeyword):]
            if n.find(freespacekeyword) == 0:
                freespace = n[len(freespacekeyword):]
            if n.find(freeimageskeyword) == 0:
                freeimages = n[len(freeimageskeyword):]
        return totalspace, freespace, freeimages

    def _find_camera_thread(self):
        while (not self.terminateThreads):
            if self.camera == '':
                rospy.logdebug('<LFCamera : try getting camera info>')
                with Locker(LOCK_CAMNET_CAPTURE):
                    cameralist = Command.run("{} --auto-detect".format(gphoto2Executable))
                self.camera = self._parse_gphoto_camera_list(cameralist)
            if self.camera == '':
                rospy.logwarn("no camera Found")
                Command.error_manager('No camera found')
            self._set_camera_model(self.camera)
            rospy.sleep(4)

    def _update_camera_parameters(self):
        # Load specific parameter
        if self.cameraModel == 'Nikon DSC D3100 (PTP mode)':
            self.imageformatConfig = "/main/capturesettings/imagequality"
            self.apertureConfig = "/main/other/5007"
            # TODO add more model here, or load xml  MathGaron

    def _parse_gphoto_camera_list(self, string):
        lineList = string.split('\n')
        # first two lines are non important information
        if len(lineList) > 3:
            # The string contain the usb number wich is not important, and we remove all spaces at the end of
            # the camera's name.
            return lineList[2].split(' usb')[0].rstrip()
        else:
            return ''

    def _run_gphoto(self, cmd):
        if self.camera == '':
            return 'no camera'
        cmd = cmd.replace('--', '\n--').replace('\n', '', 1)
        cmd_output = ''
        with Locker(LOCK_CAMNET_CAPTURE):
            for subcmd in cmd.splitlines():
                subcmd = gphoto2Executable + ' ' + subcmd
                cmd_output = Command.run(subcmd, 'GphotoCamServer gphoto command')
        return cmd_output


if __name__ == "__main__":
    rospy.init_node('gphoto_driver')
    server = GPhotoServer()
