#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu June 05 09:17:30 2014

@author: Mathieu Garon
"""
import os
import time
import io
from datetime import datetime

import roslib
roslib.load_manifest('camera_drivers')
import rospy
from sensor_msgs.msg import Image
import std_srvs.srv
import cv2
from cv_bridge import CvBridge, CvBridgeError
import picamera
import wiringpi2 as gpio
import numpy as np
import camera_driver as cd

import picamParameterHandler as pph
from camera_network_msgs.srv import *


class picam_server(cd.camera_driver):
    """Handles the requests by ROS to the picam."""
    def __init__(self):
        try:
            self.picam = picamera.PiCamera()
        except:
            rospy.logfatal("Check if the Picam is free or installed")

        self.bridge = CvBridge()
        self.id_gen = self._id_generator()
        self.camParam = pph.PicameraParameterHandler()
        try:
            self.homePath = os.environ["CAMNET_OUTPUT_DIR"]
        except KeyError:
            self.homePath = os.path.expanduser("~/Pictures")
        self.tmpPath = self.homePath + '/tmp'

        # Initialisation
        self._init_picamera()
        self._init_picamera_led()

        # ROS functions
        super(picam_server, self).__init__()
        rospy.Service('stream_video', Uint32, self.stream_video_cb)
        rospy.Service(
            'calibrate_video',
            std_srvs.srv.Empty,
            self.calibrate_video_cb)
        self.image_publisher = rospy.Publisher("/preview", Image)

        rospy.loginfo("Camera Ready")
        self._flash_led(nflash=4)
        rospy.spin()

    def __del__(self):
        self._flash_led(nflash=3)
        self.picam.close()

    def capture_image_cb(self, req):
        rospy.loginfo("Taking Picture")

        if not os.path.exists(self.tmpPath):
            os.makedirs(self.tmpPath)

        pictureFileName = "{0}/unloaded_{1}.{2}".format(
            self.tmpPath,
            self.id_gen.next(),
            self.camParam.get_format(),
        )
        self._set_timer(req.timer)
        self.picam.capture(pictureFileName, format=self.camParam.get_format())
        self._flash_led(nflash=2)
        return 'Image saved as ' + pictureFileName

    def stream_video_cb(self, req):
        stream = io.BytesIO()
        rospy.loginfo("Start Video streaming with " +
                      str(req.integer) +
                      " frames.")
        gpio.digitalWrite(self.led, True)
        for i in range(req.integer):
            stream.flush()
            stream.seek(0)
            self.picam.capture(stream, format='jpeg', resize=(320, 240))
            data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            image = cv2.imdecode(data, 1)
            try:
                self.image_publisher.publish(
                    self.bridge.cv2_to_imgmsg(
                        image,
                        "bgr8"))
            except CvBridgeError as e:
                rospy.logwarn("stream_video_cb : " + e)
        gpio.digitalWrite(self.led, False)
        return {}

    def capture_video_cb(self, req):
        rospy.loginfo("Capturing Video")
        self._mkdir(self.tmpPath)
        videoFileName = self.tmpPath + '/unloaded_' + \
            self.id_gen.next() + '.h264'
        self._record_video(videoFileName, req.integer)
        return {}

    def calibrate_picture_cb(self, req):
        rospy.loginfo("Picture Calibration")
        self.picam.shutter_speed = 0
        self.picam.awb_mode = 'auto'
        rospy.sleep(1)
        new_awb = self.picam.awb_gains
        new_exp = self.picam.exposure_speed
        self.picam.awb_mode = 'off'
        self.picam.awb_gains = new_awb
        self.picam.shutter_speed = new_exp
        self._flash_led(nflash=6)
        return {}

    def calibrate_video_cb(self, req):
        rospy.loginfo("Video Calibration")
        self._mkdir(self.tmpPath)
        videoFileName = self.tmpPath + '/Calibration.h264'
        saturationFlag = False
        while(True):
            self._record_video(videoFileName, 0.2)
            video = cv2.VideoCapture(videoFileName)
            pixelVal = self._get_center_pixel_value(video)
            if(pixelVal > 254):
                self.picam.brightness = self.picam.brightness - 2
                saturationFlag = True
            else:
                # this condition mean that we are at the edge of saturation
                if(saturationFlag):
                    break
                self.picam.brightness = self.picam.brightness + 2
                saturationFlag = False

        try:
            os.remove(videoFileName)
        except:
            rospy.logwarn("error deleting calibration file")
        self._flash_led(nflash=6)
        return {}

    def _get_center_pixel_value(self, video):
        pixelMean = 0
        pixels = 0
        while (video.isOpened()):
            ret, frame = video.read()
            if not ret:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            pixels += 5
            pixelMean += gray[height / 2, width / 2]
            pixelMean += gray[(height / 2) + 1, width / 2]
            pixelMean += gray[(height / 2) - 1, width / 2]
            pixelMean += gray[height / 2, (width / 2) + 1]
            pixelMean += gray[height / 2, (width / 2) - 1]
        pixelMean = pixelMean / pixels
        rospy.loginfo(
            "Mean value : " +
            str(pixelMean) +
            " pixels : " +
            str(pixels))
        video.release()
        return pixelMean

    def load_camera_cb(self, req):
        # reset generator
        self.id_gen = self._id_generator()

        # to make sure it create the right path
        loadPath = os.path.join(
            self.homePath,
            self._filename_format(req.path, 0, 'dummy'),
        )
        if loadPath.find('..') != -1:
            rospy.logwarn("Use of .. is prohibited")
            return "error"
        directory = os.path.dirname(loadPath)
        rospy.loginfo("Loading Picture to folder " + directory)
        if not os.path.exists(directory):
            os.makedirs(directory)
        count = 0
        for pictureFile in os.listdir(self.tmpPath):
            fileFormat = pictureFile.split('.')[-1]
            os.rename(
                os.path.join(self.tmpPath, pictureFile),
                os.path.join(self.homePath, self._filename_format(
                    req.path,
                    count,
                    fileFormat)
                )
            )
            count += 1
        return "Transfered " + str(count) + " files."

    def set_camera_cb(self, req):
        rospy.loginfo("Setting camera's Configuration to " + str(req))
        if(req.iso != ""):
            self.picam.ISO = int(float(req.iso))
        if(req.imageformat != ""):
            self.camParam.set_format(req.imageformat)
        if(req.aperture != ""):
            self.picam.brightness = int(float(req.aperture))
        if(req.shutterspeed != ""):
            self.picam.shutter_speed = int(float(req.shutterspeed))

        return "Picam set"

    def get_camera_cb(self, req):
        rospy.loginfo("Getting camera's Configuration")
        iso = str(self.picam.ISO)
        imageformat = str(self.camParam.get_format())
        aperture = str(self.picam.brightness)
        shutterspeed = str(self.picam.shutter_speed)

        if req.getAllInformation:
            iso = "current ISO : " + iso + \
                "\n Choice : 100\nChoice : 200\nChoice : 320\nChoice : 400\nChoice : 500\nChoice : 640\nChoice : 800\n"
            imageformat = "current Image format : " + imageformat + \
                "\nChoice : jpeg\nChoice : png\nChoice : gif\nChoice : bmp\nChoice : yuv\nChoice : rgb\nChoice : rgba\nChoice : bgr\nChoice : bgra\n"
            shutterspeed = "current Shutterspeed : " + shutterspeed + \
                "\nChoice : 0(auto)\nChoice : (int)usec\n"
            aperture = "current aperture : " + \
                aperture + "\nChoice : 0 - 100\n"

        return {
            'iso': iso,
            'imageformat': imageformat,
            'aperture': aperture,
            'shutterspeed': shutterspeed}

    def _filename_format(self, string_, pictureId=0, pictureFormat='jpeg'):
        """
        Produces the name of the image. string_ comes from
        camera_controller's camera_handler.py:_generatePictureName
        """
        string_ = string_.replace('%C', pictureFormat)
        string_ = string_.replace('%n', str(pictureId))
        return datetime.now().strftime(string_)

    def _id_generator(self):
        for i in range(10000000):
            yield str(i)

    def _mkdir(self, dirPath):
        if not os.path.exists(dirPath):
            os.makedirs(dirPath)

    def _init_picamera(self):
        self.picam.exposure_mode = 'fixedfps'
        self.picam.awb_mode = 'off'
        self.picam.awb_gains = 1.4
        self.picam.resolution = (1296, 972)
        self.picam.framerate = 40
        self.camParam.set_camera_parameters()

    def _init_picamera_led(self):
        self.led = 5
        os.system("gpio export " + str(self.led) + " out")
        if gpio.wiringPiSetupSys() != 0:
            rospy.logfatal("Unable to setup gpio")
        gpio.digitalWrite(self.led, False)

    def _flash_led(self, nflash=1, delay=0.1):
        # nflash is the number of blink the led will make
        for n in range(nflash):
            gpio.digitalWrite(self.led, True)
            rospy.sleep(delay)
            gpio.digitalWrite(self.led, False)
            rospy.sleep(delay)

    def _set_timer(self, delay_second):
        """
        This function set a timer in seconds, and toggle the led every 300 ms. Once the timer have 2 seconds left, the
        led flash faster so the user know that the picture will be taken.
        """
        startTime = rospy.get_rostime().secs
        endTimerTime = rospy.Time(startTime + delay_second)

        gpioState = False
        blinkTime = rospy.get_rostime()
        blinkDelay_nsec = 300000000
        endBlinkTime = rospy.Time(
            blinkTime.secs,
            (blinkTime.nsecs + blinkDelay_nsec))

        while rospy.get_rostime().secs < endTimerTime.secs:
            if rospy.get_rostime() > endBlinkTime:
                gpioState = not gpioState
                if endTimerTime.secs - rospy.get_rostime().secs < 2:
                    blinkDelay_nsec = 100000000
                blinkTime = rospy.get_rostime()
                endBlinkTime = rospy.Time(
                    blinkTime.secs,
                    (blinkTime.nsecs + blinkDelay_nsec))
            gpio.digitalWrite(self.led, gpioState)
        return

    def _record_video(self, filename, delay):
        gpio.digitalWrite(self.led, True)
        self.picam.start_recording(filename)
        rospy.sleep(delay)
        self.picam.stop_recording()
        gpio.digitalWrite(self.led, False)


if __name__ == "__main__":
    rospy.init_node('picam')
    server = picam_server()
