__author__ = 'jbecirovski'
import os
import sys


try:
    import rospy
except ImportError:
    print('Could not import rospy.')
    pass


if 'rospy' in sys.modules:
    try:
        LOCK_DIR = os.environ['LOCK_DIR']
        if not os.path.isdir(LOCK_DIR):
            raise KeyError
    except KeyError:
        LOCK_DIR = '/run/lock'
        rospy.logwarn('*** WARN: Bad lock directory replaced by {}'.format(LOCK_DIR))

    try:
        HOME_DIR = os.environ['HOME']
        if not os.path.isdir(HOME_DIR):
            raise KeyError
    except KeyError:
        HOME_DIR = '/home/pi/'
        rospy.logwarn('*** WARN: Bad home directory replaced by {}'.format(HOME_DIR))

    try:
        CAMNET_OUTPUT_DIR = os.environ['CAMNET_OUTPUT_DIR']
        if not os.path.isdir(CAMNET_OUTPUT_DIR):
            raise KeyError
    except KeyError:
        CAMNET_OUTPUT_DIR = os.path.join(HOME_DIR, 'camera-output')
        rospy.logwarn('*** WARN: Bad camera output directory replaced by {}'.format(CAMNET_OUTPUT_DIR))
        try:
            os.mkdir(CAMNET_OUTPUT_DIR)
        except OSError as msg:
            rospy.logwarn('*** WARN: %s' % msg)

    try:
        CAMNET_SERVER_DATA_DIR = os.environ['CAMNET_SERVER_DATA_DIR']
    except KeyError:
        CAMNET_SERVER_DATA_DIR = ('JUBEC7@victoria.gel.ulaval.ca:'
                                  '/home-local/yahog.extra.nobkp/www/pictures/unprocessed/raw_data/')
        rospy.logwarn('*** WARN: Bad server data directory replaced by {}'.format(CAMNET_SERVER_DATA_DIR))

    try:
        CAMNET_OUTPUT_DIR = os.environ['CAMNET_OUTPUT_DIR']
        if not os.path.isdir(CAMNET_OUTPUT_DIR):
            raise KeyError
    except KeyError:
        CAMNET_OUTPUT_DIR = os.path.join(HOME_DIR, 'camera-output')
        rospy.logwarn('*** WARN: Bad camera output directory replaced by {}'.format(CAMNET_OUTPUT_DIR))
        try:
            os.mkdir(CAMNET_OUTPUT_DIR)
        except OSError as msg:
            rospy.logwarn('*** WARN: %s' % msg)

    try:
        CAMNET_SERVER_DATA_DIR = os.environ['CAMNET_SERVER_DATA_DIR']
    except KeyError:
        CAMNET_SERVER_DATA_DIR = ('JUBEC7@victoria.gel.ulaval.ca:'
                                  '/home-local/yahog.extra.nobkp/www/pictures/unprocessed/raw_data/')
        rospy.logwarn('*** WARN: Bad server data directory replaced by {}'.format(CAMNET_SERVER_DATA_DIR))

    try:
        CAMERA_NAME = os.environ['CAMERA_NAME']
    except KeyError:
        CAMERA_NAME = 'ObjCam'
        rospy.logwarn('*** WARN: Bad camera name replaced by {}'.format(CAMERA_NAME))

    try:
        ROS_IP = os.environ['ROS_IP']
    except KeyError:
        ROS_IP = '127.0.0.1'
        rospy.logwarn('*** WARN: Bad ROS ip replaced by {}'.format(ROS_IP))

LOCK_DEFAULT_PROCESS = 'gphoto2'
LOCK_WHITE_LIST = {'gphoto2'}
LOCK_CAMNET_CAPTURE = 'camnet_capture'
DL_DATA_SERIE_SIZE = 10
GPIO_POWER_CONTROL = 5
LATITUDE_DEG, LONGITUDE_DEG = 46.779586, -71.275474
