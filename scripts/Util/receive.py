import os

__author__ = 'jbecirovski'


def get_raw_from_camera(path):
    """
    Get all raw pictures from camera to path
    :param path: string
    """
    os.chdir(path)
    os.system('gphoto2 --get-all-raw-data')