import os

__author__ = 'jbecirovski'

try:
    input_lock_dir = os.environ['LOCK_DIR']
    if not os.path.isdir(input_lock_dir):
        raise KeyError
except KeyError:
    input_lock_dir = '/run/lock'

LOCK_DIR = input_lock_dir
LOCK_DEFAULT_PROCESS = 'gphoto2'
LOCK_WHITE_LIST = {'gphoto2'}
DL_DATA_SERIE_SIZE = 10