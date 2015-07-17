import os

from convert import *

__author__ = 'jbecirovski'


def extract_exif(path_src):
    """
    Push exif information of picture inside dictionary
    :param path_src: string
    :return: dictionary
    """
    dict_exif = {}
    info_exif = os.popen('dcraw -i -v {}'.format(path_src)).read().splitlines()

    try:
        if not info_exif:
            raise AssertionError
    except AssertionError:
        print('Current path is not available :', path_src)
        return None

    for info in info_exif:
        if ': ' in info:
            dict_exif[info.split(': ')[0]] = info.split(': ')[1].strip()

    return dict_exif


def extract_date_from_exif(path_src, dict_exif=0):
    """
    Extract date format YYYYMMDD from exif
    :param path_src: string
    :param dict_exif: dictionary
    :return: string
    """
    if not dict_exif:
        str_timestamp = extract_exif(path_src)['Timestamp'].split()
    else:
        str_timestamp = dict_exif['Timestamp'].split()

    year = str_timestamp[-1]
    month = cvt_month_str_to_int(str_timestamp[1])
    day = str_timestamp[2]

    return '{:0>4}{:0>2}{:0>2}'.format(year, month, day)


def extract_time_from_exif(path_src, dict_exif=0):
    """
    Extract time format HHMMSS from exif
    :param path_src: string
    :param dict_exif: dictionary
    :return: string
    """
    if not dict_exif:
        str_timestamp = extract_exif(path_src)['Timestamp'].split()
    else:
        str_timestamp = dict_exif['Timestamp'].split()

    hour = str_timestamp[3].split(':')[0]
    minute = str_timestamp[3].split(':')[1]
    second = str_timestamp[3].split(':')[2]

    return '{:0>2}{:0>2}{:0>2}'.format(hour, minute, second)