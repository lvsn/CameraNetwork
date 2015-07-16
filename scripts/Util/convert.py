import os

import datetime
import time
import pysolar.solar

from constant import *
from extract import *


__author__ = 'jbecirovski'

class DatetimePysolar(datetime.datetime):
    def __init__(self, year, month, day, hour, minute, seconde):
        datetime.datetime.__init__(year, month, day, hour, minute, seconde)

    def timestamp(self):
        return time.mktime(self.timetuple())


def convert_month_StrToInt(str_month):
    """
    :param str_month: str month like Dec/December, etc.
    :return: str integer of month
    """
    month = {'dec': '12', 'nov': '11', 'oct': '10', 'sep': '09', 'aug': '08', 'jui': '07', 'jun': '06', 'may': '05', 'apr': '04', 'mar': '03', 'feb': '02', 'jan': '01'}
    try:
        return month[str_month[:3].lower()]
    except KeyError as msg:
        print('KeyError:', msg, 'should be', month.keys())
    except TypeError as msg:
        print('TypeError:', msg, 'should be string.')


def convert_timestamp_second(str_timestamp):
    """
    Convert str format HHMMSS to second
    :param str_timestamp: string
    :return: int
    """
    return int(str_timestamp[:2])*3600 + int(str_timestamp[2:4])*60 + int(str_timestamp[4:6])


def convert_name_timestamp(src_path, file):
    """
    :param src_path: str
    """
    path = src_path + file
    exif = extract_exif(path)
    exif_date = extract_date_from_exif(path, exif)
    exif_time = extract_time_from_exif(path, exif)
    new_pic_name = '{}_{}_{}.CR2'.format(exif_date[2:], exif_time, 'n')
    os.rename(path, src_path + new_pic_name)