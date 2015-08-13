__author__ = 'jbecirovski'

import time, datetime

import pysolar

from constant import *


class DatetimePysolar(datetime.datetime):
    """ PySolar need python3.4 and this class allows to be used with python2.7 """
    def __init__(self, year, month, day, hour, minute, second):
        datetime.datetime.__init__(year, month, day, hour, minute, second)

    def timestamp(self):
        return time.mktime(self.timetuple())


def is_it_day():
    """
    _is_day() -> bool
    """
    date_time = datetime.datetime.now()
    date_time = DatetimePysolar(date_time.year, date_time.month, date_time.day,
                                date_time.hour, date_time.minute, date_time.second)
    solar_altitude = pysolar.solar.get_altitude(LATITUDE_DEG, LONGITUDE_DEG, date_time)
    rospy.logdebug('Solar Altitude: {}'.format(solar_altitude))
    return solar_altitude >= 0.0
