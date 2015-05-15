from scripts.Util.extract import *
from scripts.Util.convert import *
import os
__author__ = 'jbecirovski'


def sort_pix_by_date(path_src):
    """
    Sort name file by timestamp
    :param path_src: string
    :return: dictionary
    """
    dict_by_date = {}
    for file in os.listdir(path_src):
        if file.endswith('.CR2'):
            exif_date = extract_date_from_exif(path_src + file)
            try:
                dict_by_date[exif_date].append(file)
            except KeyError:
                dict_by_date[exif_date] = [file]
    return dict_by_date


def sort_pix_by_time(path_src, list_pix=0):
    """
    Create a dictionary with HHMMSS key and list of pictures associated
    :param path_src: string
    :return: dictionary
    """
    dict_by_time = {}
    exif_ref = '000000'
    for file in (sorted(os.listdir(path_src)) if not list_pix else sorted(list_pix)):
        if file.endswith('.CR2'):
            exif_time = extract_time_from_exif(path_src + file)
            if convert_timestamp_second(exif_time) >= convert_timestamp_second(exif_ref) + 150 or len(dict_by_time[exif_ref]) == 7:
                dict_by_time[exif_time] = [file]
                exif_ref = exif_time
            else:
                dict_by_time[exif_ref].append(file)
    return dict_by_time


def sort_pix_by_time_inside_date(path_src):
    """
    Create a dictionary with Date key and dictionary with timestamp key associated and list of
    pictures associated.
    :param path_src: string
    :return: dictionary
    """
    dict_by_time_inside_date = {}
    dict_by_time_inside_date.update(sort_pix_by_date(path_src))
    for date in dict_by_time_inside_date.keys():
        list_files = dict_by_time_inside_date[date]
        dict_by_time_inside_date[date] = sort_pix_by_time(path_src, list_files)
    return dict_by_time_inside_date