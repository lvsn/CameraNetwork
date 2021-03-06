from scripts.Util.extract import *
from scripts.Util.convert import *
import os
import sys

__author__ = 'jbecirovski'


def sort_pix_by_date(path_src):
    """
    Sort name file by timestamp
    :param path_src: string
    :return: dictionary
    """
    dict_by_date = {}
    total_file = len(os.listdir(path_src))
    each_file = 1/float(total_file)
    i = 0
    for file in os.listdir(path_src):
        if file.endswith('.CR2'):
            exif_date = extract_date_from_exif(path_src + file)
            try:
                dict_by_date[exif_date].append(file)
            except KeyError:
                dict_by_date[exif_date] = [file]
        i += each_file
        sys.stdout.write("\r > Indexing by date: {:>6.2%}".format(i))
        sys.stdout.flush()
    print('')
    return dict_by_date


def sort_pix_by_time(path_src, list_pix=0):
    """
    Create a dictionary with HHMMSS key and list of pictures associated
    :param path_src: string
    :return: dictionary
    """
    dict_by_time = {}
    exif_ref = '000000'
    total_file = len(os.listdir(path_src))
    each_file = 1/float(total_file)
    i = 0
    for file in (sorted(os.listdir(path_src)) if not list_pix else sorted(list_pix)):
        if file.endswith('.CR2'):
            exif_time = extract_time_from_exif(path_src + file)
            if cvt_timestamp_to_second(exif_time) >= cvt_timestamp_to_second(exif_ref) + 60:
                dict_by_time[exif_time] = {'data': [file]}
                exif_ref = exif_time
            else:
                dict_by_time[exif_ref]['data'].append(file)
        i += each_file
        sys.stdout.write("\r > Indexing by timestamp: {:>6.2%}".format(i))
        sys.stdout.flush()

    # Add pattern
    for key_time in dict_by_time:
        first_time = cvt_timestamp_to_second(extract_time_from_exif(path_src + dict_by_time[key_time]['data'][0]))
        last_time = cvt_timestamp_to_second(extract_time_from_exif(path_src + dict_by_time[key_time]['data'][-1]))
        if first_time + 20 > last_time and len(dict_by_time[key_time]['data']) == 7:
            dict_by_time[key_time]['pattern'] = [2, 1, 3, 5, 4, 6, 7]
        else:
            dict_by_time[key_time]['pattern'] = range(1, len(dict_by_time[key_time]['data']) + 1)
    print('')

    # try:
    #     for time_key in dict_by_time.keys():
    #         if not len(dict_by_time[time_key]['data']) == 7:
    #             del dict_by_time[time_key]
    # except KeyError:
    #     pass

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
        dict_sorted = sort_pix_by_time(path_src, list_files)
        dict_by_time_inside_date[date] = dict_sorted
    return dict_by_time_inside_date
