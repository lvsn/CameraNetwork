
import os, sys, subprocess

from extract import *
from convert import *
from sort import *

__author__ = 'jbecirovski'


def create_folder_from_dictionary(path_dst, dict_tree):
    """
    Create folder with date of the picture to destination folder
    :param path_dst: string
    :param dict_tree: dictionary
    """
    for key in dict_tree.keys():
        try:
            os.mkdir(path_dst + key)
            sys.stdout.write('\r > MKDIR {}{}'.format(path_dst, key))
            sys.stdout.flush()
        except:
            pass
        if isinstance(dict_tree[key], dict):
            try:
                dict_tree[key]['data']
            except KeyError:
                create_folder_from_dictionary(path_dst + key + '/', dict_tree[key])


def rename_with_timestamp(path_src):
    """
    Rename all raw img inside path folder with timestamp from their exif like YYMMDD_HHMMSS_n.CR2
    :param path_src: str path
    """
    print('> Pictures renaming with timestamp ...')
    files = subprocess.check_output(['ls', path_src]).splitlines()
    files = [file for file in files if '.CR2' in file]
    new_files = []
    for j, file in enumerate(files):
        exif_time = extract_time_from_exif(path_src + file)
        exif_date = extract_date_from_exif(path_src + file)
        i = 1
        new_pic_name = '{}_{}_{}.CR2'.format(exif_date[2:], exif_time, i)
        while 1:
            if new_pic_name not in new_files:
                print(' >> {}/{} RENAME {} to {}'.format(j+1, len(files), file, new_pic_name))
                new_files.append(new_pic_name)
                os.rename(path_src + file, path_src + new_pic_name)
                break
            else:
                i += 1
                new_pic_name = '{}_{}_{}.CR2'.format(exif_date[2:], exif_time, i)
    print('  > renaming Done')

def create_dict_with_files(src_path):
    """
    Create dictionary thanks to file names like YYMMDD_HHMMSS_n.CR2
    :param src_path: str path
    :return: dictionary [date][time][file for file in files]
    """
    print('> Creating dictionary with file names')
    files = subprocess.check_output(['ls', src_path]).splitlines()
    files = [file for file in files if '.CR2' in file]
    dict_date = {}

    print('>> Register dates')
    for file in files:
        file_splited = file.split('_')
        date = '20' + file_splited[0]
        if date in dict_date.keys():
            dict_date[date].append(file)
        else:
            dict_date[date] = []
            dict_date[date].append(file)

    print('>> Register times per dates')
    for date in dict_date.keys():
        file_list = dict_date[date]
        dict_date[date] = {}
        time_ref = '000000'

        for file in file_list:
            file_splited = file.split('_')
            file_time = cvt_timestamp_to_second(file_splited[1])

            if file_time >= cvt_timestamp_to_second(time_ref) + 60:
                dict_date[date][file_splited[1]] = []
                dict_date[date][file_splited[1]].append(file)
                time_ref = file_splited[1]
            else:
                if time_ref in dict_date[date].keys():
                    dict_date[date][time_ref].append(file)
                else:
                    dict_date[date][time_ref] = [file]

    for date in dict_date.keys():
        for time in dict_date[date].keys():
            if not len(dict_date[date][time]) == 7:
                try:
                    del dict_date[date][time]
                except KeyError:
                    pass
    print('>> Done')
    return dict_date

def organize_dictionary_day_night(dict_date_time):
    print('> Converting datetime dictionary by day and night ...')
    dict_day_night = {}
    for k_date in dict_date_time.keys():
        for k_time in k_date.keys():
            date_time = DatetimePysolar(int(k_date[:4]),
                                        int(k_date[4:6]),
                                        int(k_date[6:]),
                                        int(k_time[:2]),
                                        int(k_time[2:4]),
                                        int(k_date[4:]))
            solar_altitude = pysolar.solar.get_altitude(LATITUDE_DEG, LONGITUDE_DEG, date_time)
            if solar_altitude > 0:
                dict_day_night[k_date] = {k_time: k_date[k_time]}
            else:
                dict_day_night[k_date + '_N'] = {k_time: k_date[k_time]}
    print('>> Done')
    return dict_day_night

def move_with_dictionary(src_path, dst_path, tree_files):
    """
    :param src_path: str source path where pictures are
    :param dst_path: str destination path where pictures should move
    :param tree_files: dict with tree of directories and files
    """
    print('> Pictures moving ...')
    for date in tree_files.keys():
        print('>> Date: {}'.format(date))
        for time in tree_files[date].keys():
            print('>>> Time: {}'.format(time))
            for i, file in enumerate(tree_files[date][time]):
                new_file = '_'.join(part if '.CR2' not in part else '{}.CR2'.format(i + 1) for part in file.split('_'))
                print(' >>>> file {} to {}'.format(file, dst_path + date + '/' + time + '/' + new_file))
                os.rename(src_path + file, dst_path + date + '/' + time + '/' + new_file)
    print('> Moving done')


def organize_folder(path_src):
    """
    Organize folder like : path_src/YYYYMMDD/HHMMSS/YYMMDD_HHMMSS_n.CR2
    :param path_src: string
    """
    folder_process = 'processed'

    print('Folder organization begin')
    rename_with_timestamp(path_src)
    print('> Folder indexing ...')
    dict_date = create_dict_with_files(path_src)
    dict_date = organize_dictionary_day_night(dict_date)

    try:
        os.mkdir(path_src + folder_process)
    except:
        pass

    print('> Folder tree creating ...')
    create_folder_from_dictionary(path_src + folder_process + '/', dict_date)
    print(' > Done                         ')
    move_with_dictionary(path_src, path_src + folder_process + '/', dict_date)
    print('Folder organization finished')