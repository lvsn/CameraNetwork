
import os
from scripts.Util.extract import *
from scripts.Util.convert import *
from scripts.Util.sort import *
import sys, subprocess



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


def move_rename_pix(path_src, path_dst, dict_pix, dict_exif=0):
    """
    Rename picture to YYMMDD_HHMMSS_n.CR2
    :param path_src: string
    :param path_dst: string
    :param dict_pix: dictionary
    :param dict_exif: dictionary
    """
    for key in sorted(dict_pix.keys()):
        try:
            if isinstance(dict_pix[key], dict) and 'data' not in dict_pix[key].keys():
                move_rename_pix(path_src, path_dst + key + '/', dict_pix[key], dict_exif)
            else:
                for i, file in enumerate(sorted(dict_pix[key]['data'])):
                    exif_time = extract_time_from_exif(path_src + file, dict_exif)
                    exif_date = extract_date_from_exif(path_src + file, dict_exif)
                    new_pic_name = '{}_{}_{}.CR2'.format(exif_date[2:], exif_time, dict_pix[key]['pattern'][i])
                    os.rename(path_src + file, path_dst + key + '/' + new_pic_name)
                    sys.stdout.write('\r > MV {} to {}'.format(file, path_dst + key + '/' + new_pic_name))
                    sys.stdout.flush()
        except:
            raise AssertionError


def rename_with_timestamp(path_src):
    """
    Rename all raw img inside path folder with timestamp from their exif
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
            file_time = convert_timestamp_second(file_splited[1])

            if file_time >= convert_timestamp_second(time_ref) + 60:
                dict_date[date][file_splited[1]] = []
                dict_date[date][file_splited[1]].append(file)
                time_ref = file_splited[1]
            else:
                dict_date[date][time_ref].append(file)

    for date in dict_date.keys():
        for time in dict_date[date].keys():
            if not len(dict_date[date][time]) == 7:
                try:
                    del dict_date[date][time]
                except KeyError:
                    pass
    print('> Done')
    return dict_date

def move_with_dictionary(src_path, dst_path, tree_files):
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

    try:
        os.mkdir(path_src + folder_process)
    except:
        pass

    print('> Folder tree creating ...')
    create_folder_from_dictionary(path_src + folder_process + '/', dict_date)
    print(' > Done                         ')
    move_with_dictionary(path_src, path_src + folder_process + '/', dict_date)
    print('Folder organization finished')