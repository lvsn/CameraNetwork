import os
from scripts.Util.extract import *
from scripts.Util.convert import *
from scripts.Util.sort import *
import sys

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
        except FileExistsError:
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
        except NotADirectoryError as e:
            print(e)


def organize_folder(path_src):
    """
    Organize folder like : path_src/YYYYMMDD/HHMMSS/YYMMDD_HHMMSS_n.CR2
    :param path_src: string
    """
    folder_process = 'processed'

    print('Folder organization begin')
    print('> Folder indexing ...')
    dict_folder = sort_pix_by_time_inside_date(path_src)

    try:
        os.mkdir(path_src + folder_process)
    except FileExistsError:
        pass
    print(dict_folder)
    print('> Folder tree creating ...')
    create_folder_from_dictionary(path_src + folder_process + '/', dict_folder)
    print(' > Done                         ')
    print('> Pictures renaming ...')
    move_rename_pix(path_src, path_src + folder_process + '/', dict_folder)
    print(' > Done                         ')
    print('Folder organization finished')
