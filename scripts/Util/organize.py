import os
from scripts.Util.extract import *
from scripts.Util.convert import *
from scripts.Util.sort import *

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
            print('\_ MKDIR {}{}'.format(path_dst, key))
        except FileExistsError:
            pass
        if isinstance(dict_tree[key], dict):
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
            if isinstance(dict_pix[key], dict):
                move_rename_pix(path_src, path_dst + key + '/', dict_pix[key], dict_exif)
            else:
                for i, file in enumerate(sorted(dict_pix[key])):
                    exif_time = extract_time_from_exif(path_src + file, dict_exif)
                    exif_date = extract_date_from_exif(path_src + file, dict_exif)
                    new_pic_name = '{}_{}_{}.CR2'.format(exif_date[2:], exif_time, i + 1)
                    os.rename(path_src + file, path_dst + key + '/' + new_pic_name)
                    print('\_ MV {} to {}'.format(file, path_dst + key + '/' + new_pic_name))
        except NotADirectoryError as e:
            print(e)


def organize_folder(path_src):
    """
    Organize folder like : path_src/YYYYMMDD/HHMMSS/YYMMDD_HHMMSS_n.CR2
    :param path_src: string
    """
    print('Folder organization begin')
    print('> Folder indexing ...')
    dict_folder = sort_pix_by_time_inside_date(path_src)
    print('> Folder tree creating ...')
    create_folder_from_dictionary(path_src, dict_folder)
    print('> Pictures renaming ...')
    move_rename_pix(path_src, path_src, dict_folder)
    print('Folder organization finished')