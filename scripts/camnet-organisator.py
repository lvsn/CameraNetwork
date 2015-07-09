import os, subprocess, sys

import exifread

__author__ = 'jbecirovski'


def regroup_all_raw_data(path_src):
    """
    Regroup all raw data since path_src into path_src/processed/ folder
    :param path_src: str
    """
    try:
        os.mkdir(os.path.join(path_src, 'processed'))
    except OSError:
        sys.stdout.write('WARN: processed folder already exists\n')

    if not os.path.exists(os.path.join(path_src, 'raw_data')):
        sys.stdout.write('*** ERROR: Make sure pictures are inside raw_data folder here %s ***\n' % os.getcwd())
        os.mkdir(os.path.join(os.getcwd(), 'raw_data'))
        exit(0)

    output_cmd = subprocess.Popen('find raw_data/ -name *.CR2 -type f', shell=True, stdout=subprocess.PIPE).communicate()
    pictures_path = map(os.path.abspath, output_cmd[0].splitlines())

    total_pics = len(pictures_path)
    pics = 1

    for pic_path in pictures_path:
        n = 1
        new_pic_name = rename_pictures(pic_path)
        while os.path.exists(os.path.join(path_src, 'processed', new_pic_name.replace('X', str(n)))):
            n += 1
        os.rename(pic_path, os.path.join(path_src, 'processed', new_pic_name.replace('X', str(n))))
        sys.stdout.write('({}/{}) MV {} to {}\n'.format(pics, total_pics, pic_path, os.path.join(path_src, 'processed', new_pic_name.replace('X', str(n)))))
        pics += 1

def extract_exif(path_src):
    """
    Extract exif from raw data:
    1 - Extract raw exif from picture
    2 - Format date and time in dictionary
    :param path_src: str - file source path
    :return: dict - exif information
    """
    exif_raw_info = exifread.process_file(open(path_src))
    if exif_raw_info is None:
        raise TypeError('File is not picture')

    exif_info = {}
    exif_info['date'] = ''.join(str(exif_raw_info['Image DateTime']).split()[0].split(':'))
    exif_info['time'] = ''.join(str(exif_raw_info['Image DateTime']).split()[1].split(':'))

    return exif_info


def rename_pictures(path_src, exif_info=None):
    """
    Rename picture with exif date and time:
    1 - Extract date and time information from picture
    2 - Format new name with these information
    :param path_src: srt - file source path
    :return: str - 'YYMMDD_HHMMSS_'
    """
    if exif_info is None:
        exif_info = extract_exif(path_src)
    return '{}_{}_X.CR2'.format(exif_info['date'][2:], exif_info['time'])

def create_dict_with_files(src_path):
    """
    Create dictionary thanks to file names like YYMMDD_HHMMSS_n.CR2
    :param src_path: str path
    :return: dictionary [date][time][file for file in files]
    """
    sys.stdout.write('> Creating dictionary with file names\n')
    files = subprocess.check_output(['ls', src_path]).splitlines()
    files = [file for file in files if '.CR2' in file]
    dict_date = {}

    sys.stdout.write('>> Register dates\n')
    for file in files:
        file_splited = file.split('_')
        date = '20' + file_splited[0]
        if date in dict_date.keys():
            dict_date[date].append(file)
        else:
            dict_date[date] = []
            dict_date[date].append(file)

    sys.stdout.write('>> Register times per dates\n')
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

    sys.stdout.write('> Done\n')
    return dict_date

def convert_timestamp_second(str_timestamp):
    """
    Convert str format HHMMSS to second
    :param str_timestamp: string
    :return: int
    """
    return int(str_timestamp[:2])*3600 + int(str_timestamp[2:4])*60 + int(str_timestamp[4:6])

def create_folder_from_dictionary(path_dst, dict_tree):
    """
    Create folder with date of the picture to destination folder
    :param path_dst: string
    :param dict_tree: dictionary
    """
    for key in dict_tree.keys():
        try:
            os.mkdir(os.path.join(path_dst, key))
            sys.stdout.write('> MKDIR {}\n'.format(os.path.join(path_dst, key)))
        except:
            pass
        if isinstance(dict_tree[key], dict):
            try:
                dict_tree[key]['data']
            except KeyError:
                create_folder_from_dictionary(os.path.join(path_dst, key), dict_tree[key])

def move_with_dictionary(src_path, dst_path, tree_files):
    """
    :param src_path: str source path where pictures are
    :param dst_path: str destination path where pictures should move
    :param tree_files: dict with tree of directories and files
    """
    sys.stdout.write('> Pictures moving ...\n')
    for date in tree_files.keys():
        sys.stdout.write('>> Date: {}\n'.format(date))
        for time in tree_files[date].keys():
            sys.stdout.write('>>> Time: {}\n'.format(time))
            for i, file in enumerate(tree_files[date][time]):
                new_file = '_'.join(part if '.CR2' not in part else '{}.CR2'.format(i + 1) for part in file.split('_'))
                sys.stdout.write(' >>>> file {} to {}\n'.format(file, os.path.join(dst_path, date, time, new_file)))
                os.rename(os.path.join(src_path, file), os.path.join(dst_path, date, time, new_file))
    sys.stdout.write('> Moving done\n')

if __name__ == '__main__':
    """
    Organize raw data with specific name in specific folders.
    1 - Get all path of raw pictures
    2 - Create folder tree
    3 - Move of raw pictures to its specific folder
    """
    sys.stdout.write('*** CAMNET ORGANIZER - Started ***\n')
    sys.stdout.write('Current path: %s\n' % os.getcwd())
    regroup_all_raw_data(os.getcwd())
    folder_tree = create_dict_with_files(os.path.join(os.getcwd(), 'processed'))
    create_folder_from_dictionary(os.getcwd(), folder_tree)
    move_with_dictionary(os.path.join(os.getcwd(), 'processed'), os.getcwd(), folder_tree)
    sys.stdout.write('*** CAMNET ORGANIZER - Finished ***\n')

