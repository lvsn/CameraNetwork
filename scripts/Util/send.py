import os

__author__ = 'jbecirovski'


def send_folder_to(path_src, path_dst):
    """
    Sending folder with pictures to another place with rsync
    :param path_src: string
    :param path_dst: string
    """
    print('Folder sending ...')
    os.system('rsync -avrz --progress --remove-source-files {} {}'.format(path_src, path_dst))