import subprocess

__author__ = 'jbecirovski'


def stop_AEB_capture():
    print('AEB stopping begin...')
    str_sub = subprocess.check_output('/bin/ps aux | /bin/grep python | /bin/grep launch_aeb.py', shell=True).splitlines()
    if len(str_sub) > 1:
        pid = str_sub[0].split()[1]
        subprocess.call('kill {}'.format(pid), shell=True)
        print('AEB finished.')
    else:
        print('AEB not running.')
