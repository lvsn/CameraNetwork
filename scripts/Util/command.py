import os, sys, envoy, time

import rospy

from scripts.Util.constant import *

__author__ = 'jbecirovski'


class Locker(object):
    """ Control lock process by creating and deleting empty file inside tmpfs folder (LOCK_DIR) """
    @staticmethod
    def lock(process_name=LOCK_DEFAULT_PROCESS):
        try:
            open(os.path.join(LOCK_DIR, process_name), 'a').close()
        except:
            rospy.logerr('*** ERROR: Can\'t lock {}: {}'.format(process_name, sys.exc_info()[0]))

    @staticmethod
    def unlock(process_name=LOCK_DEFAULT_PROCESS):
        try:
            os.remove(os.path.join(LOCK_DIR, process_name))
        except:
            rospy.logerr('*** ERROR: Can\'t unlock {}: {}'.format(process_name, sys.exc_info()[0]))

    @staticmethod
    def is_lock(process_name=LOCK_DEFAULT_PROCESS):
        if os.path.exists(os.path.join(LOCK_DIR, process_name)):
            return True
        else:
            return False

class Command(object):
    @staticmethod
    def run(cmd, msg='unspecified category'):
        try:
            process_name = cmd.split()[0]
            if process_name in LOCK_WHITE_LIST:
                while Locker.is_lock(process_name):
                    time.sleep(0.01)
                Locker.lock(process_name)
            rospy.logdebug('Cmd exec:\n {}'.format(cmd))
            cmd_output = envoy.run(cmd)
            if process_name in LOCK_WHITE_LIST:
                Locker.unlock(process_name)
            if cmd_output.status_code:
                warn_msg = '*** WARN: command error spotted from {}\n{}\n{}'.format(msg, cmd, cmd_output.std_err)
                rospy.logwarn(warn_msg)
                raise AssertionError(warn_msg)

            return cmd_output.std_out

        except:
            rospy.logerr('*** ERROR: command [{}]: {}'.format(cmd, sys.exc_info()[1]))