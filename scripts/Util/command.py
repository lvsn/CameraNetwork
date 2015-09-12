import os
import sys
import time
import errno

import envoy

try:
    import rospy
except ImportError:
    pass


try:
    import RPi.GPIO as GPIO
except RuntimeError:
    pass


from constant import *

__author__ = 'jbecirovski'


class Locker(object):
    """ Control lock process by creating and deleting empty file inside tmpfs folder (LOCK_DIR) """
    def __init__(self, process_name=LOCK_DEFAULT_PROCESS):
        self.p = process_name
        self.fn = os.path.join(LOCK_DIR, process_name)

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
        return os.path.exists(os.path.join(LOCK_DIR, process_name))

    def __enter__(self):
        while True:
            try:
                os.open(self.fn, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            except Exception as e:
                if e.errno == errno.ENOENT:
                    time.sleep(1)
                    continue
            break

    def __exit__(self):
        os.remove(self.fn)


class Command(object):
    @staticmethod
    def run(cmd, msg='unspecified category'):
        """
        run(cmd, msg) -> str(command output)

        Safety command runner :
        1 - Check if command is locked
        2 - Manage locker to execute command if it is necessary
        3 - Manage specific error
        4 - Output command std_out
        """
        try:
            process_name = cmd.split()[0]
            if process_name in LOCK_WHITE_LIST:
                while Locker.is_lock(process_name):
                    time.sleep(0.01)
                Locker.lock(process_name)
            rospy.loginfo('Cmd exec:\n {}'.format(cmd))
            time.sleep(0.05)
            cmd_output = envoy.run(cmd)
            if process_name in LOCK_WHITE_LIST:
                Locker.unlock(process_name)
            if cmd_output.status_code:
                warn_msg = '*** WARN: command error spotted from {}\n{}\n{}'.format(msg, cmd, cmd_output.std_err)
                time.sleep(0.05)
                rospy.logwarn(warn_msg)

                # Manage specific errors
                Command.error_manager(cmd_output.std_err)
                raise AssertionError(warn_msg)

            time.sleep(0.05)
            return cmd_output.std_out

        except:
            rospy.logerr('*** ERROR: command [{}]: {}'.format(cmd, sys.exc_info()[1]))

    @staticmethod
    def error_manager(std_err):
        """ Manage specific errors thanks to std_err log """
        if 'No camera found' in std_err:
            rospy.loginfo('Camera not detected. Camera is rebooting ...')
            CameraPowerController.camera_reboot_script()


class CameraPowerController(object):
    """ Control camera's power from pin GPIO_POWER_CONTROL """
    def __init__(self):
        if 'rospy' in sys.modules.keys():
            rospy.loginfo('*** CameraManagementBegin ***')

        self.gpio_id = GPIO_POWER_CONTROL
        self.module_name = 'RPi.GPIO'

        if self.module_name in sys.modules:
            try:
                GPIO.setmode(GPIO.Board)
                GPIO.setup(self.gpio_id, GPIO.OUT)
            except:
                if 'rospy' in sys.modules.keys():
                    rospy.logdebug('GPIO are already in use.')
    
    @staticmethod
    def camera_reboot_script():
        """ Launch script with root permission for GPIO management """
        CURRENT_DIR = os.path.dirname(__file__)
        file_path = os.path.join(CURRENT_DIR, 'cam_pw_manag.py')
        Command.run('sudo python {}'.format(file_path, 'Camera power reboot launch script'))
        time.sleep(5)


    def camera_power_on(self):
        """ Power on the camera """
        if self.module_name in sys.modules:
            try:
                GPIO.output(self.gpio_id, True)
            except:
                if 'rospy' in sys.modules.keys():
                    rospy.logerr('*** ERROR: GPIO pin {} output configuration fail'.format(self.gpio_id))
        else:
            print('GPIO {} is ON'.format(self.gpio_id))
        if 'rospy' in sys.modules.keys():
            rospy.loginfo("Camera is power on")

    def camera_power_off(self):
        """ Power off the camera """
        if self.module_name in sys.modules:
            try:
                GPIO.output(self.gpio_id, False)
            except:
                if 'rospy' in sys.modules.keys():
                    rospy.logerr('*** ERROR: GPIO pin {} output configuration fail'.format(self.gpio_id))
        else:
            print('GPIO {} is OFF'.format(self.gpio_id))
        if 'rospy' in sys.modules.keys():
            rospy.loginfo("Camera is power off")

