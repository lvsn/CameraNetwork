import os, sys, envoy, time

import rospy
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    pass

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
            rospy.logdebug('Cmd exec:\n {}'.format(cmd))
            cmd_output = envoy.run(cmd)
            if process_name in LOCK_WHITE_LIST:
                Locker.unlock(process_name)
            if cmd_output.status_code:
                warn_msg = '*** WARN: command error spotted from {}\n{}\n{}'.format(msg, cmd, cmd_output.std_err)
                rospy.logwarn(warn_msg)

                # Manage specific errors
                Command._error_manager(cmd_output.std_err)
                raise AssertionError(warn_msg)

            return cmd_output.std_out

        except:
            rospy.logerr('*** ERROR: command [{}]: {}'.format(cmd, sys.exc_info()[1]))

    @staticmethod
    def _error_manager(std_err):
        """ Manage specific errors thanks to std_err log """
        if 'No camera found' in std_err:
            rospy.loginfo('Camera not detected. Camera is rebooting ...')
            power_control = CameraPowerController()
            power_control.camera_power_off()
            rospy.sleep(1)
            power_control.camera_power_on()
            rospy.sleep(5)
            del power_control


class CameraPowerController(object):
    """ Control camera's power from pin GPIO_POWER_CONTROL """
    def __init__(self):
        self.gpio_id = GPIO_POWER_CONTROL
        self.module_name = 'RPi.GPIO'

        if self.module_name in sys.modules:
            try:
                GPIO.setmode(GPIO.Board)
                GPIO.setup(self.gpio_id, GPIO.OUT)
            except:
                rospy.logdebug('GPIO are already in use.')

    def camera_power_on(self):
        """ Power on the camera """
        if self.module_name in sys.modules:
            try:
                GPIO.output(self.gpio_id, True)
            except:
                rospy.logerr('*** ERROR: GPIO pin {} output configuration fail'.format(self.gpio_id))
        else:
            print('GPIO {} is ON'.format(self.gpio_id))
        rospy.loginfo("Camera is power on")

    def camera_power_off(self):
        """ Power off the camera """
        if self.module_name in sys.modules:
            try:
                GPIO.output(self.gpio_id, False)
            except:
                rospy.logerr('*** ERROR: GPIO pin {} output configuration fail'.format(self.gpio_id))
        else:
            print('GPIO {} is OFF'.format(self.gpio_id))
        rospy.loginfo("Camera is power off")

