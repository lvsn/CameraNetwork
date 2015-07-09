import os
import rospy

__author__ = 'jbecirovski'


class Locker:
    """  Locker is a simple file creator in tmpfs folder for locking some process and manage access services  """
    def __init__(self):
        self.lock_dir = (os.environ.get('LOCKDIR') if os.path.exists(os.environ.get('LOCKDIR')) else '/run/lock/')
        self.prefix = 'camnet_'
        self.white_list = {'gphoto2'}
        self.lock_status = dict(zip(self.white_list, [False for x in self.white_list]))

    def lock(self, process_name):
        """
        Routine: Lock process name in white list otherwise raise error.
        :param process_name: str
        """
        try:
            if not self.lock_status[process_name]:
                self._create_lock_file(process_name)
            else:
                rospy.logwarn('%s already locked' % process_name)
                raise Exception('%s already locked' % process_name)
        except KeyError as msg:
            rospy.logerr("%s is not in lock white list" % process_name)
            raise NameError(msg)

    def unlock(self, process_name):
        """
        Routine: Unlock process name in white list otherwise raise error.
        :param process_name: str
        """
        try:
            if self.lock_status[process_name]:
                self._delete_lock_file(process_name)
            else:
                rospy.logwarn('%s already unlocked' % process_name)
                raise Exception('%s already unlocked' % process_name)
        except KeyError as msg:
            rospy.logerr("%s is not in lock white list" % process_name)
            raise NameError(msg)

    def is_locked(self, process_name):
        """
        Function: Return process name lock status if it's in white list otherwise raise error.
        :param process_name: str
        :return: bool
        """
        try:
            if self.lock_status[process_name]:
                return True
            else:
                return False
        except KeyError as msg:
            rospy.logerr("%s is not in lock white list" % process_name)
            raise NameError(msg)

    def _update(self):
        """  Update lock status of all white list processes  """
        for process in self.white_list:
            if os.path.exists(os.path.join(self.lock_dir, '{}{}'.format(self.prefix, process))):
                self.lock_status[process] = True
            else:
                self.lock_status[process] = False

    def _create_lock_file(self, process_name):
        """
        Routine: Create file named like process_name in lock directory system.
        :param process_name: str
        """
        pass

    def _delete_lock_file(self, process_name):
        """
        Routine: Dlete file named like process_name in lock directory system.
        :param process_name: str
        """
        pass