#!/usr/bin/env python
# license removed for brevity
import rospy
import os
from std_msgs.msg import String

def ip_sender():
    pub = rospy.Publisher('ipchat', String, queue_size=10)
    rospy.init_node('ip_sender', anonymous=True)
    r = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        str = os.environ.get('ROS_IP')
        #rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    try:
        ip_sender()
    except rospy.ROSInterruptException: pass
