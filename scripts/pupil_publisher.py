#!/usr/bin/env python
import rospy
import zmq

from pupil_ros_bridge.conversion import convert_pupil_to_msg
from pupil_ros_bridge.dummy_data import PUPIL_DATA
from pupil_ros_bridge.msg import Pupil


def talker():
    pub = rospy.Publisher('pupil', Pupil, queue_size=10)
    rospy.init_node('pupil_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(rospy.Time.now())
        pupil_msg = convert_pupil_to_msg(PUPIL_DATA)
        pub.publish(pupil_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass