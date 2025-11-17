#!/usr/bin/env python
import rospy
import zmq

from pupil_ros_bridge.conversion import convert_gaze_to_msg
from pupil_ros_bridge.dummy_data import GAZE_DATA
from pupil_ros_bridge.msg import Gaze


def talker():
    pub = rospy.Publisher('gaze', Gaze, queue_size=10)
    rospy.init_node('gaze_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(rospy.Time.now())
        gaze_msg = convert_gaze_to_msg(GAZE_DATA)
        pub.publish(gaze_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass