#!/usr/bin/env python
import rospy
import zmq

from pupil_ros_bridge.msg import Pupil, Cartesian2D

def talker():
    pub = rospy.Publisher('pupil', Pupil, queue_size=10)
    rospy.init_node('pupil_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    timestamp = 0.
    while not rospy.is_shutdown():
        rospy.loginfo(timestamp)
        norm_pos = Cartesian2D(1., 2.)
        pub.publish(pupil_timestamp=timestamp, norm_pos=norm_pos)
        rate.sleep()
        timestamp += 0.01

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass