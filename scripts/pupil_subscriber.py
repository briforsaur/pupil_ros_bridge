#!/usr/bin/env python
import rospy

from pupil_ros_bridge.msg import Pupil
from pupil_ros_bridge.tools import test_function

def callback(data: Pupil):
    rospy.loginfo(f"{rospy.get_caller_id()}I heard {data.pupil_timestamp}, {data.norm_pos}")
    
def listener():
    test_function(2)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pupil_subscriber', anonymous=True)

    rospy.Subscriber("pupil", Pupil, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()