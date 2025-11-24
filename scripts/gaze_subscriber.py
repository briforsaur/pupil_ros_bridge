#!/usr/bin/env python

# Copyright 2025 Shane Forbrigger
# Licensed under the MIT License (see LICENSE file in project root)
import rospy

from pupil_ros_bridge.msg import Gaze

def callback(data: Gaze):
    rospy.loginfo(f"{rospy.get_caller_id()} I heard {data.timestamp},\n"
                  f"{data.norm_pos},\n{data.base_timestamps}")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gaze_subscriber', anonymous=True)

    rospy.Subscriber("gaze", Gaze, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()