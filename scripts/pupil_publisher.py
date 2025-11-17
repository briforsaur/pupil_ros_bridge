#!/usr/bin/env python
import rospy
from typing import Any, Dict
import zmq

from pupil_ros_bridge.msg import Pupil, Cartesian2D, Cartesian3D, Ellipse, EllipseAxes, Sphere, ProjectedSphere, Circle3D
from pupil_ros_bridge.dummy_data import PUPIL_DATA

def convert_pupil_to_msg(pupil_data: Dict[str, Any]) -> Pupil:
    pupil_data["world_index"] = pupil_data.get("world_index", -1)
    return Pupil(
        timestamp=pupil_data['timestamp'],
        world_index=pupil_data['world_index'],
        confidence=pupil_data['confidence'],
        norm_pos=Cartesian2D(*pupil_data['norm_pos']),
        diameter=pupil_data['diameter'],
        ellipse=Ellipse(
            center=Cartesian2D(*pupil_data['ellipse']['center']),
            axes=EllipseAxes(*pupil_data['ellipse']['axes']),
            angle=pupil_data['ellipse']['angle']
        ),
        diameter_3d=pupil_data['diameter_3d'],
        sphere=Sphere(
            center=Cartesian3D(*pupil_data['sphere']['center']),
            radius=pupil_data['sphere']['radius'],
        ),
        circle_3d=Circle3D(
            center=Cartesian3D(*pupil_data['circle_3d']['center']),
            normal=Cartesian3D(*pupil_data['circle_3d']['normal']),
            radius=pupil_data['circle_3d']['radius'],
        ),
        theta=pupil_data['theta'],
        phi=pupil_data['phi'],
        projected_sphere=ProjectedSphere(
            center=Cartesian2D(*pupil_data['projected_sphere']['center']),
            axes=EllipseAxes(*pupil_data['projected_sphere']['axes']),
            angle=pupil_data['projected_sphere']['angle'],
        )
    )

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