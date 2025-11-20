from typing import Dict, Iterable
import rospy
import zmq

from .dummy_data import ALL_DATA
from .constants import Topic, TOPIC_CONFIGS
from .ipc import PupilNetworkHandler


def run_ipc_bridge(topics: Iterable[str], pupil_handler: PupilNetworkHandler) -> None:
    # Converting input topic strings to a list of corresponding enums
    _topics = [Topic(topic) for topic in topics]
    # Constucting publishers for each topic with appropriate message types
    publishers = {
        topic: rospy.Publisher(
            topic.value, TOPIC_CONFIGS[topic].message, queue_size=10
        ) 
        for topic in _topics
    }
    rospy.init_node(f'pupil_ipc_publisher')
    ros_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(rospy.Time.now())
        data = pupil_handler.get_latest_data()
        for topic, datum in data.items():
            # Get the conversion function for corresponding topic
            conversion_fcn = TOPIC_CONFIGS[topic].conversion_fcn
            msg = conversion_fcn(datum)
            publishers[topic].publish(msg)
        ros_rate.sleep()