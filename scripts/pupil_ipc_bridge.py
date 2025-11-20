#!/usr/bin/env python
from argparse import ArgumentParser, Namespace
import rospy
from typing import Iterable

from pupil_ros_bridge.constants import (
    DEFAULT_PUPIL_IP, 
    DEFAULT_PUPIL_PORT, 
    Topic
)
from pupil_ros_bridge.ipc import PupilNetworkHandler
from pupil_ros_bridge.publishers import run_ipc_bridge


def main(
    topics: Iterable[str], 
    ipc_ip: str = DEFAULT_PUPIL_IP, 
    ipc_port: str = DEFAULT_PUPIL_PORT
) -> None:
    pupil_handler = PupilNetworkHandler(ipc_ip, ipc_port, topics)
    run_ipc_bridge(topics, pupil_handler)


def _get_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument(
        "topics",
        nargs='+',
        choices=[member.value for member in Topic], 
        help="Pupil topics to subscribe to."
    )
    parser.add_argument(
        "--ipc_ip",
        default=DEFAULT_PUPIL_IP,
        help=(
            "IPv4 address that the Pupil Network plugin is broadcasting on. "
            f"(default={DEFAULT_PUPIL_IP})"
        )
    )
    parser.add_argument(
        "--ipc_port",
        default=DEFAULT_PUPIL_PORT,
        help=(
            "Network port that the Pupil Network plugin is broadcasting on. "
            f"(default={DEFAULT_PUPIL_PORT})"
        ),
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = _get_args()
    try:
        main(**vars(args))
    except rospy.ROSInterruptException:
        pass