# Copyright 2025 Shane Forbrigger
# Licensed under the MIT License (see LICENSE file in project root)

from enum import Enum, unique
from dataclasses import dataclass
from typing import Any, Callable, Dict, Type, Union

from .conversion import convert_pupil_to_msg, convert_gaze_to_msg
from pupil_ros_bridge.msg import Pupil, Gaze


DEFAULT_PUPIL_IP = "127.0.0.1"
DEFAULT_PUPIL_PORT = "50020"


@unique
class Topic(Enum):
    PUPIL = "pupil"
    GAZE = "gaze"


PupilMessageType = Union[Pupil, Gaze]
ConversionFcnType = Callable[[Dict[str, Any]], PupilMessageType]


@dataclass
class TopicConfig:
    message: Type[PupilMessageType]
    conversion_fcn: ConversionFcnType


TOPIC_CONFIGS = {
    Topic.PUPIL: TopicConfig(
        message=Pupil, conversion_fcn=convert_pupil_to_msg
    ),
    Topic.GAZE: TopicConfig(
        message=Gaze, conversion_fcn=convert_gaze_to_msg
    ),
}