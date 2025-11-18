from enum import Enum, unique
from dataclasses import dataclass
from genpy import Message
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
class TopicParams:
    message: Type[PupilMessageType]
    conversion_fcn: ConversionFcnType


TOPIC_PARAMS = {
    Topic.PUPIL: TopicParams(
        message=Pupil, conversion_fcn=convert_pupil_to_msg
    ),
    Topic.GAZE: TopicParams(
        message=Gaze, conversion_fcn=convert_gaze_to_msg
    ),
}