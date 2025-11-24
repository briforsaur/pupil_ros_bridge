from msgpack import unpackb
from typing import Any, Iterable, Dict, Tuple
import zmq

from .constants import Topic

class PupilNetworkHandler:

    def __init__(self, pupil_ip: str, pupil_port: str, topics: Iterable[str]) -> None:
        _topics = [Topic(topic) for topic in topics]
        context = zmq.Context()
        # Open requests port
        self._req = context.socket(zmq.REQ)
        self._req.connect(f"tcp://{pupil_ip}:{pupil_port}")
        # Open subscription port
        self._req.send_string("SUB_PORT")
        sub_port = self._req.recv_string()
        self._sub = context.socket(zmq.SUB)
        sub_uri = f"tcp://{pupil_ip}:{sub_port}"
        self._sub.connect(sub_uri)
        print(f"Pupil subscription port established at {sub_uri}")
        # Set subscription topic
        self._subscribe_to(_topics)

    def _subscribe_to(self, topics: Iterable[Topic]) -> None:
        for topic in topics:
            if topic == Topic.PUPIL:
                self._sub.setsockopt_string(zmq.SUBSCRIBE, f"{topic.value}.0.3d")
                self._sub.setsockopt_string(zmq.SUBSCRIBE, f"{topic.value}.1.3d")
            else:
                self._sub.setsockopt_string(zmq.SUBSCRIBE, f"{topic.value}")
    
    def _recv_from_sub(self) -> Tuple[str, Dict[str, Any]]:
        topic = self._sub.recv_string()
        payload = unpackb(self._sub.recv(), raw=False)
        return topic, payload
    
    def _has_new_data_available(self) -> bool:
        return self._sub.get(zmq.EVENTS) & zmq.POLLIN # type: ignore
    
    def get_latest_data(self) -> Dict[Topic, Dict[str, Any]]:
        data = {}
        while self._has_new_data_available():
            # Continue collecting data from the buffer until none are left
            topic, payload = self._recv_from_sub()
            main_topic, subtopic = topic.split(".", maxsplit=1)
            data.update({Topic(main_topic): payload})
        return data