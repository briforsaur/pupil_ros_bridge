from typing import Any, Dict

from pupil_ros_bridge.msg import (
    BaseTimestamps,
    Cartesian2D,
    Cartesian3D,
    Circle3D,
    Ellipse,
    EllipseAxes,
    Gaze,
    ProjectedSphere,
    Pupil,
    Sphere,
)

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


def convert_gaze_to_msg(gaze_data: Dict[str, Any]) -> Gaze:
    topic: str = gaze_data['topic']
    eye_ids = topic.split('.')[2]
    base_id = _BASE_IDS[eye_ids]
    base_timestamps = [-1., -1.]
    for i, eye_id in enumerate(eye_ids):
        base_timestamps[int(eye_id)] = gaze_data['base_data'][i]['timestamp']
    return Gaze(
        timestamp=gaze_data['timestamp'],
        eye_center_3d=Cartesian3D(*gaze_data['eye_center_3d']),
        gaze_normal_3d=Cartesian3D(*gaze_data['gaze_normal_3d']),
        gaze_point_3d=Cartesian3D(*gaze_data['gaze_point_3d']),
        norm_pos=Cartesian2D(*gaze_data['norm_pos']),
        confidence=gaze_data['confidence'],
        base_id=base_id,
        base_timestamps=BaseTimestamps(*base_timestamps),
    )


def get_baseid_from_topic(topic: str) -> int:
    """Get an integer representing the eyes used to generate a gaze datum

    Pupil Capture reports which eye data were used to generate a gaze datum using the
    topic field of the gaze data structure. The topic is of the form 
    ``"gaze.[ids].3d"``, where ``ids`` can have the following values:

        ``"0"`` - eye 0 (the right eye) was used

        ``"1"`` - eye 1 (the left eye) was used

        ``"01"`` - both eyes were used

    Parameters
    ----------

    Returns
    -------
    int
        An integer representing the eyes used. Consider each eye as a digit in a binary
        number: ``00`` means neither eye was used (never the case), ``01`` (1) means
        that only the right eye was used, ``10`` (2) means that only the left eye was
        used, and ``11`` (3) means that both eyes were used.
    """
    eye_ids = topic.split('.')[2]
    return _BASE_IDS[eye_ids]


_BASE_IDS = {
    '0': 1, # Eye 0 was used to determine the gaze
    '1': 2, # Eye 1 was used to determine the gaze
    '01': 3, # Eyes 0 and 1 were used to determine the gaze
}