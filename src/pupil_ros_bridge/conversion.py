from typing import Any, Dict, List

from pupil_ros_bridge.msg import (
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
    eye_ids = get_eye_ids_from(topic)
    # The eye center and gaze normal keys vary depending on if one or both eyes are used
    # in the gaze mapping. The base data structure also changes
    if len(eye_ids) == 2: # both eyes were used
        eye_centers = [
            Cartesian3D(*data) for data in gaze_data['eye_centers_3d'].values()
        ]
        gaze_normals = [
            Cartesian3D(*data) for data in gaze_data['gaze_normals_3d'].values()
        ]
        base_timestamps = [data['timestamp'] for data in gaze_data['base_data']]
    else:
        # Replacing missing eye data with NaN
        eye_id = eye_ids[0]
        default_3d = Cartesian3D(*[float('nan') for i in range(3)])
        eye_centers = [default_3d, default_3d]
        eye_centers[eye_id] = Cartesian3D(*gaze_data['eye_center_3d'])
        gaze_normals = [default_3d, default_3d]
        gaze_normals[eye_id] = Cartesian3D(*gaze_data['gaze_normal_3d'])
        base_timestamps = [float('nan'), float('nan')]
        base_timestamps[eye_id] = gaze_data['base_data'][0]['timestamp']
    return Gaze(
        timestamp=gaze_data['timestamp'],
        eye_centers_3d=eye_centers,
        gaze_normals_3d=gaze_normals,
        gaze_point_3d=Cartesian3D(*gaze_data['gaze_point_3d']),
        norm_pos=Cartesian2D(*gaze_data['norm_pos']),
        confidence=gaze_data['confidence'],
        base_timestamps=base_timestamps,
    )


def get_eye_ids_from(topic: str) -> List[int]:
    """Get a list of integers representing the eyes used to generate a gaze datum

    Pupil Capture reports which eye data were used to generate a gaze datum using the
    topic field of the gaze data structure. The topic is of the form 
    ``"gaze.3d.[ids]"``, where ``ids`` can have the following values:

        ``"0"`` - eye 0 (the right eye) was used

        ``"1"`` - eye 1 (the left eye) was used

        ``"01"`` - both eyes were used

    Parameters
    ----------
    topic: str
        The Pupil data topic string, of the form ``"gaze.3d.[ids]"``

    Returns
    -------
    List[int]
        A list of integers
    """
    eye_ids = topic.split('.')[2]
    return [int(eye_id) for eye_id in eye_ids]


_BASE_IDS = {
    '0': 1, # Eye 0 was used to determine the gaze
    '1': 2, # Eye 1 was used to determine the gaze
    '01': 3, # Eyes 0 and 1 were used to determine the gaze
}