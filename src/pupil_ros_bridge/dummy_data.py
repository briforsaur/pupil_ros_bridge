# Copyright 2025 Shane Forbrigger
# Licensed under the MIT License (see LICENSE file in project root)

from .constants import Topic


PUPIL_DATA = {
    'id': 1,
    'topic': 'pupil.1.3d',
    'method': 'pye3d 0.3.0 real-time',
    'norm_pos': [0.5938061519478426, 0.4389843750068906],
    'diameter': 41.224949577758856,
    'confidence': 0.99,
    'timestamp': 6.5765209999954095,
    'sphere': {
        'center': [4.67643591364379, 5.913909147596557, 36.528994515798], 
        'radius': 10.392304845413264,
    },
    'projected_sphere': {
        'center': [132.0299939915214, 141.60236768648187], 
        'axes': [192.58360848063384, 192.58360848063384], 
        'angle': 0.0,
    },
    'circle_3d': {
        'center': [1.53836466727819, 1.3412017267934662, 27.740203034799837], 
        'normal': [-0.30196104647090055, -0.4400089767210106, -0.8457018545676296],
        'radius': 1.7847506653755676,
    },
    'diameter_3d': 3.569501330751135,
    'ellipse': {
        'center': [114.01078117398578, 107.71499999867702], 
        'axes': [37.48489337063001, 41.224949577758856], 
        'angle': 59.37551243589842,
    },
    'location': [114.01078117398578, 107.71499999867702], 
    'model_confidence': 1.0,
    'theta': 2.026404996588558,
    'phi': -1.9137412671123133
}

GAZE_DATA = {
    'eye_center_3d': [-119.38909706848258, -388.0839708590091, 1283.4086286771635], 
    'gaze_normal_3d': [0.4621196785728402, -0.41186919787652404, 0.7853719924445254],
    'gaze_point_3d': [111.67074221793756, -594.0185697972712, 1676.0946248994262],
    'norm_pos': [0.5336138838348605, 0.8171195872805199],
    'topic': 'gaze.3d.1.',
    'confidence': 0.84503173828125,
    'timestamp': 6.528818999999203, 
    'base_data': [PUPIL_DATA]
}

ALL_DATA = {
    Topic.PUPIL: PUPIL_DATA, 
    Topic.GAZE: GAZE_DATA,
}