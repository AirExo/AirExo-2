import numpy as np

#################### Robot Constants ####################

ROBOT_LEFT_FLANGE_TO_CAM = np.array(
    [
        [-0.01240050, 0.99905890, 0.04157753, -0.09342833],
        [-0.99978572, -0.01307791, 0.01605635, 0.02158097],
        [0.01658444, -0.04136945, 0.99900651, -0.00380356],
        [0.00000000, 0.00000000, 0.00000000, 1.00000000]
    ], dtype = np.float32
)

ROBOT_RIGHT_FLANGE_TO_CAM = np.array(
    [
        [-0.10785065, 0.99415284, 0.00529436, -0.08709522],
        [-0.99405175, -0.10791755, 0.01465851, 0.00088501],
        [0.01514411, -0.00368180, 0.99987853, -0.00513650],
        [0.00000000, 0.00000000, 0.00000000, 1.00000000]
    ], dtype = np.float32
)

ROBOT_TCP_TO_FLANGE = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, -0.170],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32
)

ROBOT_LEFT_TCP_TO_CAM = ROBOT_TCP_TO_FLANGE @ ROBOT_LEFT_FLANGE_TO_CAM
ROBOT_RIGHT_TCP_TO_CAM = ROBOT_TCP_TO_FLANGE @ ROBOT_RIGHT_FLANGE_TO_CAM

ROBOT_LEFT_CAM_TO_TCP = np.linalg.inv(ROBOT_LEFT_TCP_TO_CAM)
ROBOT_RIGHT_CAM_TO_TCP = np.linalg.inv(ROBOT_RIGHT_TCP_TO_CAM)

# Note. the following transformations correspond to the Flexiv robot API transformations, not
#       URDF transformations. After setting up the mounting options, the Flexiv robot API will
#       autonomous rotate the axis when calculating tcp pose.
ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE = 0.135
ROBOT_LEFT_REAL_BASE_TO_REAL_BASE = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32
)
ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32
)

#################### AirExo Constants ####################

AIREXO_BASE_TO_MARKER = np.array(
    [
        [0, 1, 0, 0],
        [0, 0, 1, 0.037 + 0.003 + 0.055],
        [1, 0, 0, 0.3525],
        [0, 0, 0, 1]
    ], dtype = np.float32
)

AIREXO_LEFT_TCP_TO_JOINT7 = np.array(
    [
        [0.0, -1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, -0.245],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32
)

AIREXO_RIGHT_TCP_TO_JOINT7 = np.array(
    [
        [0.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, -0.245],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype = np.float32
)

#################### URDF Constants ####################

# Robot URDF base to real base
ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype = np.float32)

LEFT_ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [1, 0, 0, 0],
    [0, np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, -np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, 0, 0, 1]
], dtype = np.float32)

RIGHT_ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [1, 0, 0, 0],
    [0, np.sqrt(2) / 2, -np.sqrt(2) / 2, 0],
    [0, np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
    [0, 0, 0, 1]
], dtype = np.float32)

AIREXO_PREDEFINED_TRANSFORMATION = np.array([
    [0, 1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 0, 0, 1]
], dtype = np.float32)

#################### Render Constants ####################
O3D_RENDER_TRANSFORMATION = np.array(
    [
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ], dtype = np.float32
)

#################### Rotation Representation Constants ####################
VALID_ROTATION_REPRESENTATIONS = [
    'axis_angle',
    'euler_angles',
    'quaternion',
    'matrix',
    'rotation_6d',
    'rotation_9d',
    'rotation_10d'
]
ROTATION_REPRESENTATION_DIMS = {
    'axis_angle': 3,
    'euler_angles': 3,
    'quaternion': 4,
    'matrix': 9,
    'rotation_6d': 6,
    'rotation_9d': 9,
    'rotation_10d': 10
}
