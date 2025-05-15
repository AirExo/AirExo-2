"""
AirExo / Robot State Transformer based on URDF.

Author: Hongjie Fang.
"""

import os

import airexo.helpers.urdf_airexo as airexo_helper

from airexo.helpers.constants import *


def airexo_transform_tcp(
    left_joint,
    right_joint,
    left_joint_cfgs,
    right_joint_cfgs,
    left_calib_cfgs,
    right_calib_cfgs,
    is_rad = False,
    urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
    real_robot_base = False,
    **kwargs
):
    """
    Transform AirExo joints to tcp poses.

    Parameters:
    - left_joint: left joint encoder readings of AirExo;
    - right_joint: right joint encoder readings of AirExo;
    - left_joint_cfgs: left joint configurations of AirExo;
    - right_joint_cfgs: right joint configurations of AirExo;
    - left_calib_cfgs: left joint calibration configurations of AirExo;
    - right_calib_cfgs: right joint calibration configurations of AirExo;
    - is_rad: whether the joint readings is represented by radius;
    - urdf_file: the urdf file of AirExo;
    - to_real_robot: whether to transform to real robot tcp.

    Returns:
    - left_tcp, right_tcp: the tcp poses of the left/right arm w.r.t. the AirExo base coordinate.
    """
    cur_transforms, visuals_map = airexo_helper.forward_kinematic(
        left_joint = left_joint,
        right_joint = right_joint,
        left_joint_cfgs = left_joint_cfgs,
        right_joint_cfgs = right_joint_cfgs,
        left_calib_cfgs = left_calib_cfgs,
        right_calib_cfgs = right_calib_cfgs,
        urdf_file = urdf_file,
        is_rad = is_rad,
        with_visuals_map = True
    )
    
    # Here, left/right tcp is defined on the (shared) base coordinate of AirExo and robot URDF files.
    left_tcp = AIREXO_PREDEFINED_TRANSFORMATION @ cur_transforms["L7"].matrix() @ visuals_map["L7"][0].offset.matrix() @ AIREXO_LEFT_TCP_TO_JOINT7
    right_tcp = AIREXO_PREDEFINED_TRANSFORMATION @ cur_transforms["R7"].matrix() @ visuals_map["R7"][0].offset.matrix() @ AIREXO_RIGHT_TCP_TO_JOINT7

    if real_robot_base:
        # Here, we transform the tcp to the left/right robot control coordinates.
        left_tcp = ROBOT_LEFT_REAL_BASE_TO_REAL_BASE @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION) @ left_tcp
        right_tcp = ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION) @ right_tcp

    return left_tcp, right_tcp


def robot_transform_tcp(
    left_tcp,
    right_tcp,
    rotation_rep = "quaternion",
    to_control = False,
):
    """
    Transformations between robot individual tcps and unified tcps in the base coordinate.
    
    Parameters:
    - left_tcp, right_tcp: the tcp poses of the left/right robot;
    - to_control: 
        + False: transform robot individual tcps to unified tcps;
        + True: transform unified tcps to robot individual tcps.
    
    Returns: the transformed tcp poses.
    """
    assert rotation_rep in VALID_ROTATION_REPRESENTATIONS, "Invalid rotation representation: {}".format(rotation_rep)
    
    if to_control:
        if rotation_rep != "matrix":
            left_tcp[1] = left_tcp[1] - ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
            right_tcp[1] = right_tcp[1] + ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
        else:
            left_tcp[1, 3] = left_tcp[1, 3] - ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
            right_tcp[1, 3] = right_tcp[1, 3] + ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
    else:
        if rotation_rep != "matrix":
            left_tcp[1] = left_tcp[1] + ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
            right_tcp[1] = right_tcp[1] - ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
        else:
            left_tcp[1, 3] = left_tcp[1, 3] + ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
            right_tcp[1, 3] = right_tcp[1, 3] - ROBOT_REAL_BASE_TO_INDIVIDUAL_REAL_BASE
    return left_tcp, right_tcp
