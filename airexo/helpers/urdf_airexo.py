"""
Helper functions of the AirExo URDF model.

Authors: Hongjie Fang.
"""

import os
import numpy as np
import kinpy as kp

from airexo.helpers.degree import deg_2_rad, rad_2_deg, deg_percentile, deg_clip_in_range


def convert_parallel_gripper_joint_state(gripper_state, amin, amax, adir):
    gripper_state = deg_clip_in_range(gripper_state, amin, amax, adir)
    gripper_width = deg_percentile(gripper_state, amin, amax, adir) * 0.105
    return {
        "gripper_finger1": -gripper_width / 2,
        "gripper_finger2": gripper_width / 2
    }
    

def convert_joint_states(
    left_joint, 
    right_joint, 
    left_joint_cfgs,
    right_joint_cfgs,
    left_calib_cfgs,
    right_calib_cfgs,
    is_rad = True,
    **kwargs
):
    """
    Convert joint states for AirExo.
    """
    joint_states = {}
    
    for idx in range(left_joint_cfgs.num_robot_joints):
        value = left_joint[idx]
        if is_rad:
            value = rad_2_deg(value)
        value = left_calib_cfgs["joint{}".format(idx + 1)].airexo - value
        value = value * left_joint_cfgs["joint{}".format(idx + 1)].direction
        value = deg_2_rad(value)
        joint_states["left_joint{}".format(idx + 1)] = value

    idx_gripper = left_joint_cfgs.num_robot_joints
    left_gripper_joint_states = convert_parallel_gripper_joint_state(
        left_joint[idx_gripper],
        amin = left_joint_cfgs["joint{}".format(idx_gripper + 1)].min,
        amax = left_joint_cfgs["joint{}".format(idx_gripper + 1)].max,
        adir = left_joint_cfgs["joint{}".format(idx_gripper + 1)].direction
    )
    for key, value in left_gripper_joint_states.items():
        joint_states["left_{}".format(key)] = value

    for idx in range(right_joint_cfgs.num_robot_joints):
        value = right_joint[idx]
        if is_rad:
            value = rad_2_deg(value)
        value = right_calib_cfgs["joint{}".format(idx + 1)].airexo - value
        value = value * right_joint_cfgs["joint{}".format(idx + 1)].direction
        value = deg_2_rad(value)
        joint_states["right_joint{}".format(idx + 1)] = value

    idx_gripper = right_joint_cfgs.num_robot_joints
    right_gripper_joint_states = convert_parallel_gripper_joint_state(
        right_joint[idx_gripper],
        amin = right_joint_cfgs["joint{}".format(idx_gripper + 1)].min,
        amax = right_joint_cfgs["joint{}".format(idx_gripper + 1)].max,
        adir = right_joint_cfgs["joint{}".format(idx_gripper + 1)].direction
    )
    for key, value in right_gripper_joint_states.items():
        joint_states["right_{}".format(key)] = -value

    return joint_states


def forward_kinematic(
    left_joint, 
    right_joint,
    left_joint_cfgs,
    right_joint_cfgs,
    left_calib_cfgs,
    right_calib_cfgs,
    is_rad = False,
    urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
    with_visuals_map = True,
    **kwargs
):
    """
    Forward kinematic of AirExo.

    Parameters:
    - left_joint: left joint encoder readings of AirExo;
    - right_joint: right joint encoder readings of AirExo;
    - left_joint_cfgs: left joint configurations of AirExo;
    - right_joint_cfgs: right joint configurations of AirExo;
    - left_calib_cfgs: left joint calibration configurations of AirExo;
    - right_calib_cfgs: right joint calibration configurations of AirExo;
    - is_rad: whether the joint value is represented in radius;
    - urdf_file: the urdf file of AirExo;
    - with_visuals_map: whether to return visuals map.

    Returns:
    - transforms: the transformations matrix of each mesh w.r.t. AirExo base;
    - visuals_map: the mapping to retrieve the corresponding mesh name.
    """

    model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))

    joint_states = convert_joint_states(
        left_joint = left_joint,
        right_joint = right_joint,
        left_joint_cfgs = left_joint_cfgs,
        right_joint_cfgs = right_joint_cfgs,
        left_calib_cfgs = left_calib_cfgs,
        right_calib_cfgs = right_calib_cfgs,
        is_rad = is_rad
    )

    if with_visuals_map:
        return model_chain.forward_kinematics(joint_states), model_chain.visuals_map()
    else:
        return model_chain.forward_kinematics(joint_states)
