"""
Helper functions of the robot URDF model (Flexiv bimanual + Robotiq 2F-85).

Authors: Hongjie Fang.
"""

import os
import math
import numpy as np
import kinpy as kp


def calc_robotiq_state(open_length):
    return 0.725 - math.asin(open_length / 0.1143)


def convert_robotiq_gripper_joint_state(gripper_width):
    gripper_width = np.clip(gripper_width, 0, 0.085)
    state = calc_robotiq_state(gripper_width)

    return {
        'finger_joint': state,
        'left_outer_finger_joint': 0, 
        'left_inner_knuckle_joint': state, 
        'left_inner_finger_joint': -state, 
        'right_inner_knuckle_joint': -state, 
        'right_inner_finger_joint': state, 
        'right_outer_knuckle_joint': -state, 
        'right_outer_finger_joint': 0
    }


def convert_joint_states_single(
    joint,
    joint_cfgs,
    is_rad = True,
    **kwargs
):
    joint_states = {}

    for idx in range(joint_cfgs.num_robot_joints):
        joint_states["joint{}".format(idx + 1)] = joint[idx]
        if not is_rad:
            joint_states["joint{}".format(idx + 1)] = joint[idx] / 180.0 * np.pi
    
    gripper_joint_states = convert_robotiq_gripper_joint_state(joint[joint_cfgs.num_robot_joints])

    return {**joint_states, **gripper_joint_states}


def convert_joint_states(
    left_joint, 
    right_joint, 
    left_joint_cfgs,
    right_joint_cfgs,
    is_rad = True,
    seperate = False,
    **kwargs
):
    left_joint_states = convert_joint_states_single(left_joint, left_joint_cfgs, is_rad = is_rad, **kwargs)
    right_joint_states = convert_joint_states_single(right_joint, right_joint_cfgs, is_rad = is_rad, **kwargs)
    
    if seperate:
        return left_joint_states, right_joint_states

    joint_states = {}
    for key, value in left_joint_states.items():
        joint_states["left_{}".format(key)] = value
    for key, value in right_joint_states.items():
        joint_states["right_{}".format(key)] = value
    return joint_states


def forward_kinematic_single(
    joint,
    joint_cfgs,
    is_rad = True,
    urdf_file = os.path.join("airexo", "urdf_models", "robot", "left_robot_inhand.urdf"),
    with_visuals_map = True,
    **kwargs
):
    """
    Forward kinematic of single robot arm.

    Parameters
    - joint: joint encoder readings of the robot;
    - joint_cfgs: joint configurations of the robot;
    - is_rad: whether the joint value is represented in radius;
    - urdf_file: the urdf file of the robot;
    - with_visuals_map: whether to return visuals map.

    Returns:
    - transforms: the transformations matrix of each mesh w.r.t. robot base;
    - visuals_map: the mapping to retrieve the corresponding mesh name.
    """

    model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))

    joint_states = convert_joint_states_single(
        joint = joint,
        joint_cfgs = joint_cfgs,
        is_rad = is_rad,
        **kwargs
    )

    if with_visuals_map:
        return model_chain.forward_kinematics(joint_states), model_chain.visuals_map()
    else:
        return model_chain.forward_kinematics(joint_states)


def forward_kinematic(
    left_joint, 
    right_joint,
    left_joint_cfgs,
    right_joint_cfgs,
    is_rad = True,
    urdf_file = os.path.join("airexo", "urdf_models", "robot", "robot.urdf"),
    with_visuals_map = True,
    **kwargs
):
    """
    Forward kinematic of dual robot arm.

    Parameters:
    - left_joint: left joint encoder readings of the robot;
    - right_joint: right joint encoder readings of the robot;
    - left_joint_cfgs: left joint configurations of the robot;
    - right_joint_cfgs: right joint configurations of the robot;
    - is_rad: whether the joint value is represented in radius;
    - urdf_file: the urdf file of the robot;
    - with_visuals_map: whether to return visuals map.

    Returns:
    - transforms: the transformations matrix of each mesh w.r.t. robot base;
    - visuals_map: the mapping to retrieve the corresponding mesh name.
    """

    model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))

    joint_states = convert_joint_states(
        left_joint = left_joint,
        right_joint = right_joint,
        left_joint_cfgs = left_joint_cfgs,
        right_joint_cfgs = right_joint_cfgs,
        is_rad = is_rad,
        seperate = False,
        **kwargs
    )

    if with_visuals_map:
        return model_chain.forward_kinematics(joint_states), model_chain.visuals_map()
    else:
        return model_chain.forward_kinematics(joint_states)
