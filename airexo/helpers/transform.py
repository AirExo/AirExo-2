"""
Joint Mapping Functions.

Authors: Hongjie Fang.
"""

import numpy as np

import airexo.helpers.degree as utils_deg


def transform_joint(robot_cfg, airexo_cfg, calib_cfg, data):
    type = calib_cfg.type
    amin, amax, adir = airexo_cfg.min, airexo_cfg.max, airexo_cfg.direction
    rmin, rmax = robot_cfg.min, robot_cfg.max

    if airexo_cfg.rad:
        data = utils_deg.rad_2_deg(data)

    if type == "fixed":
        data = robot_cfg.fixed_value
        rdir = robot_cfg.direction
        if "zero_centered" in robot_cfg.keys() and robot_cfg.zero_centered:
            data = utils_deg.deg_zero_centered(data, rmin, rmax, rdir)
        if "rad" in robot_cfg.keys() and robot_cfg.rad:
            data = utils_deg.deg_2_rad(data)
        return data
    elif type == "scaling":
        data = utils_deg.deg_clip_in_range(data, amin, amax, adir)
        data = utils_deg.deg_percentile(data, amin, amax, adir)
        # return np.clip((data - amin) / (amax - amin), 0, 1) * (rmax - rmin) + rmin
        return data * (rmax - rmin) + rmin
    elif type == "mapping":
        data = utils_deg.deg_clip_in_range(data, amin, amax, adir)
        rdir = robot_cfg.direction
        amap = calib_cfg.airexo
        rmap = calib_cfg.robot
        data = utils_deg.deg_clip_in_range(rmap + utils_deg.deg_distance(amap, data, adir) * rdir, rmin, rmax, rdir)
        if "zero_centered" in robot_cfg.keys() and robot_cfg.zero_centered:
            data = utils_deg.deg_zero_centered(data, rmin, rmax, rdir)
        if "rad" in robot_cfg.keys() and robot_cfg.rad:
            data = utils_deg.deg_2_rad(data)
        return data
    
def transform_arm(robot_cfgs, airexo_cfgs, calib_cfgs, data):
    transformed_data = np.zeros_like(data)
    for joint_id in range(airexo_cfgs.num_joints):
        transformed_data[joint_id] = transform_joint(
            robot_cfg = robot_cfgs["joint{}".format(joint_id + 1)],
            airexo_cfg = airexo_cfgs["joint{}".format(joint_id + 1)],
            calib_cfg = calib_cfgs["joint{}".format(joint_id + 1)],
            data = data[joint_id]
        )
    return transformed_data