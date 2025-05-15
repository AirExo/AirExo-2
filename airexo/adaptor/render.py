"""
Render Script.

Usage: 
  Set up all other configurations in the config files, then
  python -m airexo.scripts.render +path=[Task Path]
    
Authors: Hongjie Fang.
"""

import os
import cv2
import h5py
import hydra
import numpy as np

from tqdm import tqdm
from omegaconf import OmegaConf

from airexo.helpers.logger import setup_loggers
from airexo.helpers.transform import transform_arm


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "adaptor"),
    config_name = "render.yaml"
)
def main(cfg):
    setup_loggers()
    OmegaConf.resolve(cfg)  

    # set up paths
    path = cfg.path
    if path is None:
        raise AttributeError("Please provide scene path.")
    cam_path = os.path.join(path, "cam_{}".format(cfg.camera_serial))
    color_path = os.path.join(cam_path, "color")
    lowdim_path = os.path.join(path, "lowdim")

    airexo_path = os.path.join(cam_path, "render_airexo")
    airexo_color_path = os.path.join(airexo_path, "color")
    airexo_depth_path = os.path.join(airexo_path, "depth")
    airexo_mask_path = os.path.join(airexo_path, "mask")
    os.makedirs(airexo_path, exist_ok = True) 
    os.makedirs(airexo_color_path, exist_ok = True) 
    os.makedirs(airexo_depth_path, exist_ok = True) 
    os.makedirs(airexo_mask_path, exist_ok = True) 

    robot_path = os.path.join(cam_path, "render_robot")
    robot_color_path = os.path.join(robot_path, "color")
    robot_depth_path = os.path.join(robot_path, "depth")
    robot_mask_path = os.path.join(robot_path, "mask")
    os.makedirs(robot_path, exist_ok = True) 
    os.makedirs(robot_color_path, exist_ok = True) 
    os.makedirs(robot_depth_path, exist_ok = True) 
    os.makedirs(robot_mask_path, exist_ok = True) 

    # initialize timestamps, and fetch joint info
    timestamps = sorted([int(os.path.splitext(x)[0]) for x in os.listdir(color_path)])
    airexo_left_file = h5py.File(os.path.join(lowdim_path, "airexo_left.h5"), "r")
    airexo_right_file = h5py.File(os.path.join(lowdim_path, "airexo_right.h5"), "r")

    airexo_left = []
    airexo_right = []
    for timestamp in timestamps:
        idx_left_airexo = np.argmin(np.abs(np.array(airexo_left_file["timestamp"]) - int(timestamp)))
        idx_right_airexo = np.argmin(np.abs(np.array(airexo_right_file["timestamp"]) - int(timestamp)))
        airexo_left.append(airexo_left_file["encoder"][idx_left_airexo])
        airexo_right.append(airexo_right_file["encoder"][idx_right_airexo])

    airexo_left_file.close()
    airexo_right_file.close()

    # initialize caliberation info
    calib_info = hydra.utils.instantiate(cfg.calib_info)

    # render airexo images
    airexo_renderer = hydra.utils.instantiate(
        cfg.airexo_renderer, 
        cam_to_base = calib_info.get_camera_to_base(cfg.camera_serial),
        intrinsic = calib_info.get_intrinsic(cfg.camera_serial)
    )

    for i in tqdm(range(len(timestamps))):
        timestamp = timestamps[i]
        airexo_renderer.update_joints(airexo_left[i], airexo_right[i])
        color = airexo_renderer.render_image()
        depth = airexo_renderer.render_depth()
        mask = airexo_renderer.render_mask(depth = depth)
        depth = np.clip(depth * 1000, 0, 65535).astype(np.uint16)

        cv2.imwrite(os.path.join(airexo_color_path, "{}.png".format(timestamp)), color[:, :, ::-1])
        cv2.imwrite(os.path.join(airexo_depth_path, "{}.png".format(timestamp)), depth)
        cv2.imwrite(os.path.join(airexo_mask_path, "{}.png".format(timestamp)), mask)

    del airexo_renderer

    # render robot images
    robot_renderer = hydra.utils.instantiate(
        cfg.robot_renderer,
        cam_to_base = calib_info.get_camera_to_base(cfg.camera_serial),
        intrinsic = calib_info.get_intrinsic(cfg.camera_serial)
    )

    for i in tqdm(range(len(timestamps))):
        timestamp = timestamps[i]

        robot_left_joint = transform_arm(
            robot_cfgs = cfg.robot_left_joint_cfgs,
            airexo_cfgs = cfg.airexo_left_joint_cfgs,
            calib_cfgs = cfg.left_calib_cfgs,
            data = airexo_left[i]
        )
        robot_right_joint = transform_arm(
            robot_cfgs = cfg.robot_right_joint_cfgs,
            airexo_cfgs = cfg.airexo_right_joint_cfgs,
            calib_cfgs = cfg.right_calib_cfgs,
            data = airexo_right[i]
        )

        robot_renderer.update_joints(robot_left_joint, robot_right_joint)
        color = robot_renderer.render_image()
        depth = robot_renderer.render_depth()
        mask = robot_renderer.render_mask(depth = depth)
        depth = np.clip(depth * 1000, 0, 65535).astype(np.uint16)

        cv2.imwrite(os.path.join(robot_color_path, "{}.png".format(timestamp)), color[:, :, ::-1])
        cv2.imwrite(os.path.join(robot_depth_path, "{}.png".format(timestamp)), depth)
        cv2.imwrite(os.path.join(robot_mask_path, "{}.png".format(timestamp)), mask)

    del robot_renderer


if __name__ == '__main__':
    main()