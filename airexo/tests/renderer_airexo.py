import os
import cv2
import time
import hydra
import numpy as np
import matplotlib.pyplot as plt

from omegaconf import OmegaConf

from airexo.helpers.constants import *
from airexo.helpers.transform import transform_arm


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests", "renderer"),
    config_name = "airexo"
)
def main(cfg):
    OmegaConf.resolve(cfg)
    
    if not cfg.fixed:
        left_airexo = hydra.utils.instantiate(cfg.left_airexo)
        right_airexo = hydra.utils.instantiate(cfg.right_airexo)
        time.sleep(2)
        left_joint = left_airexo.get_angle()
        right_joint = right_airexo.get_angle()
    else:
        left_joint = cfg.fixed_left_airexo
        right_joint = cfg.fixed_right_airexo
    
    calib_info = hydra.utils.instantiate(cfg.calib_info)

    airexo_renderer = hydra.utils.instantiate(
        cfg.airexo_renderer, 
        cam_to_base = calib_info.get_camera_to_base(cfg.camera_serial),
        intrinsic = calib_info.get_intrinsic(cfg.camera_serial)
    )

    airexo_renderer.update_joints(left_joint, right_joint)
    image_airexo = airexo_renderer.render_image()
    depth_airexo = airexo_renderer.render_depth()
    mask_airexo = airexo_renderer.render_mask(depth = depth_airexo)

    del airexo_renderer

    robot_renderer = hydra.utils.instantiate(
        cfg.robot_renderer,
        cam_to_base = calib_info.get_camera_to_base(cfg.camera_serial),
        intrinsic = calib_info.get_intrinsic(cfg.camera_serial)
    )

    robot_left_joint = transform_arm(
        robot_cfgs = cfg.robot_left_joint_cfgs,
        airexo_cfgs = cfg.airexo_left_joint_cfgs,
        calib_cfgs = cfg.left_calib_cfgs,
        data = left_joint
    )
    robot_right_joint = transform_arm(
        robot_cfgs = cfg.robot_right_joint_cfgs,
        airexo_cfgs = cfg.airexo_right_joint_cfgs,
        calib_cfgs = cfg.right_calib_cfgs,
        data = right_joint
    )

    robot_renderer.update_joints(robot_left_joint, robot_right_joint)
    image_robot = robot_renderer.render_image()
    depth_robot = robot_renderer.render_depth()
    mask_robot = robot_renderer.render_mask(depth = depth_robot)

    del robot_renderer

    # Create a figure with 2 rows and 3 columns
    fig, axes = plt.subplots(2, 3, figsize=(12, 8))

    # Display each image in the correct subplot
    axes[0, 0].imshow(image_airexo)
    axes[0, 0].set_title("RGB [AirExo]")
    axes[0, 0].axis('off')

    axes[0, 1].imshow(depth_airexo, cmap='plasma')
    axes[0, 1].set_title("Depth [AirExo]")
    axes[0, 1].axis('off')

    axes[0, 2].imshow(mask_airexo, cmap='gray')
    axes[0, 2].set_title("Mask [AirExo]")
    axes[0, 2].axis('off')

    axes[1, 0].imshow(image_robot)
    axes[1, 0].set_title("RGB [Robot]")
    axes[1, 0].axis('off')

    axes[1, 1].imshow(depth_robot, cmap='plasma')
    axes[1, 1].set_title("Depth [Robot]")
    axes[1, 1].axis('off')

    axes[1, 2].imshow(mask_robot, cmap='gray')
    axes[1, 2].set_title("Mask [Robot]")
    axes[1, 2].axis('off')

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()

    if not cfg.fixed:
        left_airexo.stop()
        right_airexo.stop()
    

if __name__ == '__main__':
    main()