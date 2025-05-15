import os
import time
import hydra
import matplotlib.pyplot as plt

from omegaconf import OmegaConf

from airexo.helpers.constants import *


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests", "renderer"),
    config_name = "robot"
)
def main(cfg):
    OmegaConf.resolve(cfg)
    
    if not cfg.fixed:
        left_robot = hydra.utils.instantiate(cfg.left_robot)
        right_robot = hydra.utils.instantiate(cfg.right_robot)
        time.sleep(2)
        left_joint = left_robot.get_joint_pos()
        left_gripper_width = left_robot.gripper.get_gripper_width()
        right_joint = right_robot.get_joint_pos()
        right_gripper_width = right_robot.gripper.get_gripper_width()
        left_joint = np.concatenate([left_joint, [left_gripper_width]], axis = 0)
        right_joint = np.concatenate([right_joint, [right_gripper_width]], axis = 0)
    else:
        left_joint = cfg.fixed_left_robot
        right_joint = cfg.fixed_right_robot
    
    calib_info = hydra.utils.instantiate(cfg.calib_info)

    renderer = hydra.utils.instantiate(
        cfg.renderer, 
        cam_to_base = calib_info.get_camera_to_base(cfg.camera_serial),
        intrinsic = calib_info.get_intrinsic(cfg.camera_serial)
    )

    # Render robot on robot image.
    tic = time.time()
    renderer.update_joints(left_joint, right_joint)
    image = renderer.render_image()
    depth = renderer.render_depth()
    mask = renderer.render_mask(depth = depth)
    print("Elapsed time:", time.time() - tic)
    
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))

    # Display each image in the correct subplot
    axes[0].imshow(image)
    axes[0].set_title("RGB [Robot]")
    axes[0].axis('off')

    axes[1].imshow(depth, cmap='plasma')
    axes[1].set_title("Depth [Robot]")
    axes[1].axis('off')

    axes[2].imshow(mask, cmap='gray')
    axes[2].set_title("Mask [Robot]")
    axes[2].axis('off')

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()

    if not cfg.fixed:
        left_robot.stop()
        right_robot.stop()
    

if __name__ == '__main__':
    main()