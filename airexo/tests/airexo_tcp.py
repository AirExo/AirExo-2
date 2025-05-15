import os
import time
import hydra
from omegaconf import OmegaConf

from airexo.helpers.state import airexo_transform_tcp


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests"),
    config_name = "airexo_tcp"
)
def main(cfg):
    OmegaConf.resolve(cfg)
    
    left_airexo = hydra.utils.instantiate(cfg.left_airexo)
    right_airexo = hydra.utils.instantiate(cfg.right_airexo)

    while True:
        left_tcp, right_tcp = airexo_transform_tcp(
            left_joint = left_airexo.get_info(),
            right_joint = right_airexo.get_info(),
            left_joint_cfgs = left_airexo.joint_cfgs,
            right_joint_cfgs = right_airexo.joint_cfgs,
            left_calib_cfgs = cfg.left_calib_cfgs,
            right_calib_cfgs = cfg.right_calib_cfgs,
            is_rad = False,
            urdf_file = cfg.urdf_file,
            real_robot_base = True
        )
        print("left:", left_tcp, "\n right:", right_tcp)

if __name__ == '__main__':
    main()
