import os
import time
import hydra
from omegaconf import OmegaConf


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests", "teleop")
)
def main(cfg):
    OmegaConf.resolve(cfg)
    
    controller = hydra.utils.instantiate(cfg)

    controller.initialize()
    time.sleep(2)
    controller.start()
    time.sleep(2000000)
    controller.stop()
    time.sleep(2)
    controller.init_robot()


if __name__ == '__main__':
    main()
