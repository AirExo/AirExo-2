"""
Visualization.
"""

import os
import hydra

from omegaconf import OmegaConf


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests", "vis")
)
def main(cfg):
    vis = hydra.utils.instantiate(cfg)
    vis.run()
    vis.stop()


if __name__ == '__main__':
    main()