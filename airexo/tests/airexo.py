import os
import time
import hydra
from omegaconf import OmegaConf


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests"),
    config_name = "airexo"
)
def main(cfg):
    OmegaConf.resolve(cfg)
    
    airexo = hydra.utils.instantiate(cfg.airexo)

    while True:
        print(airexo.get_states())
    
if __name__ == '__main__':
    main()
