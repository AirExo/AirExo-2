"""
Calibration Script.

Usage: 
  - For robot: 
      python -m airexo.calibration.main --config-name=robot
  - For AirExo:
      python -m airexo.calibration.main --config-name=airexo
  - For AirExo annotation:
      python -m airexo.calibration.main --config-name=airexo_annotator_cam
      python -m airexo.calibration.main --config-name=airexo_annotator_data
  - For AirExo (finetuning via differentiable rendering):
      python -m airexo.calibration.main --config-name=airexo_solver_diff_ren
    
Authors: Hongjie Fang.
"""

import os
import hydra

from omegaconf import OmegaConf

from airexo.helpers.logger import setup_loggers


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "calibration")
)
def main(cfg):
    setup_loggers()
    OmegaConf.resolve(cfg)  
    if cfg.type == "calibrator":  
        calibrator = hydra.utils.instantiate(cfg.calibrator)
        calibrator.calibrate()
        calibrator.stop()
    elif cfg.type == "solver":
        solver = hydra.utils.instantiate(cfg.solver)
        solver.solve(**cfg.solver_params)
    elif cfg.type == "annotator":
        annotator = hydra.utils.instantiate(cfg.annotator)
        annotator.run(**cfg.annotator_params)
        annotator.stop()
    else:
        raise AttributeError("Invalid type: {}.".format(cfg.type))

if __name__ == '__main__':
    main()