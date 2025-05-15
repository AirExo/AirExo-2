import os
import cv2
import time
import hydra
import numpy as np

from omegaconf import OmegaConf

from airexo.helpers.logger import setup_loggers
from airexo.helpers.shared_memory import SharedMemoryManager


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "cameras")
)
def main(cfg):
    setup_loggers()

    OmegaConf.resolve(cfg)

    path = os.path.join(cfg.save_path, "cam_{}".format(cfg.serial))
    color_path = os.path.join(path, "color")
    depth_path = os.path.join(path, "depth")
    os.makedirs(path, exist_ok = True)
    os.makedirs(color_path, exist_ok = True)
    os.makedirs(depth_path, exist_ok = True)   
    interval = 1.0 / cfg.save_freq

    camera = hydra.utils.instantiate(cfg)
    camera.logger.info("Start collecting.")

    init = np.array([False, False], dtype = np.bool_)
    shm_receiver = SharedMemoryManager("record_flag", 1, shape = init.shape, dtype = init.dtype)

    while True:
        tic = time.time()
        timestamp, color, depth = camera.get_rgbd_images()
        cv2.imwrite(os.path.join(color_path, "{}.png".format(int(timestamp))), color[:, :, ::-1])
        cv2.imwrite(os.path.join(depth_path, "{}.png".format(int(timestamp))), depth)
        has_stop = shm_receiver.execute()[-1]
        if has_stop:
            break
        duration = time.time() - tic
        if duration < interval:
            time.sleep(interval - duration)

    shm_receiver.close()
    camera.logger.info("Stop.")
    camera.stop()


if __name__ == '__main__':
    main()