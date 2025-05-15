"""
Image Adaptor.

Authors: Jingjing Chen.
"""

import os
import cv2
import hydra
import numpy as np

from glob import glob
from tqdm import tqdm
from omegaconf import OmegaConf
from hydra.utils import to_absolute_path

def process_scene(config):
    base_path = to_absolute_path(config.paths.base_path)
    scene_name = config.paths.scene_name
    camera_id = config.paths.camera_id
    
    print(f"Processing {scene_name}...")
    scene_path = os.path.join(base_path, scene_name, camera_id)
    
    inpainting_dir = os.path.join(scene_path, config.dirs.inpainting)
    controlnet_dir = os.path.join(scene_path, config.dirs.controlnet)
    mask_dir = os.path.join(scene_path, config.dirs.mask)
    output_dir = os.path.join(scene_path, config.dirs.output)
    
    if not all(os.path.exists(d) for d in [inpainting_dir, controlnet_dir, mask_dir]):
        print(f"Skip {scene_name}: Some directories do not exist")
        if config.verbose:
            print(f"Missing directories: ")
            for d, path in [
                ("Inpainting", inpainting_dir), 
                ("ControlNet", controlnet_dir), 
                ("Mask", mask_dir)
            ]:
                if not os.path.exists(path):
                    print(f"  - {d}: {path}")
        return False
    
    os.makedirs(output_dir, exist_ok=True)
    
    image_files = [os.path.basename(x) for x in glob(os.path.join(inpainting_dir, "*.png"))]
    if not image_files:
        print(f"No images found in {inpainting_dir}")
        return False
    
    for img_name in tqdm(image_files, desc=f"Processing images in {scene_name}"):
        inpaint_img = cv2.imread(os.path.join(inpainting_dir, img_name))
        control_img = cv2.imread(os.path.join(controlnet_dir, img_name))
        mask = cv2.imread(os.path.join(mask_dir, img_name), cv2.IMREAD_GRAYSCALE)
        
        if inpaint_img is None or control_img is None or mask is None:
            print(f"Warning: Failed to read image {img_name} in {scene_name}")
            continue
        
        inpaint_img[mask > 0] = control_img[mask > 0]
        cv2.imwrite(os.path.join(output_dir, img_name), inpaint_img)
    
    print(f"{scene_name} done")
    return True

@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "adaptor"),
    config_name = "image_adaptor.yaml"
)
def main(cfg):
    OmegaConf.resolve(cfg)  
    
    if process_scene(cfg):
        print(f"Successfully processed {cfg.paths.scene_name}")
    else:
        print(f"Failed to process {cfg.paths.scene_name}")


if __name__ == "__main__":
    main()