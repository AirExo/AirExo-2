"""
Depth Adaptor.

Authors: Jingjing Chen.
"""

import os
import cv2
import sys
import glob
import hydra
import numpy as np
from PIL import Image
from tqdm import tqdm
from pathlib import Path
from omegaconf import OmegaConf
from hydra.utils import to_absolute_path


def visualize_depth(depth_map, save_path):
    depth_map = np.array(depth_map)
    depth_map = depth_map[:, 80:1200]
    depth_min = depth_map.min()
    depth_max = depth_map.max()
    depth_normalized = ((depth_map - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(save_path, depth_colored)
    return depth_colored

def load_depth_base(cfg):
    scene_path = os.path.join(to_absolute_path(cfg.paths.base_path), cfg.paths.scene_name)
    empty_depth_dir = to_absolute_path(cfg.paths.empty_depth_dir)
    
    if empty_depth_dir is None or not os.path.exists(empty_depth_dir):
        raise FileNotFoundError(f"Not found {empty_depth_dir}")
    
    depth_files = sorted(glob.glob(os.path.join(empty_depth_dir, '*.png')))
    if not depth_files:
        raise FileNotFoundError(f"No depth images found in {empty_depth_dir}")
    
    empty_depth = np.array(Image.open(depth_files[0]))
    print(f"Using empty depth from: {depth_files[0]}")
    
    mask_dir = os.path.join(scene_path, f"{cfg.paths.camera_id}/{cfg.dirs.sam_base_mask}")
    if os.path.exists(mask_dir):
        mask_files = sorted(os.listdir(mask_dir))
        min_mask_file = mask_files[0]
        mask_path = os.path.join(mask_dir, min_mask_file)
        mask = np.array(Image.open(mask_path), dtype=np.uint8)
        mask_binary = mask > 0
        
        depth_dir = os.path.join(scene_path, f"{cfg.paths.camera_id}/{cfg.dirs.depth}")
        depth_file = os.path.join(depth_dir, min_mask_file)
        depth = np.array(Image.open(depth_file))
        
        depth_meters = np.where(mask_binary, depth, empty_depth)
    else:
        depth_meters = empty_depth
    
    if cfg.visualization.viz_empty:
        viz_path = to_absolute_path(cfg.visualization.viz_path)
        print(f"Visualizing empty background depth to {viz_path}")
        visualize_depth(depth_meters, viz_path)
    
    return depth_meters

def validate_scene_path(cfg):
    scene_name = cfg.paths.scene_name
    scene_path = os.path.join(to_absolute_path(cfg.paths.base_path), scene_name)
    camera_id = cfg.paths.camera_id
    
    if not os.path.exists(scene_path):
        print(f"Scene {scene_name} does not exist at {scene_path}")
        sys.exit(1)
    
    required_paths = [
        os.path.join(scene_path, f"{camera_id}/{cfg.dirs.sam_mask}"),
        os.path.join(scene_path, f"{camera_id}/{cfg.dirs.depth}"),
        os.path.join(scene_path, f"{camera_id}/{cfg.dirs.render_robot}")
    ]
    
    for path in required_paths:
        if not os.path.exists(path):
            print(f"Required directory {path} does not exist")
            sys.exit(1)
    
    timestamps_dir = Path(scene_path) / f"{camera_id}/{cfg.dirs.color}"
    if not timestamps_dir.exists():
        print(f"Color directory missing in {scene_name}")
        sys.exit(1)
    
    return timestamps_dir, scene_path

def process_depth_frame(timestamp, paths, depth_meters, use_airexo_mask):
    hand_mask = np.array(Image.open(os.path.join(paths['sam_mask'], f"{timestamp}.png")))
    depth_image = np.array(Image.open(os.path.join(paths['depth'], f"{timestamp}.png")))
    depth_robot = np.array(Image.open(os.path.join(paths['render_robot_depth'], f"{timestamp}.png")))
    robot_mask = np.array(Image.open(os.path.join(paths['render_robot_mask'], f"{timestamp}.png")))
    
    depth_image_inpainted = depth_image.copy()
    
    base_mask_indices = hand_mask == 255
    depth_image_inpainted[base_mask_indices] = depth_meters[base_mask_indices]
    
    if use_airexo_mask and os.path.exists(paths['airexo_mask']):
        airexo_mask_path = os.path.join(paths['airexo_mask'], f"{timestamp}.png")
        airexo_depth_path = os.path.join(paths['airexo_depth'], f"{timestamp}.png")
        
        if os.path.exists(airexo_mask_path) and os.path.exists(airexo_depth_path):
            airexo_mask = np.array(Image.open(airexo_mask_path))
            airexo_depth = np.array(Image.open(airexo_depth_path))
            
            airexo_mask_indices = airexo_mask == 255
            replace_mask = airexo_mask_indices & (depth_image_inpainted >= airexo_depth)
            depth_image_inpainted[replace_mask] = depth_meters[replace_mask]
    
    robot_mask_indices = robot_mask == 255
    depth_image_inpainted[robot_mask_indices] = depth_robot[robot_mask_indices]
    
    depth_mm = depth_image_inpainted.astype(np.uint16)
    depth_img = Image.fromarray(depth_mm)
    depth_output_path = os.path.join(paths['depth_vis'], f"{timestamp}.png")
    depth_img.save(depth_output_path, format='PNG')
    
    return True


def process_scene(cfg):
    print(f"Processing scene: {cfg.paths.scene_name}...")
    
    timestamps_dir, scene_path = validate_scene_path(cfg)
    
    camera_id = cfg.paths.camera_id
    depth_vis_path = os.path.join(scene_path, f"{camera_id}/{cfg.dirs.output}")
    os.makedirs(depth_vis_path, exist_ok=True)
    timestamps = sorted([int(timestamp.stem) for timestamp in timestamps_dir.glob("*.png")])
    print(f"Processing {len(timestamps)} frames...")
    
    depth_meters = load_depth_base(cfg)
    
    paths = {
        'sam_mask': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.sam_mask}"),
        'depth': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.depth}"),
        'depth_vis': depth_vis_path,
        'render_robot_depth': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.render_robot}/depth"),
        'render_robot_mask': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.render_robot}/mask"),
        'airexo_mask': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.render_airexo}/mask"),
        'airexo_depth': os.path.join(scene_path, f"{camera_id}/{cfg.dirs.render_airexo}/depth")
    }
    
    successful_frames = 0
    for timestamp in tqdm(timestamps):
        if process_depth_frame(timestamp, paths, depth_meters, cfg.processing.use_airexo_mask):
            successful_frames += 1
    
    print(f"Successfully processed {successful_frames}/{len(timestamps)} frames")
    return successful_frames, len(timestamps)


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "adaptor"),
    config_name = "depth_adaptor.yaml"
)
def main(cfg):
    OmegaConf.resolve(cfg)  
    process_scene(cfg)

if __name__ == "__main__":
    main()