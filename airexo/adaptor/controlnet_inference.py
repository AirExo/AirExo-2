"""
ControlNet Inference.

Authors: [Authors of ControlNet], Jingjing Chen.
"""

import os
import cv2
import sys
import torch
import hydra
import numpy as np
from PIL import Image
from tqdm import tqdm
from pathlib import Path
from omegaconf import DictConfig, OmegaConf

# Import ControlNet related modules
sys.path.append(os.path.join(os.getcwd(), "dependencies", "ControlNet"))
from cldm.cldm import ControlLDM
from ldm.models.diffusion.ddim import DDIMSampler

class ControlNetInference:
    def __init__(self, config_path, model_path, device="cuda"):
        self.device = device
        self.config = OmegaConf.load(config_path)
        self.model = self.load_model(model_path)
        self.sampler = DDIMSampler(self.model)

    def load_model(self, path):
        # Update config to use local tokenizer path if available
        if "cond_stage_config" in self.config.model.params:
            if "params" in self.config.model.params.cond_stage_config:
                if "version" in self.config.model.params.cond_stage_config.params:
                    # If you have local tokenizer files, specify the path here
                    local_tokenizer_path = "/path/to/your/clip/tokenizer"
                    if os.path.exists(local_tokenizer_path):
                        self.config.model.params.cond_stage_config.params.version = local_tokenizer_path

        model = ControlLDM(**self.config.model.params)
        ckpt = torch.load(path, map_location="cpu")
        model.load_state_dict(ckpt["state_dict"])
        model = model.to(self.device)
        model.eval()
        return model

    def batch_preprocess_images(self, images, image_size=512):
        processed_images = []
        original_sizes = []
        
        for image in images:
            if isinstance(image, np.ndarray):
                original_sizes.append(image.shape[:2])
                image = Image.fromarray(image.astype('uint8'))
            else:
                original_sizes.append(image.size[::-1])
            
            image = image.resize((image_size, image_size))
            image = np.array(image).astype(np.float32) / 255.0
            image = image[None].transpose(0, 3, 1, 2)
            processed_images.append(torch.from_numpy(image))
        
        self.original_sizes = original_sizes
        batch_images = torch.cat(processed_images, dim=0).to(self.device)
        return batch_images

    def batch_inference(
        self,
        control_images,
        prompts,
        num_samples=1,
        image_size=512,
        ddim_steps=50,
        scale=9.0,
        seed=None,
        batch_size=4
    ):
        if seed is not None:
            torch.manual_seed(seed)
            
        all_samples = []
        for i in range(0, len(control_images), batch_size):
            batch_images = control_images[i:i + batch_size]
            batch_prompts = prompts[i:i + batch_size]
            
            batch_control = self.batch_preprocess_images(batch_images, image_size)
            
            cond = {
                "c_concat": [batch_control],
                "c_crossattn": [self.model.get_learned_conditioning(batch_prompts * num_samples)]
            }
            un_cond = {
                "c_concat": [batch_control],
                "c_crossattn": [self.model.get_learned_conditioning([""] * (len(batch_prompts) * num_samples))]
            }
            
            shape = (4, image_size // 8, image_size // 8)
            samples, _ = self.sampler.sample(
                ddim_steps,
                len(batch_prompts) * num_samples,
                shape,
                cond,
                verbose=False,
                unconditional_guidance_scale=scale,
                unconditional_conditioning=un_cond,
            )
            
            x_samples = self.model.decode_first_stage(samples)
            x_samples = torch.clamp((x_samples + 1.0) / 2.0, min=0.0, max=1.0)
            all_samples.append(x_samples)
            
        return torch.cat(all_samples, dim=0)

    def batch_save_images(self, tensor, output_paths, masks, output_images):
        images = tensor.cpu().numpy().transpose(0, 2, 3, 1) * 255
        images = images.round().astype(np.uint8)
        
        for idx, (image, output_path, mask, output_image) in enumerate(zip(images, output_paths, masks, output_images)):
            if hasattr(self, 'original_sizes'):
                pil_image = Image.fromarray(image).resize((self.original_sizes[idx][1], self.original_sizes[idx][0]))
                image = np.array(pil_image)
            
            if mask is not None and image is not None:
                if len(mask.shape) == 2:
                    mask = mask[..., None]
                mask = mask.astype(bool)
                
                output_image[mask.squeeze()] = image[mask.squeeze()]
                Image.fromarray(output_image).save(f"{output_path}.png")
            else:
                Image.fromarray(image).save(f"{output_path}.png")


def visualize_depth(depth_map, save_path, is_inverse=False):
    depth_map = depth_map[:,80:1200]
    depth_min = depth_map.min()
    depth_max = depth_map.max()
    depth_normalized = ((depth_map - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(save_path, depth_colored)
    return depth_colored


def process_scene(cfg: DictConfig) -> None:
    base_path = hydra.utils.to_absolute_path(cfg.paths.base_path)
    scene_name = cfg.paths.scene_name
    scene_path = os.path.join(base_path, scene_name)
    camera_id = cfg.paths.camera_id
    camera_path = os.path.join(scene_path, camera_id)
    
    render_path = os.path.join(camera_path, cfg.dirs.render_robot)
    depth_path = os.path.join(camera_path, cfg.dirs.depth)
    color_path = os.path.join(camera_path, cfg.dirs.color)
    output_path = os.path.join(camera_path, cfg.dirs.output)
    os.makedirs(output_path, exist_ok=True)
    
    controlnet_config_path = hydra.utils.to_absolute_path(cfg.model.config_path)
    model_path = hydra.utils.to_absolute_path(cfg.model.checkpoint_path)
    inferencer = ControlNetInference(controlnet_config_path, model_path, device=cfg.model.device)
    
    timestamps_dir = Path(color_path)
    timestamps = sorted([int(timestamp.stem) for timestamp in timestamps_dir.glob("*.png")])
    print(f"Processing {len(timestamps)} frames in {scene_name}...")
    
    batch_images = []
    batch_masks = []
    batch_output_images = []
    batch_timestamps = []
    batch_output_paths = []
    batch_depth_robots = []
    
    for i, timestamp in enumerate(tqdm(timestamps)):
        depth_image = np.array(Image.open(os.path.join(depth_path, f"{timestamp}.png")))
        image_robot = np.array(Image.open(os.path.join(render_path, f"color/{timestamp}.png")))
        depth_robot = np.array(Image.open(os.path.join(render_path, f"depth/{timestamp}.png")))
        
        batch_images.append(image_robot)
        batch_masks.append(None)
        batch_output_images.append(None)
        batch_timestamps.append(timestamp)
        batch_output_paths.append(os.path.join(output_path, f"{timestamp}"))
        batch_depth_robots.append(depth_robot)
        
        if len(batch_images) >= cfg.inference.batch_size or i == len(timestamps) - 1:
            batch_prompts = [cfg.inference.prompt] * len(batch_images)
            samples = inferencer.batch_inference(
                batch_images,
                batch_prompts,
                cfg.inference.num_samples,
                image_size=cfg.inference.image_size,
                ddim_steps=cfg.inference.steps,
                scale=cfg.inference.scale,
                seed=cfg.inference.seed,
                batch_size=cfg.inference.batch_size
            )
            
            inferencer.batch_save_images(samples, batch_output_paths, batch_masks, batch_output_images)
            
            batch_images = []
            batch_masks = []
            batch_output_images = []
            batch_timestamps = []
            batch_output_paths = []
            batch_depth_robots = []
    
    print(f"Processing complete. Results saved to {output_path}")



@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "adaptor"),
    config_name = "controlnet_inference.yaml"
)
def main(cfg):
    OmegaConf.resolve(cfg)  
    process_scene(cfg)


if __name__ == "__main__":
    main()