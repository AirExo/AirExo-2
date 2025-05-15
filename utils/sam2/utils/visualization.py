import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, List, Tuple, Optional
import cv2
import os
from tqdm import tqdm
import glob

class Visualizer:
    def __init__(self, tmp_dir: str, scene_name: str, start_frame: int, visualize_dir: str, visualize_name: str):
        self.tmp_dir = tmp_dir
        self.scene_name = scene_name
        self.start_frame = start_frame
        self.visualize_dir = visualize_dir
        self.visualize_name = visualize_name
        self.setup_directories()

    def setup_directories(self):
        self.mask_dir = os.path.join(self.tmp_dir, "masks", self.scene_name)
        self.seg_result_dir = os.path.join(self.tmp_dir, "seg_result", self.scene_name)
        
        os.makedirs(self.mask_dir, exist_ok=True)
        os.makedirs(self.seg_result_dir, exist_ok=True)

    def show_points(self, coords: np.ndarray, labels: np.ndarray, ax, marker_size: int = 200) -> None:
        pos_points = coords[labels==1]
        neg_points = coords[labels==0]
        
        ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', 
                  s=marker_size, edgecolor='white', linewidth=1.25)
        ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', 
                  s=marker_size, edgecolor='white', linewidth=1.25)

    def show_mask(self, mask: np.ndarray, ax, frame_idx: Optional[int] = None,
                 obj_id: Optional[int] = None, random_color: bool = False) -> None:          
        mask_save_dir = os.path.join(self.tmp_dir, "masks", self.scene_name)
        os.makedirs(mask_save_dir, exist_ok=True)
        
        mask_uint8 = mask.astype(np.uint8) * 255
        
        if len(mask_uint8.shape) == 3:
            mask_uint8 = mask_uint8[0]  
        if frame_idx is not None:    
            mask_filename = f"mask_{frame_idx+self.start_frame}.png"
        else:
            mask_filename = f"test.png"
        cv2.imwrite(os.path.join(mask_save_dir, mask_filename), mask_uint8)

        color = (np.concatenate([np.random.random(3), np.array([0.6])], axis=0) if random_color 
                else np.array([*plt.get_cmap("tab10")(0 if obj_id is None else obj_id)[:3], 0.6]))
        
        h, w = mask.shape[-2:]
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        ax.imshow(mask_image, alpha=0.6)
        plt.draw()

        if frame_idx is not None:
            fig = ax.figure
            save_dir = os.path.join(self.tmp_dir, "seg_result", self.scene_name)
            os.makedirs(save_dir, exist_ok=True)
            fig.savefig(os.path.join(save_dir, f"output_{frame_idx+self.start_frame}.png"), 
                    dpi=300, bbox_inches='tight')

    def _save_mask(self, mask: np.ndarray, frame_idx: int) -> None:
        mask_uint8 = mask.astype(np.uint8) * 255
        if len(mask_uint8.shape) == 3:
            mask_uint8 = mask_uint8[0]
            
        mask_filename = f"mask_{frame_idx}.png"
        cv2.imwrite(os.path.join(self.mask_dir, mask_filename), mask_uint8)

    # def _get_mask_color(self, obj_id: Optional[int], random_color: bool) -> np.ndarray:
    #     if random_color:
    #         return np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    #     return np.array([*plt.get_cmap("tab10")(0 if obj_id is None else obj_id)[:3], 0.6])

    # def _display_mask(self, mask: np.ndarray, color: np.ndarray, ax) -> None:
    #     h, w = mask.shape[-2:]
    #     mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    #     ax.imshow(mask_image, alpha=0.6)
    #     plt.draw()

    # def _save_visualization(self, ax, frame_idx: int) -> None:
    #     fig = ax.figure
    #     fig.savefig(
    #         os.path.join(self.seg_result_dir, f"output_{frame_idx}.png"),
    #         dpi=300, 
    #         bbox_inches='tight'
    #     )

    def visualize_frame_range(self, start_idx: int, end_idx: int, interval: int) -> None:
        n_images = interval
        n_cols = int(np.ceil(np.sqrt(n_images)))
        n_rows = int(np.ceil(n_images / n_cols))

        fig_width = 4 * n_cols
        fig_height = 4 * n_rows

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(fig_width, fig_height))
        fig.suptitle(f'Frames {start_idx} to {end_idx}', fontsize=16)

        if n_rows == 1:
            axes = np.array([axes])
        if n_cols == 1:
            axes = np.array([axes]).T

        self._populate_grid(axes.flatten(), start_idx, end_idx, n_images)
        
        plt.tight_layout()
        plt.show()

    def _populate_grid(self, axes_flat: np.ndarray, start_idx: int, end_idx: int, n_images: int) -> None:
        for i in range(n_images):
            curr_idx = start_idx + i
            if curr_idx <= end_idx:
                pattern = os.path.join(self.seg_result_dir, f"masked_{curr_idx}*")
                matching_files = glob.glob(pattern)
                if matching_files:
                    img_path = matching_files[0]  # Get the first matching file
                    img = plt.imread(img_path)
                    axes_flat[i].imshow(img)
            axes_flat[i].axis('off')

    def process_mask(self, mask: np.ndarray, kernel_size: int = 6) -> np.ndarray:
        """Process mask with morphological operations."""
        if len(mask.shape) == 3:
            mask = mask[0]
            
        mask_uint8 = mask.astype(np.uint8) * 255
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        eroded_mask = cv2.erode(mask_uint8, kernel, iterations=1)
        eroded_mask = eroded_mask > 0
        
        if len(mask.shape) == 3:
            eroded_mask = eroded_mask[np.newaxis, ...]
        return eroded_mask


    def save_mask_files(self, frame_idx, frame_names, video_segments):
        self.mask_save_dir = os.path.join(self.visualize_dir, self.scene_name, self.visualize_name)

        os.makedirs(self.mask_save_dir, exist_ok=True)
        segments = video_segments[frame_idx]
        for obj_id, mask in segments.items():
            mask_uint8 = mask.astype(np.uint8) * 255
            
            if len(mask_uint8.shape) == 3:
                mask_uint8 = mask_uint8[0]  
            mask_filename = f"{frame_names[frame_idx].split('.')[0]}.png"
            cv2.imwrite(os.path.join(self.mask_save_dir, mask_filename), mask_uint8)



    def save_combined_filed(self, frame_idx, frame_names, video_segments, base_color_path):
        color_save_dir = os.path.join(self.tmp_dir, "seg_result", self.scene_name)
        os.makedirs(color_save_dir, exist_ok=True)

        segments = video_segments[frame_idx]
        color_path = os.path.join(base_color_path, frame_names[frame_idx].replace(".jpg", ".png"))
        img = cv2.imread(color_path)
        overlay = img.copy()
        for obj_id, mask in segments.items():
            # mask = self.process_mask(mask)
            if len(mask.shape) == 3:
                mask = mask[0]
        
            overlay[mask > 0] = cv2.addWeighted(
                overlay[mask > 0], 
                1.0,  
                np.full_like(overlay[mask > 0], [0, 0, 255]),  
                0.5,  
                0
            )
    
            timestamp = frame_names[frame_idx].split('.')[0]
            save_path = os.path.join(color_save_dir, f"masked_{frame_idx+self.start_frame}_{timestamp}.png")
            cv2.imwrite(save_path, overlay)


