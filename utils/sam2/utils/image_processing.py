import os
from PIL import Image
from tqdm import tqdm
from typing import List, Optional
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self, input_folder: str, output_folder: str):
        self.input_folder = input_folder
        self.output_folder = output_folder
        os.makedirs(output_folder, exist_ok=True)

    def convert_png_to_jpg(self, start_frame: int, end_frame: int) -> None:
        """Convert PNG images to JPG format starting from specified frame."""
        self._clear_output_directory()
        png_files = self._get_sorted_png_files(start_frame, end_frame)
        self._process_png_files(png_files, start_frame)

    def _clear_output_directory(self) -> None:
        if os.path.exists(self.output_folder):
            print(f"Clearing existing files in {self.output_folder}")
            for file in os.listdir(self.output_folder):
                file_path = os.path.join(self.output_folder, file)
                try:
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                except Exception as e:
                    print(f"Error deleting {file_path}: {e}")

    def _get_sorted_png_files(self, start_frame: int, end_frame: int) -> List[str]:
        all_png_files = [f for f in os.listdir(self.input_folder) if f.endswith(".png")]
        all_png_files.sort(key=lambda x: int(os.path.splitext(x)[0]))
        
        total_frames = len(all_png_files)
        if start_frame >= total_frames:
            raise ValueError(f"Start frame {start_frame} is greater than total frames {total_frames}")
        
        return all_png_files[start_frame:end_frame]

    def _process_png_files(self, png_files: List[str], start_frame: int) -> None:
        print(f"Found {len(png_files)} PNG files to convert, starting from frame {start_frame}")
        
        for png_file in tqdm(png_files, desc="Converting PNG to JPG"):
            self._convert_single_file(png_file)

    def _convert_single_file(self, png_file: str) -> None:
        png_path = os.path.join(self.input_folder, png_file)
        jpg_file = png_file.replace(".png", ".jpg")
        jpg_path = os.path.join(self.output_folder, jpg_file)
        
        with Image.open(png_path) as png_image:
            rgb_image = png_image.convert("RGB")
            rgb_image.save(jpg_path, quality=100)

    @staticmethod
    def process_mask(mask: np.ndarray, kernel_size: int = 3) -> np.ndarray:
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