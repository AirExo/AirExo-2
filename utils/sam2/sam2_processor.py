# sam2_processor.py
import torch
import numpy as np
import os
from PIL import Image
import matplotlib.pyplot as plt
from tqdm import tqdm
import functools
import cv2
from typing import Dict, List, Tuple, Optional, Any
import sys
sys.path.append('/home/jingjing/workspace/sam2')
from sam2.build_sam import build_sam2_video_predictor
from utils.image_processing import ImageProcessor
from utils.visualization import Visualizer

class Sam2:
    def __init__(self, config):
        self.config = config
        self.setup_attributes()
        self.setup_processing()
        self.setup_model()

    def setup_attributes(self):
        self.scene_name = self.config.scene_name
        self.base_dir = self.config.get('model', 'base_dir')
        self.tmp_dir = self.config.get('data', 'tmp_dir')
        self.input_data_root = self.config.get('data', 'input_data_root')
        self.camera_id = self.config.get('data', 'camera_id')

        self.visualize_dir = self.config.get('data', 'visualize_dir')
        self.visualize_name = self.config.get('data', 'visualize_name')

        self.start_frame = self.config.get('processing', 'start_frame')
        self.end_frame = self.config.get('processing', 'end_frame')
        self.interval = self.config.get('processing', 'interval')
        
        # Initialize annotation attributes
        self.points = []
        self.labels = []
        self.prompts = {}
        self.is_accepting_clicks = True
        self.prefix = self.config.get('data', 'prefix')

    def setup_processing(self):
        input_folder = os.path.join(self.input_data_root, self.scene_name, self.prefix)
        jpg_folder = os.path.join(self.tmp_dir, "image_jpg", self.scene_name)
        
        self.image_processor = ImageProcessor(input_folder, jpg_folder)
        self.visualizer = Visualizer(self.tmp_dir, self.scene_name, self.start_frame, self.visualize_dir, self.visualize_name)
        
        if self.config.get('processing', 'convert_png_to_jpg'):
            print("Converting PNG to JPG...")
            self.image_processor.convert_png_to_jpg(self.start_frame, self.end_frame)

    def get_jpg_path(self):
        return os.path.join(self.input_data_root, self.scene_name, self.prefix)

    def get_mask_path(self):
        pass

    def setup_model(self):
        self.device = self._setup_device()
        self._init_sam2_model()

    def _setup_device(self) -> torch.device:
        if torch.cuda.is_available():
            device = torch.device("cuda")
            self._configure_cuda()
        elif torch.backends.mps.is_available():
            device = torch.device("mps")
            print("\nSupport for MPS devices is preliminary. SAM 2 might give different outputs and degraded performance.")
        else:
            device = torch.device("cpu")
        print(f"Using device: {device}")
        return device

    def _configure_cuda(self):
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

    def _init_sam2_model(self):
        sam2_checkpoint =self.config.get('model', 'checkpoint')
        model_cfg = self.config.get('model', 'config')
        
        self.predictor = build_sam2_video_predictor(model_cfg, sam2_checkpoint, device=self.device)
        
        self.video_dir = os.path.join(self.tmp_dir, "image_jpg", self.scene_name)
        self.frame_names = sorted(
            [p for p in os.listdir(self.video_dir) if p.lower().endswith(('.jpg', '.jpeg'))],
            key=lambda p: int(os.path.splitext(p)[0])
        )
        
        self.inference_state = self.predictor.init_state(video_path=self.video_dir)
        self.predictor.reset_state(self.inference_state)

    def init_model(self, ann_frame_idx: int = 0):
        self.points = []
        self.labels = []
        self.prompts = {}

        # Setup display
        plt.figure(figsize=(9, 6))
        plt.title(f"frame {ann_frame_idx+self.start_frame}")
        plt.imshow(Image.open(os.path.join(self.video_dir, self.frame_names[ann_frame_idx])))
        
        ann_obj_id = 1
        plt.gca().set_title("Click to add points and labels")
        onclick_callback = functools.partial(self.onclick, ann_frame_idx=ann_frame_idx, ann_obj_id=ann_obj_id)
        onkey_callback = functools.partial(self.onkey, ann_frame_idx=ann_frame_idx, ann_obj_id=ann_obj_id)

        canvas = plt.gcf().canvas
        canvas.mpl_connect('button_press_event', onclick_callback)
        canvas.mpl_connect('key_press_event', onkey_callback)
        plt.show()

    def onclick(self, event, ann_frame_idx: int, ann_obj_id: int):
        if not self.is_accepting_clicks:
            return
        x, y = int(event.xdata), int(event.ydata)
        # print(x)
        # print(y)
        # x = 2*x + 207 
        # y = 720-2*(720-y) -160
        self.is_accepting_clicks = False
        
        # Left click (button 1) = label 1, right click (button 3) = label 0
        if event.button in [1, 3]:
            label = 1 if event.button == 1 else 0
            self.points.append([x, y])
            self.labels.append(label)
            print(f"Added point ({x}, {y}) with label {label}")
        
        self.prompts[ann_obj_id] = np.array(self.points), np.array(self.labels)

        _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
            inference_state=self.inference_state,
            frame_idx=ann_frame_idx,
            obj_id=ann_obj_id,
            points=np.array(self.points),
            labels=np.array(self.labels)
        )

        images = plt.gca().images
        if len(images) > 1:
            for img in images[1:]:
                img.remove()

        for i, out_obj_id in enumerate(out_obj_ids):
            self.visualizer.show_points(*self.prompts[out_obj_id], plt.gca())
            self.visualizer.show_mask(
                (out_mask_logits[i] > 0.0).cpu().numpy(), 
                plt.gca(), 
                obj_id=out_obj_id
            )

        self.is_accepting_clicks = True

    def onkey(self, event, ann_frame_idx: int, ann_obj_id: int):
        if event.key == 'z': 
            if len(self.points) > 0:
                removed_point = self.points.pop()
                removed_label = self.labels.pop()
                print(f"Undo: Removed point {removed_point} with label {removed_label}")
                
                if len(self.points) > 0:  
                    self.prompts[ann_obj_id] = np.array(self.points), np.array(self.labels)
                    _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                        inference_state=self.inference_state,
                        frame_idx=ann_frame_idx,
                        obj_id=ann_obj_id,
                        points=np.array(self.points),
                        labels=np.array(self.labels)
                    )

                    ax = plt.gca()
                    images = ax.images
                    if len(images) > 1:
                        for img in images[1:]:
                            img.remove()
                    
                    for collection in ax.collections:
                        collection.remove()

                    for i, out_obj_id in enumerate(out_obj_ids):
                        self.visualizer.show_points(*self.prompts[out_obj_id], ax)
                        self.visualizer.show_mask(
                            (out_mask_logits[i] > 0.0).cpu().numpy(),
                            ax,
                            obj_id=out_obj_id
                        )
                else: 
                    ax = plt.gca()
                    images = ax.images
                    if len(images) > 1:
                        for img in images[1:]:
                            img.remove()
                    for collection in ax.collections:
                        collection.remove()
                    plt.draw()

    def _process_frame(self, frame_idx: int, segments: Dict[int, np.ndarray], 
                      camera_params: Optional[Dict] = None, tcp_data: Optional[Dict] = None):
        color_path = os.path.join(
            self.input_data_root, 
            self.scene_name, 
            self.prefix, 
            self.frame_names[frame_idx].replace(".jpg", ".png")
        )
        
        plt.close("all")
        fig = plt.figure(figsize=(12, 8))
        plt.title(f"frame {frame_idx+self.start_frame}")
        plt.imshow(Image.open(color_path))
        
        for obj_id, mask in segments.items():
            # eroded_mask = self.image_processor.process_mask(mask)
            eroded_mask = mask
            self.visualizer.show_mask(eroded_mask, plt.gca(), frame_idx=frame_idx, obj_id=obj_id)
        
        plt.close()

    def _get_retry_frame(self, start: int, end: int) -> int:
        while True:
            try:
                frame = int(input(f"Enter the frame number ({start}-{end-1}) to restart from: "))
                if start <= frame < end:
                    return frame
                print(f"Please enter a number between {start} and {end-1}")
            except ValueError:
                print("Please enter a valid number")

    def generalize(self):
        while True:
            need_restart, retry_frame = self._process_frames_restart()
            if need_restart:
                self._restart_from_frame(retry_frame)
            else:
                break

    def _process_frames_restart(self) -> Tuple[bool, Optional[int]]:
        """Process frames with restart capability."""
        video_segments = self._propagate_segmentation()
        
        for frame_idx in tqdm(range(0, len(self.frame_names))):
            self.visualizer.save_mask_files(frame_idx, self.frame_names, video_segments)
            # self.visualizer.save_erode_mask_files(frame_idx, self.frame_names, video_segments)
            self.visualizer.save_combined_filed(frame_idx, self.frame_names, video_segments, os.path.join(self.input_data_root, self.scene_name, self.prefix))
            
            # if (frame_idx + 1) % self.interval == 0 or frame_idx + 1 == len(self.frame_names):
            if (frame_idx + 1) % self.interval == 0 :
                start_idx = frame_idx - (self.interval - 1)
                self.visualizer.visualize_frame_range(
                    start_idx + self.start_frame, 
                    frame_idx + self.start_frame,
                    self.interval
                )
                
                while True:
                    response = input(f"Review frames {start_idx+self.start_frame}-{frame_idx+self.start_frame}. " 
                                  f"Press Enter if OK, 'n' if there's an issue: ")
                    if response.lower() == 'n':
                        retry_frame = self._get_retry_frame(
                            start_idx + self.start_frame, 
                            frame_idx + self.start_frame + 1
                        )
                        return True, retry_frame
                    elif response == '':
                        break
                    print("Please press Enter to continue or 'n' to mark an issue.")
                plt.close()
            

        return False, None

    def _restart_from_frame(self, retry_frame: int):
        print(f"Restarting from frame {retry_frame}")
        self.start_frame = retry_frame
        self.setup_processing()
        # self.image_processor.convert_png_to_jpg(self.start_frame)
        self._init_sam2_model()
        print("Starting annotation...")
        self.init_model(ann_frame_idx=0)

    def _propagate_segmentation(self) -> Dict[int, Dict[int, np.ndarray]]:
        return {
            out_frame_idx: {
                out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
                for i, out_obj_id in enumerate(out_obj_ids)
            }
            for out_frame_idx, out_obj_ids, out_mask_logits 
            in self.predictor.propagate_in_video(self.inference_state)
        }