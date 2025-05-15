import os
import cv2
import numpy as np
from torch.utils.data import Dataset

class RoboticArmDataset(Dataset):
    def __init__(self, camera_dir, render_dir, prompt=None, img_size=512):
        """
        Dataset for ControlNet training with robotic arm images.
        
        Args:
            camera_dir: Directory containing real camera images
            render_dir: Directory containing rendered images
            prompt: Text prompt for conditioning
            img_size: Size to resize images to
        """
        self.camera_dir = camera_dir
        self.render_dir = render_dir
        self.img_size = img_size
        self.prompt = prompt 
        
        self.data = [f for f in os.listdir(camera_dir)
                    if f.lower().endswith(('.png', '.jpg'))]
    
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        img_name = self.data[idx]
        
        source = cv2.imread(os.path.join(self.render_dir, img_name))
        target = cv2.imread(os.path.join(self.camera_dir, img_name))
        source = cv2.resize(source, (self.img_size, self.img_size), interpolation=cv2.INTER_AREA)
        target = cv2.resize(target, (self.img_size, self.img_size), interpolation=cv2.INTER_AREA)
        source = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
        target = cv2.cvtColor(target, cv2.COLOR_BGR2RGB)
        
        source = source.astype(np.float32) / 255.0
        target = (target.astype(np.float32) / 127.5) - 1.0
        
        return dict(jpg=target, txt=self.prompt, hint=source)