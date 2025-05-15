import os
import yaml
from typing import Optional, Dict, Any

class Config:
    def __init__(self, config_path: str = "config/default.yaml"):
        self.config_data = self._load_yaml(config_path)
        self._get_user_inputs()
        self._process_paths()
        self._validate_config()

    def _load_yaml(self, config_path: str) -> dict:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
            
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        return config

    def _get_user_inputs(self):
        self.config_data['processing']['convert_png_to_jpg'] = self._get_user_convert_choice()
        self.config_data['processing']['interval'] = self._get_user_interval_choice()
        self.config_data['processing']['start_frame'] = self._get_start_frame()
        self.config_data['processing']['end_frame'] = self._get_end_frame()


    def _process_paths(self):
        self.config_data['data']['tmp_dir'] = self.config_data['data']['tmp_dir'].replace(
            '${data.input_data_root}', 
            self.config_data['data']['input_data_root']
        )
        
        os.makedirs(self.config_data['data']['tmp_dir'], exist_ok=True)

    def _validate_config(self):
        required_paths = [
            self.config_data['data']['input_data_root']
        ]
        
        for path in required_paths:
            if not os.path.exists(path):
                raise FileNotFoundError(f"Required path does not exist: {path}")

    @staticmethod
    def _get_user_convert_choice() -> bool:
        """Get user input for PNG to JPG conversion."""
        return True
        while True:
            choice = input("Do you want to convert PNG to JPG? (yes/no): ").lower()
            if choice in ['yes', 'y']:
                return True
            elif choice in ['no', 'n']:
                return False
            print("Please enter 'yes' or 'no'")

    @staticmethod
    def _get_user_interval_choice() -> int:
        """Get user input for visualization interval."""
        return 1000
        while True:
            try:
                frame_num = int(input("Enter the interval number for visualization: "))
                if frame_num > 0:
                    return frame_num
                print("Please enter a positive number")
            except ValueError:
                print("Please enter a valid number")

    @staticmethod
    def _get_start_frame() -> int:
        return 0
        """Get user input for starting frame number."""
        while True:
            try:
                frame_num = int(input("Enter the starting frame number for conversion: "))
                if frame_num >= 0:
                    return frame_num
                print("Please enter a non-negative number")
            except ValueError:
                print("Please enter a valid number")
                
    @staticmethod
    def _get_end_frame() -> int:
        # return 1
        while True:
            try:
                frame_num = int(input("Enter the ending frame number for conversion: "))
                if frame_num >= 0:
                    return frame_num
                print("Please enter a non-negative number")
            except ValueError:
                print("Please enter a valid number")


    def get(self, section: str, key: str, default: Any = None) -> Any:
        """Safe getter for configuration values."""
        return self.config_data.get(section, {}).get(key, default)

    def __getattr__(self, name: str) -> Any:
        """Allow direct access to nested config values."""
        for section in self.config_data.values():
            if isinstance(section, dict) and name in section:
                return section[name]
        raise AttributeError(f"'Config' object has no attribute '{name}'")