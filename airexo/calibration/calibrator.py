"""
Calibrator.

Authors: Hongjie Fang.
"""

import os
import cv2
import time
import hydra
import logging
import numpy as np

from omegaconf import OmegaConf

from airexo.helpers.logger import ColoredLogger
from airexo.helpers.calibration import aruco_detector


class Calibrator:
    """
    ArUco calibrator with additional shared_memory information.
    """
    def __init__(
        self,
        calib_type,
        calib_path,
        camera_serials_global,
        camera_serial_inhand_left,
        camera_serial_inhand_right,
        device_left,
        device_right,
        aruco_dict = "DICT_7X7_250",
        aruco_idx = 0,
        marker_length = 150,
        vis = True,
        logger_name: str = "Calibrator",
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)
        self.calib_type = calib_type
        self.device_left = device_left
        self.device_right = device_right

        # Initialize calibration folder.
        self.calib_path = os.path.join(calib_path)
        if not os.path.exists(self.calib_path):
            os.makedirs(self.calib_path)

        # Initialize cameras.
        self.logger.info("Initialize cameras.")
        if self.calib_type == "robot":
            assert camera_serial_inhand_left is not None, "Inhand left camera serial is required for robot calibration."
            assert camera_serial_inhand_right is not None, "Inhand right camera serial is required for robot calibration."
            self.camera_serials = list(camera_serials_global + [camera_serial_inhand_left, camera_serial_inhand_right])
        else:
            self.camera_serials = list(camera_serials_global)
            if camera_serial_inhand_left is not None:
                self.camera_serials += [camera_serial_inhand_left]
            if camera_serial_inhand_right is not None:
                self.camera_serials += [camera_serial_inhand_right]

        self.camera_serials_global = list(camera_serials_global)
        self.camera_serial_inhand_left = camera_serial_inhand_left
        self.camera_serial_inhand_right = camera_serial_inhand_right
        self.cameras = []
        for i, serial in enumerate(self.camera_serials):
            camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(serial)))
            camera = hydra.utils.instantiate(camera_cfg)
            self.cameras.append(camera)

        # ArUco parameters
        self.aruco_dict = getattr(cv2.aruco, aruco_dict)
        self.aruco_idx = aruco_idx
        self.marker_length = marker_length
        self.vis = vis

    def get_camera_frames(self):
        res = {}
        for i, cam in enumerate(self.cameras):
            serial = self.camera_serials[i]
            _, img = cam.get_rgb_image()
            res[serial] = img
        return res

    def get_camera_intrinsics(self):
        res = {}
        for i, cam in enumerate(self.cameras):
            serial = self.camera_serials[i]
            intrinsics = cam.get_intrinsic(return_mat = True)
            res[serial] = intrinsics
        return res

    def calibrate(self):
        self.logger.info("Calibrate ... Please remain still.")
        # Get intrinsics
        intrinsics = self.get_camera_intrinsics()
        # Get camera frames
        cam_frames = self.get_camera_frames()
        # Get additional device info
        left_states = self.device_left.get_states()
        right_states = self.device_right.get_states()
        # Get timestamp
        timestamp = int(time.time() * 1000)
        # Process camera frames to extrinsics
        extrinsics = {}
        for i, serial in enumerate(self.camera_serials):
            extrinsics[serial] = aruco_detector(
                cam_frames[serial],
                aruco_dict = self.aruco_dict,
                marker_length = self.marker_length, 
                camera_intrinsic = intrinsics[serial], 
                vis = self.vis
            )[self.aruco_idx]
        # Full calibration statistics
        res = {
            "type": self.calib_type,
            "camera_serials": self.camera_serials,
            "camera_serials_global": self.camera_serials_global,
            "camera_serial_inhand_left": self.camera_serial_inhand_left,
            "camera_serial_inhand_right": self.camera_serial_inhand_right,
            "intrinsics": intrinsics,
            "extrinsics": extrinsics,
            "{}_left".format(self.calib_type): left_states,
            "{}_right".format(self.calib_type): right_states
        }
        # Save into files
        np.save(os.path.join(self.calib_path, "{}.npy".format(timestamp)), res, allow_pickle = True)
        self.logger.info("Finish calibration, results saved to {}.".format(os.path.join(self.calib_path, "{}.npy".format(timestamp))))

    def stop(self):
        for camera in self.cameras:
            camera.stop()
        self.device_left.stop()
        self.device_right.stop()
        