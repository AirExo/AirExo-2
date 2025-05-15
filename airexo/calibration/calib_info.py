"""
Calibration Information.

Authors: Hongjie Fang.
"""

import os
import numpy as np

from airexo.helpers.constants import *
from airexo.helpers.rotation import xyz_rot_to_mat, average_xyz_rot_quat


class CalibrationInfo:
    """
    Calibration Information.

    Features includes:
    - Calculate transformation from cameras to left/right/overall base;
    - Get camera intrinsics.
    """
    def __init__(
        self,
        calib_path,
        calib_timestamp,
        **kwargs
    ):
        self.calib_path = calib_path
        self.calib_timestamp = calib_timestamp
        self.calib_file_path = os.path.join(calib_path, "{}.npy".format(calib_timestamp))
        assert os.path.exists(self.calib_file_path), "Calibration file {} not exists.".format(self.calib_file_path)
        calib_file = np.load(self.calib_file_path, allow_pickle = True).item()
        self.calib_type = calib_file["type"]
        assert self.calib_type in ["robot", "airexo", "airexo_upd"], "Invalid calibration type: {}".format(self.calib_type)
        
        self.camera_serials = calib_file["camera_serials"]
        self.camera_serials_global = calib_file["camera_serials_global"]
        self.camera_serial_inhand_left = calib_file["camera_serial_inhand_left"]
        self.camera_serial_inhand_right = calib_file["camera_serial_inhand_right"]
        self.intrinsics = calib_file["intrinsics"]
        self.extrinsics = calib_file["extrinsics"]
        if self.calib_type[:6] == "airexo":
            self.airexo_left = calib_file["airexo_left"]
            self.airexo_right = calib_file["airexo_right"]
        else:
            self.robot_left = calib_file["robot_left"]
            self.robot_right = calib_file["robot_right"]
        if self.calib_type == "airexo_upd":
            self.upd_serial = calib_file["upd"]["camera_serial"]
            self.upd_camera_to_base = calib_file["upd"]["camera_to_base"]
        for serial in self.camera_serials:
            assert serial in self.intrinsics.keys(), "Cannot find camera {} in intrinsics.".format(serial)
            assert serial in self.extrinsics.keys(), "Cannot find camera {} in extrinsics.".format(serial)
    
    def to_dict(self):
        calib_dict = {
            "type": self.calib_type,
            "camera_serials": self.camera_serials,
            "camera_serials_global": self.camera_serials_global,
            "camera_serial_inhand_left": self.camera_serial_inhand_left,
            "camera_serial_inhand_right": self.camera_serial_inhand_right,
            "intrinsics": self.intrinsics,
            "extrinsics": self.extrinsics
        }
        if self.calib_type[:6] == "airexo":
            calib_dict["airexo_left"] = self.airexo_left
            calib_dict["airexo_right"] = self.airexo_right
        else:
            calib_dict["robot_left"] = self.robot_left
            calib_dict["robot_right"] = self.robot_right
        if self.calib_type == "airexo_upd":
            calib_dict["upd"] = {}
            calib_dict["upd"]["camera_serial"] = self.upd_serial 
            calib_dict["upd"]["camera_to_base"] = self.upd_camera_to_base
        return calib_dict

    def get_intrinsic(self, serial):
        assert serial in self.camera_serials, "Invalid serial {}".format(serial)
        return self.intrinsics[serial]
    
    def get_camera_to_base(self, serial, real_base = False):
        assert serial in self.camera_serials_global
        if self.calib_type == "airexo":
            return self.extrinsics[serial] @ np.linalg.inv(AIREXO_BASE_TO_MARKER)
        elif self.calib_type == "airexo_upd":
            if serial == self.upd_serial:
                return self.upd_camera_to_base
            else:
                return self.extrinsics[serial] @ np.linalg.inv(self.extrinsics[self.upd_serial]) @ self.upd_camera_to_base
        else:
            left_cam_to_base = self.get_camera_to_robot_left_base(serial, real_base = True) @ ROBOT_LEFT_REAL_BASE_TO_REAL_BASE
            right_cam_to_base = self.get_camera_to_robot_right_base(serial, real_base = True) @ ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE
            cam_to_base = average_xyz_rot_quat(left_cam_to_base, right_cam_to_base, rotation_rep = "matrix")
            if not real_base:
                cam_to_base = cam_to_base @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION)
            return cam_to_base
    
    def get_camera_to_robot_left_base(self, serial, real_base = False):
        assert serial in self.camera_serials_global
        assert self.calib_type == "robot"
        left_cam_to_base = self.extrinsics[serial] @ np.linalg.inv(self.extrinsics[self.camera_serial_inhand_left]) @ ROBOT_LEFT_CAM_TO_TCP @ np.linalg.inv(xyz_rot_to_mat(self.robot_left["tcp_pose"], rotation_rep = "quaternion"))
        if not real_base:
            left_cam_to_base = left_cam_to_base @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION)
        return left_cam_to_base

    def get_camera_to_robot_right_base(self, serial, real_base = False):
        assert serial in self.camera_serials_global
        assert self.calib_type == "robot"
        right_cam_to_base = self.extrinsics[serial] @ np.linalg.inv(self.extrinsics[self.camera_serial_inhand_right]) @ ROBOT_RIGHT_CAM_TO_TCP @ np.linalg.inv(xyz_rot_to_mat(self.robot_right["tcp_pose"], rotation_rep = "quaternion"))
        if not real_base:
            right_cam_to_base = right_cam_to_base @ np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION)
        return right_cam_to_base
