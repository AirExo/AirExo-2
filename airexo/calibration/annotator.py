"""
Calibration Annotator.

Authors: Hongjie Fang.
"""

import os
import cv2
import time
import h5py
import hydra
import numpy as np

from PIL import Image
from omegaconf import OmegaConf

from airexo.helpers.constants import *
from airexo.device.airexo import AirExo
from airexo.helpers.rotation import xyz_rot_to_mat
from airexo.calibration.calib_info import CalibrationInfo
from airexo.helpers.renderer import SeparateRobotRenderer, AirExoRenderer


def display_transformation(pose):
    print("[{:.8f}, {:.8f}, {:.8f}, {:.8f}],\n[{:.8f}, {:.8f}, {:.8f}, {:.8f}],\n[{:.8f}, {:.8f}, {:.8f}, {:.8f}],\n[{:.8f}, {:.8f}, {:.8f}, {:.8f}]\n".format(pose[0, 0], pose[0, 1], pose[0, 2], pose[0, 3], pose[1, 0], pose[1, 1], pose[1, 2], pose[1, 3], pose[2, 0], pose[2, 1], pose[2, 2], pose[2, 3], pose[3, 0], pose[3, 1], pose[3, 2], pose[3, 3]))

def display_calib_cfgs(calib_cfgs):
    for key in sorted(calib_cfgs.keys()):
        print("{}:".format(key))
        for ikey in ["type", "airexo", "robot"]:
            if ikey in calib_cfgs[key].keys():
                print("  {}: {}".format(ikey, calib_cfgs[key][ikey]))


class AnnotateRobot2DSepCalibratorFromData:
    """
    Manually Annotate Robot Calibrator with 2D Seperate Renderer using Recorded Data.
    """
    def __init__(
        self,
        calib_info: CalibrationInfo,
        camera_serial,
        robot_left_joint_cfgs,
        robot_right_joint_cfgs,
        urdf_file = {
            "left": "airexo/urdf_models/robot/left_robot_inhand.urdf",
            "right": "airexo/urdf_models/robot/right_robot_inhand.urdf"
        },
        near_plane = 0.01,
        far_plane = 100.0,
        initial_line_speed = 0.003,
        initial_angle_speed = np.pi / 360,
        line_step = 0.0005,
        angle_step = np.pi / 1440,
        timestamp_step = 1,
        **kwargs
    ):
        self.calib_info = calib_info
        assert camera_serial in self.calib_info.camera_serials_global
        self.camera_serial = camera_serial
        self.robot_left_joint_cfgs = robot_left_joint_cfgs
        self.robot_right_joint_cfgs = robot_right_joint_cfgs
        self.urdf_file = urdf_file
        self.alpha = 0.3
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.initial_line_speed = initial_line_speed
        self.initial_angle_speed = initial_angle_speed
        self.line_step = line_step
        self.angle_step = angle_step
        self.timestamp_step = timestamp_step

    def render_image(self, timestamp):
        original_image = np.array(Image.open(os.path.join(self.image_path, "{}.png".format(timestamp))))

        idx_left_robot = np.argmin(np.abs(np.array(self.left_robot_lowdim_file["timestamp"]) - int(timestamp)))
        idx_right_robot = np.argmin(np.abs(np.array(self.right_robot_lowdim_file["timestamp"]) - int(timestamp)))
        idx_left_gripper = np.argmin(np.abs(np.array(self.left_gripper_lowdim_file["timestamp"]) - int(timestamp)))
        idx_right_gripper = np.argmin(np.abs(np.array(self.right_gripper_lowdim_file["timestamp"]) - int(timestamp)))

        left_joint = np.concatenate([
            self.left_robot_lowdim_file["joint_pos"][idx_left_robot],
            [self.left_gripper_lowdim_file["width"][idx_left_gripper]]
        ], axis = 0)
        right_joint = np.concatenate([
            self.right_robot_lowdim_file["joint_pos"][idx_right_robot],
            [self.right_gripper_lowdim_file["width"][idx_right_gripper]]
        ], axis = 0)

        self.renderer.update_camera_to_left_base(self.cam_to_bases[0], delay_update = True)
        self.renderer.update_camera_to_right_base(self.cam_to_bases[1], delay_update = True)
        self.renderer.update_joints(left_joint, right_joint)
        
        # Render robot on robot image.
        image = self.renderer.render_image()
        mask = self.renderer.render_mask()

        overlapped_image = image.astype(np.float32) * self.alpha + original_image.astype(np.float32) * (1 - self.alpha)
        overlapped_image = overlapped_image.astype(np.uint8)
        return np.where(mask[:, :, None] == 255, overlapped_image, original_image)
    
    def update_calibration(self, coeff = 1.0):        
        cam_to_base = self.cam_to_bases[self.annotate_mode]
        if self.annotate_idx == 1:
            cam_to_base[:3, 3] += coeff * cam_to_base[:3, 0] * self.line_speed
        elif self.annotate_idx == 2:
            cam_to_base[:3, 3] += coeff * cam_to_base[:3, 1] * self.line_speed
        elif self.annotate_idx == 3:
            cam_to_base[:3, 3] += coeff * cam_to_base[:3, 2] * self.line_speed
        elif self.annotate_idx == 4:
            cam_to_base[:3, :3] = cam_to_base[:3, :3] @ np.array([
                [1, 0, 0],
                [0, np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed)],
                [0, np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed)]
            ])
        elif self.annotate_idx == 5:
            cam_to_base[:3, :3] = cam_to_base[:3, :3] @ np.array([
                [np.cos(coeff * self.angle_speed), 0, -np.sin(coeff * self.angle_speed)],
                [0, 1, 0],
                [np.sin(coeff * self.angle_speed), 0, np.cos(coeff * self.angle_speed)]
            ])
        elif self.annotate_idx == 6:
            cam_to_base[:3, :3] = cam_to_base[:3, :3] @ np.array([
                [np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed), 0],
                [np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed), 0],
                [0, 0, 1]
            ])
        self.cam_to_bases[self.annotate_mode] = cam_to_base

    def display_text_on_image(self, image):
        text = ""
        text += ("left" if self.annotate_mode == 0 else "right")
        if self.annotate_idx == 1:
            text += "  |  translation along x"
        elif self.annotate_idx == 2:
            text += "  |  translation along y"
        elif self.annotate_idx == 3:
            text += "  |  translation along z"
        elif self.annotate_idx == 4:
            text += "  |  rotation around x"
        elif self.annotate_idx == 5:
            text += "  |  rotation around y"
        elif self.annotate_idx == 6:
            text += "  |  rotation around z"
        if self.annotate_idx <= 3:
            text += "  |  speed: {:.5f} m".format(self.line_speed)
        else:
            text += "  |  speed: {:.5f} degrees".format(self.angle_speed / np.pi * 180)
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return image

    def run(
        self,
        data_path
    ):  
        # TODO: update robot annotator
        self.data_path = data_path
        self.image_path = os.path.join(data_path, "cam_{}".format(self.camera_serial), "color")
        self.lowdim_path = os.path.join(data_path, "lowdim")
        self.left_robot_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "robot_left.h5"), "r")
        self.right_robot_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "robot_right.h5"), "r")
        self.left_gripper_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "gripper_left.h5"), "r")
        self.right_gripper_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "gripper_right.h5"), "r")
        self.timestamp_list = sorted([os.path.splitext(x)[0] for x in os.listdir(self.image_path)])
        
        self.alpha = 0.3
        self.cam_to_bases = [
            self.calib_info.get_camera_to_robot_left_base(self.camera_serial),
            self.calib_info.get_camera_to_robot_right_base(self.camera_serial)
        ]
        self.annotate_mode = 0
        self.annotate_idx = 1
        self.line_speed = self.initial_line_speed
        self.angle_speed = self.initial_line_speed

        self.renderer = SeparateRobotRenderer(
            left_joint_cfgs = self.robot_left_joint_cfgs,
            right_joint_cfgs = self.robot_right_joint_cfgs,
            cam_to_left_base = self.cam_to_bases[0],
            cam_to_right_base = self.cam_to_bases[1],
            intrinsic = self.calib_info.get_intrinsic(self.camera_serial),
            width = 1280,
            height = 720,
            near_plane = self.near_plane,
            far_plane = self.far_plane,
            urdf_file = self.urdf_file
        )

        window_name = "Annotate Robot Seperate Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        timestamp_idx = 0

        while True:
            image = self.render_image(self.timestamp_list[timestamp_idx])
            image = self.display_text_on_image(image)

            cv2.imshow(window_name, image[:, :, ::-1])
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('l'):
                self.annotate_mode = 0
                self.annotate_idx = 1
                self.line_speed = self.initial_line_speed
                self.angle_speed = self.initial_line_speed
            elif key == ord('r'):
                self.annotate_mode = 1
                self.annotate_idx = 1
                self.line_speed = self.initial_line_speed
                self.angle_speed = self.initial_line_speed
            elif key == ord('1'):
                self.annotate_idx = 1
            elif key == ord('2'):
                self.annotate_idx = 2
            elif key == ord('3'):
                self.annotate_idx = 3
            elif key == ord('4'):
                self.annotate_idx = 4
            elif key == ord('5'):
                self.annotate_idx = 5
            elif key == ord('6'):
                self.annotate_idx = 6
            elif key == ord('='):
                if self.annotate_idx <= 3:
                    self.line_speed += self.line_step
                else:
                    self.angle_speed += self.angle_step
            elif key == ord('-'):
                if self.annotate_idx <= 3:
                    self.line_speed = max(self.line_speed - self.line_step, 0)
                else:
                    self.angle_speed = max(self.angle_speed - self.angle_step, 0)
            elif key == ord('['):
                self.update_calibration(coeff = -1.0)
            elif key == ord(']'):
                self.update_calibration(coeff = 1.0)   
            elif key == ord(","):
                timestamp_idx -= self.timestamp_step
                if timestamp_idx < 0:
                    timestamp_idx = len(self.timestamp_list) - 1
            elif key == ord("."):
                timestamp_idx += self.timestamp_step 
                if timestamp_idx >= len(self.timestamp_list):
                    timestamp_idx = 0
            elif key == ord("n"):
                self.alpha = max(self.alpha - 0.1, 0.0)
            elif key == ord("m"):
                self.alpha = min(self.alpha + 0.1, 1.0)
        
        NEW_ROBOT_LEFT_CAM_TO_TCP = self.calib_info.extrinsics[self.calib_info.camera_serial_inhand_left] @ np.linalg.inv(self.calib_info.extrinsics[self.camera_serial]) @ self.cam_to_bases[0] @ ROBOT_PREDEFINED_TRANSFORMATION @ xyz_rot_to_mat(self.calib_info.robot_left["tcp_pose"], rotation_rep = "quaternion")
        NEW_ROBOT_LEFT_FLANGE_TO_CAM = np.linalg.inv(NEW_ROBOT_LEFT_CAM_TO_TCP @ ROBOT_TCP_TO_FLANGE)
        NEW_ROBOT_RIGHT_CAM_TO_TCP = self.calib_info.extrinsics[self.calib_info.camera_serial_inhand_right] @ np.linalg.inv(self.calib_info.extrinsics[self.camera_serial]) @ self.cam_to_bases[1] @ ROBOT_PREDEFINED_TRANSFORMATION @ xyz_rot_to_mat(self.calib_info.robot_right["tcp_pose"], rotation_rep = "quaternion")
        NEW_ROBOT_RIGHT_FLANGE_TO_CAM = np.linalg.inv(NEW_ROBOT_RIGHT_CAM_TO_TCP @ ROBOT_TCP_TO_FLANGE)

        print("left: \n")
        display_transformation(NEW_ROBOT_LEFT_FLANGE_TO_CAM)
        print("\n\nright: \n")
        display_transformation(NEW_ROBOT_RIGHT_FLANGE_TO_CAM)

        # clean-ups
        cv2.destroyWindow(window_name)
    
    def stop(self):
        self.left_robot_lowdim_file.close()
        self.right_robot_lowdim_file.close()
        self.left_gripper_lowdim_file.close()
        self.right_gripper_lowdim_file.close()


class AnnotateAirExo2DCalibratorFromCamera:
    """
    Manually Annotate AirExo Calibrator with 2D Renderer using Camera Data.
    """
    def __init__(
        self,
        calib_info: CalibrationInfo,
        camera_serial,
        left_airexo: AirExo,
        right_airexo: AirExo,
        left_calib_cfgs,
        right_calib_cfgs,
        urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
        near_plane = 0.01,
        far_plane = 100.0,
        initial_line_speed = 0.003,
        initial_angle_speed = np.pi / 360,
        line_step = 0.0005,
        angle_step = np.pi / 1440,
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        self.calib_info = calib_info
        assert camera_serial in self.calib_info.camera_serials_global

        self.camera_serial = camera_serial
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)

        self.left_airexo = left_airexo
        self.right_airexo = right_airexo
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs

        self.urdf_file = urdf_file
        self.alpha = 0.3
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.initial_line_speed = initial_line_speed
        self.initial_angle_speed = initial_angle_speed
        self.line_step = line_step
        self.angle_step = angle_step
    
    def render_image(self):
        _, original_image = self.camera.get_rgb_image()
        left_joint = self.left_airexo.get_angle()
        right_joint = self.right_airexo.get_angle()
        
        # Render AirExo on AirExo image.
        self.renderer.update_camera_to_base(self.cam_to_base)
        self.renderer.update_calib_cfgs(self.calib_cfgs[0], self.calib_cfgs[1], delay_update = True)
        self.renderer.update_joints(left_joint, right_joint)
        image = self.renderer.render_image()
        mask = self.renderer.render_mask()
        
        overlapped_image = image.astype(np.float32) * self.alpha + original_image.astype(np.float32) * (1 - self.alpha)
        overlapped_image = overlapped_image.astype(np.uint8)
        return np.where(mask[:, :, None] == 255, overlapped_image, original_image)

    def update_calibration(self, coeff = 1.0): 
        if self.annotate_mode == -1:
            # annotating base transformation matrix 
            if self.annotate_idx == 1:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 0] * self.line_speed
            elif self.annotate_idx == 2:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 1] * self.line_speed
            elif self.annotate_idx == 3:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 2] * self.line_speed
            elif self.annotate_idx == 4:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [1, 0, 0],
                    [0, np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed)],
                    [0, np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed)]
                ])
            elif self.annotate_idx == 5:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [np.cos(coeff * self.angle_speed), 0, -np.sin(coeff * self.angle_speed)],
                    [0, 1, 0],
                    [np.sin(coeff * self.angle_speed), 0, np.cos(coeff * self.angle_speed)]
                ])
            elif self.annotate_idx == 6:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed), 0],
                    [np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed), 0],
                    [0, 0, 1]
                ])
        else:
            # annotating AirExo angle calibration 
            deg = (self.calib_cfgs[self.annotate_mode]["joint{}".format(self.annotate_idx)].airexo + coeff * self.angle_speed / np.pi * 180) % 360
            self.calib_cfgs[self.annotate_mode]["joint{}".format(self.annotate_idx)].airexo = float(deg)
    
    def display_text_on_image(self, image):
        text = ""
        if self.annotate_mode == -1:
            # annotating base transformation matrix 
            text += "base"
            if self.annotate_idx == 1:
                text += "  |  translation along x"
            elif self.annotate_idx == 2:
                text += "  |  translation along y"
            elif self.annotate_idx == 3:
                text += "  |  translation along z"
            elif self.annotate_idx == 4:
                text += "  |  rotation around x"
            elif self.annotate_idx == 5:
                text += "  |  rotation around y"
            elif self.annotate_idx == 6:
                text += "  |  rotation around z"
            if self.annotate_idx <= 3:
                text += "  |  speed: {:.5f} m".format(self.line_speed)
            else:
                text += "  |  speed: {:.5f} degrees".format(self.angle_speed / np.pi * 180)
        else:
            # annotating AirExo angle calibration 
            text += ("left" if self.annotate_mode == 0 else "right")
            text += "  |  joint {}".format(self.annotate_idx)
            text += "  |  speed: {:.5f} degrees".format(self.angle_speed / np.pi * 180)

        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return image

    def run(
        self,
        save_path = None
    ):  
        if save_path is not None:
            # create save path and prepare saving files
            trial_timestamp = int(time.time() * 1000)
            os.makedirs(save_path, exist_ok = True)
            save_dir = os.path.join(save_path, str(trial_timestamp))
            os.makedirs(save_dir, exist_ok = True)
            calib_dict = self.calib_info.to_dict()
            calib_dict["type"] = "airexo_upd"
            calib_dict["upd"] = {}
            calib_dict["upd"]["camera_serial"] = self.camera_serial

        # initialization
        self.alpha = 0.3
        self.cam_to_base = self.calib_info.get_camera_to_base(self.camera_serial)
        self.calib_cfgs = [self.left_calib_cfgs, self.right_calib_cfgs]
        self.annotate_mode = -1
        self.annotate_idx = 1
        self.line_speed = self.initial_line_speed
        self.angle_speed = self.initial_angle_speed

        # set up renderer
        self.renderer = AirExoRenderer(
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            cam_to_base = self.cam_to_base,
            intrinsic = self.calib_info.get_intrinsic(self.camera_serial),
            width = 1280,
            height = 720,
            near_plane = self.near_plane,
            far_plane = self.far_plane,
            urdf_file = self.urdf_file
        )

        # set up window
        window_name = "Annotate AirExo Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        # start annotation
        while True:
            image = self.render_image()
            image = self.display_text_on_image(image)

            cv2.imshow(window_name, image[:, :, ::-1])
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('b'):
                self.annotate_mode = -1
                self.annotate_idx = 1
                self.line_speed = self.initial_line_speed
                self.angle_speed = self.initial_line_speed
            elif key == ord('l'):
                self.annotate_mode = 0
                self.annotate_idx = 1
                self.angle_speed = self.initial_line_speed
            elif key == ord('r'):
                self.annotate_mode = 1
                self.annotate_idx = 1
                self.angle_speed = self.initial_line_speed
            elif key == ord('1'):
                self.annotate_idx = 1
            elif key == ord('2'):
                self.annotate_idx = 2
            elif key == ord('3'):
                self.annotate_idx = 3
            elif key == ord('4'):
                self.annotate_idx = 4
            elif key == ord('5'):
                self.annotate_idx = 5
            elif key == ord('6'):
                self.annotate_idx = 6
            elif key == ord('7'):
                if self.annotate_mode != -1:
                    self.annotate_idx = 7
            elif key == ord('='):
                if self.annotate_mode == -1 and self.annotate_idx <= 3:
                    self.line_speed += self.line_step
                else:
                    self.angle_speed += self.angle_step
            elif key == ord('-'):
                if self.annotate_mode == -1 and self.annotate_idx <= 3:
                    self.line_speed = max(self.line_speed - self.line_step, 0)
                else:
                    self.angle_speed = max(self.angle_speed - self.angle_step, 0)
            elif key == ord('['):
                self.update_calibration(coeff = -1.0)
            elif key == ord(']'):
                self.update_calibration(coeff = 1.0)   
            elif key == ord("n"):
                self.alpha = max(self.alpha - 0.1, 0.0)
            elif key == ord("m"):
                self.alpha = min(self.alpha + 0.1, 1.0)

        if save_path is not None:
            # save into files
            calib_dict["upd"]["camera_to_base"] = self.cam_to_base
            np.save(os.path.join(save_dir, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)
            np.save(os.path.join(self.calib_info.calib_path, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)

            with open(os.path.join(save_dir, 'calib_left.yaml'), 'w') as file:
                OmegaConf.save(self.calib_cfgs[0], file)
            with open(os.path.join(save_dir, 'calib_right.yaml'), 'w') as file:
                OmegaConf.save(self.calib_cfgs[1], file)
        
        # clean-ups
        cv2.destroyWindow(window_name)
    
    def stop(self):
        self.left_airexo.stop()
        self.right_airexo.stop()
        self.camera.stop()


class AnnotateAirExo2DCalibratorFromData:
    """
    Manually Annotate AirExo Calibrator with 2D Renderer using Recorded Data.
    """
    def __init__(
        self,
        calib_info: CalibrationInfo,
        camera_serial,
        airexo_left_joint_cfgs,
        airexo_right_joint_cfgs,
        left_calib_cfgs,
        right_calib_cfgs,
        urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
        near_plane = 0.01,
        far_plane = 100.0,
        initial_line_speed = 0.003,
        initial_angle_speed = np.pi / 360,
        line_step = 0.0005,
        angle_step = np.pi / 1440,
        timestamp_step = 1,
        **kwargs
    ):
        self.calib_info = calib_info
        assert camera_serial in self.calib_info.camera_serials_global
        self.camera_serial = camera_serial

        self.airexo_left_joint_cfgs = airexo_left_joint_cfgs
        self.airexo_right_joint_cfgs = airexo_right_joint_cfgs
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs

        self.urdf_file = urdf_file
        self.alpha = 0.3
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.initial_line_speed = initial_line_speed
        self.initial_angle_speed = initial_angle_speed
        self.line_step = line_step
        self.angle_step = angle_step
        self.timestamp_step = timestamp_step
    
    def render_image(self, timestamp):
        original_image = np.array(Image.open(os.path.join(self.image_path, "{}.png".format(timestamp))))

        idx_left_airexo = np.argmin(np.abs(np.array(self.left_airexo_lowdim_file["timestamp"]) - int(timestamp)))
        idx_right_airexo = np.argmin(np.abs(np.array(self.right_airexo_lowdim_file["timestamp"]) - int(timestamp)))
        left_joint = self.left_airexo_lowdim_file["encoder"][idx_left_airexo]
        right_joint = self.right_airexo_lowdim_file["encoder"][idx_right_airexo]
        
        # Render AirExo on AirExo image.
        self.renderer.update_camera_to_base(self.cam_to_base)
        self.renderer.update_calib_cfgs(self.calib_cfgs[0], self.calib_cfgs[1], delay_update = True)
        self.renderer.update_joints(left_joint, right_joint)
        image = self.renderer.render_image()
        mask = self.renderer.render_mask()
        
        overlapped_image = image.astype(np.float32) * self.alpha + original_image.astype(np.float32) * (1 - self.alpha)
        overlapped_image = overlapped_image.astype(np.uint8)
        return np.where(mask[:, :, None] == 255, overlapped_image, original_image)

    def update_calibration(self, coeff = 1.0): 
        if self.annotate_mode == -1:
            # annotating base transformation matrix 
            if self.annotate_idx == 1:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 0] * self.line_speed
            elif self.annotate_idx == 2:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 1] * self.line_speed
            elif self.annotate_idx == 3:
                self.cam_to_base[:3, 3] += coeff * self.cam_to_base[:3, 2] * self.line_speed
            elif self.annotate_idx == 4:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [1, 0, 0],
                    [0, np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed)],
                    [0, np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed)]
                ])
            elif self.annotate_idx == 5:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [np.cos(coeff * self.angle_speed), 0, -np.sin(coeff * self.angle_speed)],
                    [0, 1, 0],
                    [np.sin(coeff * self.angle_speed), 0, np.cos(coeff * self.angle_speed)]
                ])
            elif self.annotate_idx == 6:
                self.cam_to_base[:3, :3] = self.cam_to_base[:3, :3] @ np.array([
                    [np.cos(coeff * self.angle_speed), -np.sin(coeff * self.angle_speed), 0],
                    [np.sin(coeff * self.angle_speed), np.cos(coeff * self.angle_speed), 0],
                    [0, 0, 1]
                ])
        else:
            # annotating AirExo angle calibration 
            deg = (self.calib_cfgs[self.annotate_mode]["joint{}".format(self.annotate_idx)].airexo + coeff * self.angle_speed / np.pi * 180) % 360
            self.calib_cfgs[self.annotate_mode]["joint{}".format(self.annotate_idx)].airexo = float(deg)
    
    def display_text_on_image(self, image):
        text = ""
        if self.annotate_mode == -1:
            # annotating base transformation matrix 
            text += "base"
            if self.annotate_idx == 1:
                text += "  |  translation along x"
            elif self.annotate_idx == 2:
                text += "  |  translation along y"
            elif self.annotate_idx == 3:
                text += "  |  translation along z"
            elif self.annotate_idx == 4:
                text += "  |  rotation around x"
            elif self.annotate_idx == 5:
                text += "  |  rotation around y"
            elif self.annotate_idx == 6:
                text += "  |  rotation around z"
            if self.annotate_idx <= 3:
                text += "  |  speed: {:.5f} m".format(self.line_speed)
            else:
                text += "  |  speed: {:.5f} degrees".format(self.angle_speed / np.pi * 180)
        else:
            # annotating AirExo angle calibration 
            text += ("left" if self.annotate_mode == 0 else "right")
            text += "  |  joint {}".format(self.annotate_idx)
            text += "  |  speed: {:.5f} degrees".format(self.angle_speed / np.pi * 180)

        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return image

    def run(
        self,
        data_path,
        save_path = None
    ): 
        # read data
        self.data_path = data_path
        self.image_path = os.path.join(data_path, "cam_{}".format(self.camera_serial), "color")
        self.lowdim_path = os.path.join(data_path, "lowdim")
        self.left_airexo_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "airexo_left.h5"), "r")
        self.right_airexo_lowdim_file = h5py.File(os.path.join(self.lowdim_path, "airexo_right.h5"), "r")

        self.timestamp_list = sorted([os.path.splitext(x)[0] for x in os.listdir(self.image_path)])
        
        if save_path is not None:
            # create save path and prepare saving files
            trial_timestamp = int(time.time() * 1000)
            os.makedirs(save_path, exist_ok = True)
            save_dir = os.path.join(save_path, str(trial_timestamp))
            os.makedirs(save_dir, exist_ok = True)
            calib_dict = self.calib_info.to_dict()
            calib_dict["type"] = "airexo_upd"
            calib_dict["upd"] = {}
            calib_dict["upd"]["camera_serial"] = self.camera_serial

        # initialization
        self.alpha = 0.3
        self.cam_to_base = self.calib_info.get_camera_to_base(self.camera_serial)
        self.calib_cfgs = [self.left_calib_cfgs, self.right_calib_cfgs]
        self.annotate_mode = -1
        self.annotate_idx = 1
        self.line_speed = self.initial_line_speed
        self.angle_speed = self.initial_angle_speed

        # set up renderer
        self.renderer = AirExoRenderer(
            left_joint_cfgs = self.airexo_left_joint_cfgs,
            right_joint_cfgs = self.airexo_right_joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            cam_to_base = self.cam_to_base,
            intrinsic = self.calib_info.get_intrinsic(self.camera_serial),
            width = 1280,
            height = 720,
            near_plane = self.near_plane,
            far_plane = self.far_plane,
            urdf_file = self.urdf_file
        )

        # set up window
        window_name = "Annotate AirExo Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        # start annotation
        timestamp_idx = 0
        while True:
            image = self.render_image(self.timestamp_list[timestamp_idx])
            image = self.display_text_on_image(image)

            cv2.imshow(window_name, image[:, :, ::-1])
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('b'):
                self.annotate_mode = -1
                self.annotate_idx = 1
                self.line_speed = self.initial_line_speed
                self.angle_speed = self.initial_line_speed
            elif key == ord('l'):
                self.annotate_mode = 0
                self.annotate_idx = 1
                self.angle_speed = self.initial_line_speed
            elif key == ord('r'):
                self.annotate_mode = 1
                self.annotate_idx = 1
                self.angle_speed = self.initial_line_speed
            elif key == ord('1'):
                self.annotate_idx = 1
            elif key == ord('2'):
                self.annotate_idx = 2
            elif key == ord('3'):
                self.annotate_idx = 3
            elif key == ord('4'):
                self.annotate_idx = 4
            elif key == ord('5'):
                self.annotate_idx = 5
            elif key == ord('6'):
                self.annotate_idx = 6
            elif key == ord('7'):
                if self.annotate_mode != -1:
                    self.annotate_idx = 7
            elif key == ord('='):
                if self.annotate_mode == -1 and self.annotate_idx <= 3:
                    self.line_speed += self.line_step
                else:
                    self.angle_speed += self.angle_step
            elif key == ord('-'):
                if self.annotate_mode == -1 and self.annotate_idx <= 3:
                    self.line_speed = max(self.line_speed - self.line_step, 0)
                else:
                    self.angle_speed = max(self.angle_speed - self.angle_step, 0)
            elif key == ord('['):
                self.update_calibration(coeff = -1.0)
            elif key == ord(']'):
                self.update_calibration(coeff = 1.0)   
            elif key == ord(","):
                timestamp_idx -= self.timestamp_step
                if timestamp_idx < 0:
                    timestamp_idx = len(self.timestamp_list) - 1
            elif key == ord("."):
                timestamp_idx += self.timestamp_step
                if timestamp_idx >= len(self.timestamp_list):
                    timestamp_idx = 0
            elif key == ord("n"):
                self.alpha = max(self.alpha - 0.1, 0.0)
            elif key == ord("m"):
                self.alpha = min(self.alpha + 0.1, 1.0)

        if save_path is not None:
            # save into files
            calib_dict["upd"]["camera_to_base"] = self.cam_to_base
            np.save(os.path.join(save_dir, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)
            np.save(os.path.join(self.calib_info.calib_path, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)

            with open(os.path.join(save_dir, 'calib_left.yaml'), 'w') as file:
                OmegaConf.save(self.calib_cfgs[0], file)
            with open(os.path.join(save_dir, 'calib_right.yaml'), 'w') as file:
                OmegaConf.save(self.calib_cfgs[1], file)

        # clean-ups
        cv2.destroyWindow(window_name)
    
    def stop(self):
        self.left_airexo_lowdim_file.close()
        self.right_airexo_lowdim_file.close()