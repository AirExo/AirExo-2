"""
Dataset Transformation Script.

Transform the dataset into the required format for the RISE-2 policy. Also implement operational space adaptor for in-the-wild demonstrations.

Usage:
  Set up all other configurations in the config files, then
  python -m airexo.adaptor.dataset_transform +path=[Task Path]

Authors: Hongjie Fang, Jingjing Chen
"""

import os
import h5py
import json
import hydra
import shutil
import numpy as np

from tqdm import tqdm
from omegaconf import OmegaConf

from airexo.helpers.logger import setup_loggers
from airexo.helpers.transform import transform_arm
from airexo.helpers.state import airexo_transform_tcp
from airexo.helpers.rotation import apply_mat_to_pose, mat_to_xyz_rot


def process_gripper_sequence(sequence, gripper_width_threshold):
    processed = sequence.copy()
    slope = None

    for i in range(1, len(sequence)):
        if sequence[i] < sequence[i-1] - gripper_width_threshold:
            current_slope = sequence[i] - sequence[i-1]
            if slope is None:
                slope = current_slope
            else:
                if current_slope < slope:
                    slope = current_slope
        if sequence[i] > sequence[i-1] + gripper_width_threshold:
            slope = None
        if slope is not None:
            predicted_value = processed[i-1] + slope
            if predicted_value <= 0:
                processed[i] = 0
            else:
                processed[i] = predicted_value
    return processed


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "adaptor"),
    config_name = "dataset_transform.yaml"
)
def main(cfg):
    setup_loggers()
    OmegaConf.resolve(cfg)

    # set up paths
    path = cfg.path
    if path is None:
        raise AttributeError("Please provide task path.")

    # read calibration info, and transform it into the required format for the RISE-2 policy.
    calib_info = hydra.utils.instantiate(cfg.calib_info)
    cam_to_base = calib_info.get_camera_to_base(serial = cfg.camera_serial)
    is_robot = (calib_info.calib_type == "robot")

    calib_res = {
        "type": calib_info.calib_type,
        "camera_serials": calib_info.camera_serials,
        "camera_serials_global": calib_info.camera_serials_global,
        "camera_serial_inhand_left": calib_info.camera_serial_inhand_left,
        "camera_serial_inhand_right": calib_info.camera_serial_inhand_right,
        "intrinsics": calib_info.intrinsics
    }
    calib_res["camera_to_robot_left"] = {}
    calib_res["camera_to_robot_right"] = {}

    for serial in calib_info.camera_serials_global:
        if is_robot:
            calib_res["camera_to_robot_left"][serial] = calib_info.get_camera_to_robot_left_base(serial, real_base = True)
            calib_res["camera_to_robot_right"][serial] = calib_info.get_camera_to_robot_right_base(serial, real_base = True)
        else:
            calib_res["camera_to_robot_left"][serial] = np.eye(4, dtype = np.float32)
            calib_res["camera_to_robot_right"][serial] = np.eye(4, dtype = np.float32)

    os.makedirs(os.path.join(path, "calib"))
    np.save(os.path.join(path, "calib", "{}.npy".format(calib_info.calib_timestamp)), calib_res, allow_pickle = True)

    # transform every scene 
    scene_list = [x for x in sorted(os.listdir(path)) if x[:6] == "scene_"]
    
    for scene in tqdm(scene_list):
        scene_path = os.path.join(path, scene)
        cam_path = os.path.join(scene_path, "cam_{}".format(cfg.camera_serial))
        color_path = os.path.join(cam_path, "color")
        lowdim_path = os.path.join(scene_path, "lowdim")

        # add calibration info into the metadata
        with open(os.path.join(scene_path, "meta.json"), "r") as f:
            meta = json.load(f)
        meta["calib_timestamp"] = calib_info.calib_timestamp
        with open(os.path.join(scene_path, "meta.json"), "w") as f:
            json.dump(meta, f)
    
        # initialize timestamps
        timestamps = sorted([int(os.path.splitext(x)[0]) for x in os.listdir(color_path)])

        if is_robot: # for teleoperated demonstrations
            # open lowdim files
            robot_left_file = h5py.File(os.path.join(lowdim_path, "robot_left.h5"), "r")
            robot_right_file = h5py.File(os.path.join(lowdim_path, "robot_right.h5"), "r")
            gripper_left_file = h5py.File(os.path.join(lowdim_path, "gripper_left.h5"), "r")
            gripper_right_file = h5py.File(os.path.join(lowdim_path, "gripper_right.h5"), "r")
            robot_left_timestamps = np.array(robot_left_file["timestamp"])
            robot_right_timestamps = np.array(robot_right_file["timestamp"])
            gripper_left_timestamps = np.array(gripper_left_file["timestamp"])
            gripper_right_timestamps = np.array(gripper_right_file["timestamp"])

            for timestamp in timestamps:
                # Find ref frame ids for left_robot/right_robot/left_gripper/right_gripper
                ref_ids_robot_left = np.argmin(np.abs(robot_left_timestamps - int(timestamp)))
                ref_ids_robot_right = np.argmin(np.abs(robot_right_timestamps - int(timestamp)))
                ref_ids_gripper_left = np.argmin(np.abs(gripper_left_timestamps - int(timestamp)))
                ref_ids_gripper_right = np.argmin(np.abs(gripper_right_timestamps - int(timestamp)))
                lowdim_dict = {
                    "robot_left": np.array(robot_left_file["tcp_pose"][ref_ids_robot_left]),
                    "robot_right": np.array(robot_right_file["tcp_pose"][ref_ids_robot_right]),
                    "gripper_left": np.array([gripper_left_file["width"][ref_ids_gripper_left], gripper_left_file["action"][ref_ids_gripper_left]], dtype = np.float32),
                    "gripper_right": np.array([gripper_right_file["width"][ref_ids_gripper_right], gripper_right_file["action"][ref_ids_gripper_right]], dtype = np.float32)
                }
                np.save(os.path.join(lowdim_path, "{}.npy".format(timestamp)), lowdim_dict, allow_pickle = True)

            robot_left_file.close()
            robot_right_file.close()
            gripper_left_file.close()
            gripper_right_file.close()
        
        else: # for in-the-wild demonstration
            # open lowdim files
            airexo_left_file = h5py.File(os.path.join(lowdim_path, "airexo_left.h5"), "r")
            airexo_right_file = h5py.File(os.path.join(lowdim_path, "airexo_right.h5"), "r")
            airexo_left_timestamps = np.array(airexo_left_file["timestamp"]) 
            airexo_right_timestamps = np.array(airexo_right_file["timestamp"])

            left_tcps = []
            right_tcps = []
            left_gripper = []
            right_gripper = []

            for timestamp in timestamps:
                idx_left_airexo = np.argmin(np.abs(airexo_left_timestamps - int(timestamp)))
                idx_right_airexo = np.argmin(np.abs(airexo_right_timestamps - int(timestamp)))
                # AirExo joint readings
                airexo_left_joint = airexo_left_file["encoder"][idx_left_airexo]
                airexo_right_joint = airexo_right_file["encoder"][idx_right_airexo]
                # transform to robot joint readings
                robot_left_joint = transform_arm(
                    robot_cfgs = cfg.robot_left_joint_cfgs,
                    airexo_cfgs = cfg.airexo_left_joint_cfgs,
                    calib_cfgs = cfg.left_calib_cfgs,
                    data = airexo_left_joint
                )
                robot_right_joint = transform_arm(
                    robot_cfgs = cfg.robot_right_joint_cfgs,
                    airexo_cfgs = cfg.airexo_right_joint_cfgs,
                    calib_cfgs = cfg.right_calib_cfgs,
                    data = airexo_right_joint
                )
                # calculate tcp in the shared base
                left_tcp_in_base, right_tcp_in_base = airexo_transform_tcp(
                    left_joint = airexo_left_joint,
                    right_joint = airexo_right_joint,
                    left_joint_cfgs = cfg.airexo_left_joint_cfgs,
                    right_joint_cfgs = cfg.airexo_right_joint_cfgs,
                    left_calib_cfgs = cfg.left_calib_cfgs,
                    right_calib_cfgs = cfg.right_calib_cfgs,
                    is_rad = False,
                    urdf_file = cfg.urdf_file,
                    real_robot_base = False
                )
                # project tcp to the camera coordinate
                left_tcp_in_camera = apply_mat_to_pose(
                    pose = left_tcp_in_base,
                    mat = cam_to_base,
                    rotation_rep = "matrix"
                )
                right_tcp_in_camera = apply_mat_to_pose(
                    pose = right_tcp_in_base,
                    mat = cam_to_base,
                    rotation_rep = "matrix"
                )
                left_tcp_in_camera = mat_to_xyz_rot(
                    left_tcp_in_camera,
                    rotation_rep = "quaternion"
                )
                right_tcp_in_camera = mat_to_xyz_rot(
                    right_tcp_in_camera,
                    rotation_rep = "quaternion"
                )
                # process gripper
                left_gripper_state = robot_left_joint[-1]
                right_gripper_state = robot_right_joint[-1]

                left_tcps.append(left_tcp_in_camera)
                right_tcps.append(right_tcp_in_camera)
                left_gripper.append(left_gripper_state)
                right_gripper.append(right_gripper_state)

            airexo_left_file.close()
            airexo_right_file.close()

            # deal with gripper states
            left_gripper_action = []
            right_gripper_action = []
            for idx in range(0, len(timestamps)):
                left_gripper_action.append(left_gripper[idx])
                right_gripper_action.append(right_gripper[idx])

            left_gripper_action = process_gripper_sequence(left_gripper_action, cfg.gripper_width_threshold)
            right_gripper_action = process_gripper_sequence(right_gripper_action, cfg.gripper_width_threshold)

            assert len(left_gripper) == len(timestamps)
            assert len(right_gripper) == len(timestamps)
            assert len(left_gripper_action) == len(timestamps)
            assert len(right_gripper_action) == len(timestamps)
            assert len(left_tcps) == len(timestamps)
            assert len(right_tcps) == len(timestamps)

            # saving into files
            for i, timestamp in enumerate(timestamps):
                res_dict = {
                    "robot_left": np.array(left_tcps[i]),
                    "robot_right": np.array(right_tcps[i]),
                    "gripper_left": np.array([left_gripper[i], left_gripper_action[i]]),
                    "gripper_right": np.array([right_gripper[i], right_gripper_action[i]])
                }
                np.save(os.path.join(lowdim_path, "{}.npy".format(timestamp)), res_dict, allow_pickle = True)
        
    # move to the train folder
    os.makedirs(os.path.join(path, "train"), exist_ok = True)

    for scene in tqdm(scene_list):
        if scene[:6] == 'scene_':
            shutil.move(os.path.join(path, scene), os.path.join(path, "train", scene))


if __name__ == '__main__':
    main()