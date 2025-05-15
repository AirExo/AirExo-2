"""
Calibration Solver.

Authors: Hongjie Fang, Jun Lv.
"""

import os
import cv2
import copy
import time
import torch
import pyredner
import numpy as np
import open3d as o3d
import pytorch_kinematics as pk
import pytorch3d.transforms.rotation_conversions as ptc

from PIL import Image
from omegaconf import OmegaConf

from airexo.helpers.constants import *
from airexo.calibration.calib_info import CalibrationInfo


class AirExoCalibrationDifferntiableRenderingSolver:
    """
    Solving Calibration using Differentiable Rendering.
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
        device = "cpu",
        max_translation = 0.03,
        max_rotation = np.pi / 60,
        max_degree = 2,
        width = 1280,
        height = 720,
        max_disparency = 0.3,
        **kwargs
    ):
        pyredner.set_print_timing(False)
        if device == "cpu":
            pyredner.set_use_gpu(False)
        else:
            pyredner.set_use_gpu(True)
        # set up parameters
        self.calib_info = calib_info
        self.camera_serial = camera_serial
        self.airexo_left_joint_cfgs = airexo_left_joint_cfgs
        self.airexo_right_joint_cfgs = airexo_right_joint_cfgs
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs
        self.urdf_file = urdf_file
        self.max_translation = max_translation
        self.max_rotation = max_rotation
        self.max_degree = max_degree
        self.max_disparency = max_disparency
        self.width = width
        self.height = height
        self.device = torch.device(device)

        # get initial solution
        self.cam_to_base = torch.from_numpy(calib_info.get_camera_to_base(serial = camera_serial)).to(self.device)

        # Additional efforts to align open3d.PinholeCamera to pyredner.Camera
        self.intrinsic = calib_info.get_intrinsic(serial = camera_serial)
        self.fx = self.intrinsic[0, 0]
        self.fy = self.intrinsic[1, 1]
        self.cx = self.intrinsic[0, 2]
        self.cy = self.intrinsic[1, 2]
        self.aspect_ratio = width / height

        redner_intrinsic_mat = torch.tensor([
            [2 * self.fx / width, 0, -1 + 2 * self.cx / width], 
            [0, 2 * self.fy / height / self.aspect_ratio, (1 - 2 * self.cy / height) / self.aspect_ratio], 
            [0, 0, 1]
        ], dtype = torch.float32, device = self.device)


        # load joint configurations
        self.left_calib = torch.zeros((self.airexo_left_joint_cfgs.num_robot_joints, ), dtype = torch.float32, device = self.device)
        self.left_direction = torch.zeros((self.airexo_left_joint_cfgs.num_robot_joints, ), dtype = torch.float32, device = self.device)
        for i in range(self.airexo_left_joint_cfgs.num_robot_joints):
            self.left_calib[i] = self.left_calib_cfgs["joint{}".format(i + 1)].airexo
            self.left_direction[i] = self.airexo_left_joint_cfgs["joint{}".format(i + 1)].direction
            
        self.right_calib = torch.zeros((self.airexo_right_joint_cfgs.num_robot_joints, ), dtype = torch.float32, device = self.device)
        self.right_direction = torch.zeros((self.airexo_right_joint_cfgs.num_robot_joints, ), dtype = torch.float32, device = self.device)
        for i in range(self.airexo_right_joint_cfgs.num_robot_joints):
            self.right_calib[i] = self.right_calib_cfgs["joint{}".format(i + 1)].airexo
            self.right_direction[i] = self.airexo_right_joint_cfgs["joint{}".format(i + 1)].direction

        # initialize forward kinematics
        self.chain = pk.build_chain_from_urdf(open(urdf_file).read())
        self.chain = self.chain.to(dtype = torch.float32, device = self.device)
        self.link_names = self.chain.get_link_names()
        self.links = self.chain.get_links()
        self.object_render = []
        self.initial_object_render = []
        self.object_shape_num = []
        self.predefined_transformation = torch.from_numpy(AIREXO_PREDEFINED_TRANSFORMATION).to(self.device)
        self.left_tcp_to_joint7 = torch.from_numpy(AIREXO_LEFT_TCP_TO_JOINT7).to(self.device)
        self.right_tcp_to_joint7 = torch.from_numpy(AIREXO_RIGHT_TCP_TO_JOINT7).to(self.device)

        # load urdf file
        for i, link_name in enumerate(self.link_names):
            for visual in self.links[i].visuals:
                init_transform = self.links[i].offset.get_matrix() @ visual.offset.get_matrix().to(self.device)
                init_transform = init_transform.reshape(4, 4)
                obj_path = visual.geom_param[0]
                obj_path = os.path.join(os.path.dirname(urdf_file), obj_path)
                render_obj = pyredner.load_obj(obj_path, return_objects = True)
                for m in render_obj:
                    m.vertices = (m.vertices @ init_transform[:3, :3]) + init_transform[:3, 3]
                    m.normals = pyredner.compute_vertex_normal(m.vertices, m.indices)
                self.object_render.extend(render_obj)
                self.initial_object_render.extend(copy.deepcopy(render_obj))
                self.object_shape_num.append(len(render_obj))

        # set up renderer: camera and scene.
        self.camera = pyredner.Camera(
            resolution = (self.height, self.width),
            intrinsic_mat = redner_intrinsic_mat,
            cam_to_world = torch.tensor([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],   
                [0, 0, 0, 1]
            ], dtype=torch.float32, device = self.device, requires_grad = False),
            clip_near = 0.01,
        )

        self.scene = pyredner.Scene(camera = self.camera, objects = self.object_render)
    

    def recover_from_params(self, params):
        """
        Recover original calibration matrix and values from initial value and predicted offset.
        """
        cam_to_base = torch.eye(4, device = self.device, dtype = torch.float32)
        cam_to_base[:3, :3] = ptc.rotation_6d_to_matrix(params[3:9])
        cam_to_base[:3, 3] = self.cam_to_base[:3, 3] + torch.clamp(params[:3], -1, 1) * self.max_translation

        left_calib = (self.left_calib + torch.clamp(params[9: 9 + self.airexo_left_joint_cfgs.num_robot_joints], -1, 1) * self.max_degree) % 360
        right_calib = (self.right_calib + torch.clamp(params[9 + self.airexo_left_joint_cfgs.num_robot_joints:], -1, 1) * self.max_degree) % 360

        return cam_to_base, left_calib, right_calib
    

    def evaluate(self, params):
        """
        Evaluate the performance of current predicted offset.
        """
        cam_to_base, left_calib, right_calib = self.recover_from_params(params)

        mask_losses = []
        depth_losses = []
        for data_idx in range(self.num_data_depth):
            mask_gt = self.masks[data_idx]
            depth_mask = self.depth_masks[data_idx]
            depth_gt = self.depths[data_idx]
            left_joint = self.left_joints[data_idx]
            right_joint = self.right_joints[data_idx]

            # calculate urdf joint values
            left_joint_robot = self.left_direction * (left_calib - left_joint[:-1]) / 180 * torch.pi
            # TODO: make sure that grippers are closed.
            gripper = torch.zeros((2, ), dtype = torch.float32, device = self.device)
            right_joint_robot = self.right_direction * (right_calib - right_joint[:-1]) / 180 * torch.pi
            urdf_joints = torch.cat([left_joint_robot, gripper, right_joint_robot, gripper], dim = 0)

            # differentiable forward kinematic and rendering
            fk_res = self.chain.forward_kinematics(urdf_joints)
            visual_idx = 0
            shape_idx = 0
            for i, link_name in enumerate(self.link_names):
                cur_transform = cam_to_base @ self.predefined_transformation @ fk_res[link_name].get_matrix().reshape(4, 4)
                for _ in range(len(self.links[i].visuals)):
                    for _ in range(self.object_shape_num[visual_idx]):
                        self.scene.shapes[shape_idx].vertices = self.initial_object_render[shape_idx].vertices @ cur_transform[:3, :3].T + cur_transform[:3, 3]
                        self.scene.shapes[shape_idx].normals = pyredner.compute_vertex_normal(self.scene.shapes[shape_idx].vertices, self.scene.shapes[shape_idx].indices)
                        shape_idx += 1
                    visual_idx += 1
            
            # render
            render_res = pyredner.render_generic(scene = self.scene, channels = [pyredner.channels.position, pyredner.channels.alpha], max_bounces = 0)
            depth = render_res[:, :, 2]
            depth_mask = (depth_mask & (depth > 0) & (depth_gt > 0))
            mask = render_res[:, :, 3]

            # calculate loss
            mask_loss = torch.mean(((mask - mask_gt) ** 2))
            mask_losses.append(mask_loss)
            cv2.imwrite("tests/{}.png".format(data_idx), ((mask - mask_gt) ** 2).cpu().detach().numpy() * 255)

            # calculate depth loss
            depth_loss = (depth - depth_gt) ** 2
            depth_loss = torch.mean(depth_loss[depth_mask])
            depth_losses.append(depth_loss)
        
        # calculate average loss and return
        avg_depth_loss = torch.stack(depth_losses).mean()
        avg_mask_loss = torch.stack(mask_losses).mean()
        
        return avg_depth_loss, avg_mask_loss


    def solve(
        self,
        data_path,
        save_path,
        num_iter = 1000, 
        lr = 0.0001,
        beta = 5,
        **kwargs
    ):
        """
        Solve the calibration problem.
        """
        # read data of depth
        self.masks = []
        self.depth_masks = []
        self.left_joints = []
        self.right_joints = []
        self.depths = []
        self.num_data_depth = len(os.listdir(data_path))

        for data_item in os.listdir(data_path):
            mask = torch.from_numpy(np.array(Image.open(os.path.join(data_path, data_item, "mask.png"))).astype(np.float32)) / 255.
            depth_mask = torch.from_numpy(np.array(Image.open(os.path.join(data_path, data_item, "depth_mask.png"))).astype(np.bool_))
            depth = torch.from_numpy(np.array(Image.open(os.path.join(data_path, data_item, "depth.png"))).astype(np.float32)) / 1000.
            left_joint = torch.from_numpy(np.load(os.path.join(data_path, data_item, "left.npy")))
            right_joint = torch.from_numpy(np.load(os.path.join(data_path, data_item, "right.npy")))
            self.masks.append(mask)
            self.depth_masks.append(depth_mask)
            self.left_joints.append(left_joint)
            self.right_joints.append(right_joint)
            self.depths.append(depth)

        self.masks = torch.stack(self.masks).to(self.device)
        self.depth_masks = torch.stack(self.depth_masks).to(self.device)
        self.depths = torch.stack(self.depths).to(self.device)
        self.left_joints = torch.stack(self.left_joints).to(self.device)
        self.right_joints = torch.stack(self.right_joints).to(self.device)

        # create save path and prepare saving files
        trial_timestamp = int(time.time() * 1000)
        os.makedirs(save_path, exist_ok = True)
        save_dir = os.path.join(save_path, str(trial_timestamp))
        os.makedirs(save_dir, exist_ok = True)
        calib_dict = self.calib_info.to_dict()
        calib_dict["type"] = "airexo_upd"
        calib_dict["upd"] = {}
        calib_dict["upd"]["camera_serial"] = self.camera_serial

        # initial value
        init_value = torch.zeros(9 + self.airexo_left_joint_cfgs.num_robot_joints + self.airexo_right_joint_cfgs.num_robot_joints, dtype = torch.float32, device = self.device)  
        init_value[3:9] = ptc.matrix_to_rotation_6d(self.cam_to_base[:3, :3])     
        params = torch.nn.Parameter(init_value.clone().detach().requires_grad_(True))
        best_params = None
        best_loss = torch.inf

        # solve the optimization problem iteratively
        optimizer = torch.optim.Adam([params], lr = lr)
        for iter in range(num_iter):
            optimizer.zero_grad()
            loss_depth, loss_mask = self.evaluate(params)
            
            loss = loss_depth + loss_mask * beta
            loss_value = loss.item()
            print("Iteration {}, Loss: {} / Depth Loss: {}, Mask Loss: {} ".format(iter, loss_value, loss_depth.item(), loss_mask.item()))

            # update results
            if loss_value < best_loss:
                best_loss = loss_value
                best_params = params
                cam_to_base, left_calib, right_calib = self.recover_from_params(best_params)
                cam_to_base = cam_to_base.cpu().detach().numpy()
                left_calib = left_calib.cpu().detach().numpy()
                right_calib = right_calib.cpu().detach().numpy()
                left_calib_cfgs = copy.deepcopy(self.left_calib_cfgs)
                right_calib_cfgs = copy.deepcopy(self.right_calib_cfgs)
                for i in range(self.airexo_left_joint_cfgs.num_robot_joints):
                    left_calib_cfgs["joint{}".format(i + 1)].airexo = float(left_calib[i])
                for i in range(self.airexo_right_joint_cfgs.num_robot_joints):
                    right_calib_cfgs["joint{}".format(i + 1)].airexo = float(right_calib[i])
                # save into files
                calib_dict["upd"]["camera_to_base"] = cam_to_base                
                np.save(os.path.join(save_dir, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)
                np.save(os.path.join(self.calib_info.calib_path, "{}.npy".format(trial_timestamp)), calib_dict, allow_pickle = True)
                with open(os.path.join(save_dir, 'calib_left.yaml'), 'w') as file:
                    OmegaConf.save(left_calib_cfgs, file)
                with open(os.path.join(save_dir, 'calib_right.yaml'), 'w') as file:
                    OmegaConf.save(right_calib_cfgs, file)
            
            loss.backward()
            optimizer.step()

