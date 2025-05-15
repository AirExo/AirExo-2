"""
AirExo / Robot Renderer based on URDF.

Author: Hongjie Fang.
"""

import os
import numpy as np
import open3d as o3d

import airexo.helpers.urdf_robot as robot_helper
import airexo.helpers.urdf_airexo as airexo_helper

from airexo.helpers.constants import *


class AirExoRenderer:
    def __init__(
        self,
        left_joint_cfgs,
        right_joint_cfgs,
        left_calib_cfgs,
        right_calib_cfgs,
        cam_to_base,
        intrinsic,
        width = 1280,
        height = 720,
        near_plane = 0.1,
        far_plane = 20.0,
        urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
        **kwargs
    ):
        """
        Parameters:
        - left_joint_cfgs: left joint configurations of AirExo;
        - right_joint_cfgs: right joint configurations of AirExo;
        - left_calib_cfgs: left joint calibration configurations of AirExo;
        - right_calib_cfgs: right joint calibration configurations of AirExo;
        - cam_to_base: the transformation matrix between camera and AirExo base;
        - intrinsic: the camera intrinsic matrix;
        - width, height: the rendered width/height of the images;
        - near_plane, far_plane: the near/far plane of the projection;
        - urdf_file: the urdf file of AirExo.
        """
        # save arguments
        self.left_joint_cfgs = left_joint_cfgs
        self.right_joint_cfgs = right_joint_cfgs
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs
        self.cam_to_base = cam_to_base
        self.urdf_file = urdf_file

        # initialize renderer
        self.renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultLit" 

        # forward kinematics of robot
        cur_transforms, self.visuals_map = airexo_helper.forward_kinematic(
            left_joint = np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32),
            right_joint = np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32),
            left_joint_cfgs = self.left_joint_cfgs,
            right_joint_cfgs = self.right_joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = right_calib_cfgs,
            is_rad = False,
            urdf_file = self.urdf_file,
            with_visuals_map = True
        )
        self.last_joints = (np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32), np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32))

        # read mesh, add into initial scene
        self.model_meshes = {}
        self.last_transforms = {}

        for link, transform in cur_transforms.items():
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                mesh_name = "{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                mesh = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(urdf_file), v.geom_param))
                mesh.transform(tf)
                mesh.compute_vertex_normals()
                self.model_meshes[mesh_name] = mesh
                self.last_transforms[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, mesh, self.material)
        
        # set camera projection
        self.renderer.scene.camera.set_projection(intrinsic, near_plane, far_plane, float(width), float(height))

    def _update_geometry(self, joints = None):
        # by default, load last joint information
        if joints is None:
            joints = self.last_joints
        left_joint, right_joint = joints

        # forward kinematics of robot
        cur_transforms = airexo_helper.forward_kinematic(
            left_joint = left_joint,
            right_joint = right_joint,
            left_joint_cfgs = self.left_joint_cfgs,
            right_joint_cfgs = self.right_joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False,
            urdf_file = self.urdf_file,
            with_visuals_map = False
        )

        # update mesh in the scene
        self.renderer.scene.clear_geometry()
        for link, transform in cur_transforms.items():
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                mesh_name = "{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                self.model_meshes[mesh_name].transform(tf @ np.linalg.inv(self.last_transforms[mesh_name]))
                self.model_meshes[mesh_name].compute_vertex_normals()
                self.last_transforms[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, self.model_meshes[mesh_name], self.material)
    
    def update_joints(self, left_joint, right_joint):
        self._update_geometry(joints = (left_joint, right_joint))
    
    def update_camera_to_base(self, cam_to_base, delay_update = False):
        self.cam_to_base = cam_to_base
        if not delay_update:
            self._update_geometry()

    def update_calib_cfgs(self, left_calib_cfgs, right_calib_cfgs, delay_update = False):
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs
        if not delay_update:
            self._update_geometry()

    def render_image(self):
        return np.asarray(self.renderer.render_to_image(), dtype = np.uint8)
    
    def render_depth(self):
        return np.asarray(self.renderer.render_to_depth_image(z_in_view_space = True), dtype = np.float32)
    
    def render_mask(self, depth = None):
        if depth is None:
            depth = self.render_depth()
        mask = np.zeros(depth.shape, dtype = np.uint8)
        mask[depth < np.inf] = 255
        return mask
        

class RobotRenderer:
    """
    Robot Renderer.
    """
    def __init__(
        self,
        left_joint_cfgs,
        right_joint_cfgs,
        cam_to_base,
        intrinsic,
        width = 1280,
        height = 720,
        near_plane = 0.01,
        far_plane = 100.0,
        urdf_file = os.path.join("airexo", "urdf_models", "robot", "robot.urdf"),
        **kwargs
    ):
        """
        Parameters:
        - left_joint_cfgs: left joint configurations of robot;
        - right_joint_cfgs: right joint configurations of robot;
        - cam_to_base: the transformation matrix between camera and robot base;
        - intrinsic: the camera intrinsic matrix;
        - width, height: the rendered width/height of the images;
        - near_plane, far_plane: the near/far plane of the projection;
        - urdf_file: the urdf file of robot.
        """
        # save arguments
        self.left_joint_cfgs = left_joint_cfgs
        self.right_joint_cfgs = right_joint_cfgs
        self.cam_to_base = cam_to_base
        self.urdf_file = urdf_file

        # initialize renderer
        self.renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultLit" 

        # forward kinematics of robot
        cur_transforms, self.visuals_map = robot_helper.forward_kinematic(
            left_joint = np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32),
            right_joint = np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32),
            left_joint_cfgs = self.left_joint_cfgs,
            right_joint_cfgs = self.right_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file,
            with_visuals_map = True
        )
        self.last_joints = (np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32), np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32))

        # read mesh, add into initial scene
        self.model_meshes = {}
        self.last_transforms = {}
        for link, transform in cur_transforms.items():
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                mesh_name = "{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                mesh = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(urdf_file), v.geom_param))
                mesh.transform(tf)
                mesh.compute_vertex_normals()
                self.model_meshes[mesh_name] = mesh
                self.last_transforms[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, mesh, self.material)
            
        # set camera projection
        self.renderer.scene.camera.set_projection(intrinsic, near_plane, far_plane, float(width), float(height))

    def _update_geometry(self, joints = None):
        # by default, load last joint information
        if joints is None:
            joints = self.last_joints
        left_joint, right_joint = joints

        # forward kinematics of robot
        cur_transforms = robot_helper.forward_kinematic(
            left_joint = left_joint,
            right_joint = right_joint,
            left_joint_cfgs = self.left_joint_cfgs,
            right_joint_cfgs = self.right_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file,
            with_visuals_map = False
        )

        # update mesh in the scene
        self.renderer.scene.clear_geometry()
        for link, transform in cur_transforms.items():
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                mesh_name = "{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                self.model_meshes[mesh_name].transform(tf @ np.linalg.inv(self.last_transforms[mesh_name]))
                self.model_meshes[mesh_name].compute_vertex_normals()
                self.last_transforms[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, self.model_meshes[mesh_name], self.material)
    
    def update_joints(self, left_joint, right_joint):
        self._update_geometry(joints = (left_joint, right_joint))
    
    def update_camera_to_base(self, cam_to_base, delay_update = False):
        self.cam_to_base = cam_to_base
        if not delay_update:
            self._update_geometry()

    def render_image(self):
        return np.asarray(self.renderer.render_to_image(), dtype = np.uint8)
    
    def render_depth(self):
        return np.asarray(self.renderer.render_to_depth_image(z_in_view_space = True), dtype = np.float32)
    
    def render_mask(self, depth = None):
        if depth is None:
            depth = self.render_depth()
        mask = np.zeros(depth.shape, dtype = np.uint8)
        mask[depth < np.inf] = 255
        return mask


class SeparateRobotRenderer:
    """
    Separate Robot Renderer.
    """
    def __init__(
        self,
        left_joint_cfgs,
        right_joint_cfgs,
        cam_to_left_base,
        cam_to_right_base,
        intrinsic,
        width = 1280,
        height = 720,
        near_plane = 0.1,
        far_plane = 20.0,
        urdf_file = {
            "left": os.path.join("airexo", "urdf_models", "robot", "left_robot.urdf"),
            "right": os.path.join("airexo", "urdf_models", "robot", "right_robot.urdf")
        },
        **kwargs
    ):
        """
        Parameters:
        - left_joint_cfgs: left joint configurations of robot;
        - right_joint_cfgs: right joint configurations of robot;
        - cam_to_left_base: the transformation matrix between camera and left robot base;
        - cam_to_right_base: the transformation matrix between camera ant right robot base;
        - intrinsic: the camera intrinsic matrix;
        - width, height: the rendered width/height of the images;
        - near_plane, far_plane: the near/far plane of the projection;
        - urdf_file: the urdf file of robot.
        """
        # save arguments
        self.left_joint_cfgs = left_joint_cfgs
        self.right_joint_cfgs = right_joint_cfgs
        self.cam_to_left_base = cam_to_left_base
        self.cam_to_right_base = cam_to_right_base
        self.urdf_file = urdf_file

        # initialize renderer
        self.renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
        self.material = o3d.visualization.rendering.MaterialRecord()
        self.material.shader = "defaultLit" 

        # forward kinematics of both arms
        cur_transforms_left, self.visuals_map_left = robot_helper.forward_kinematic_single(
            joint = np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32),
            joint_cfgs = self.left_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file["left"],
            with_visuals_map = True
        )
        cur_transforms_right, self.visuals_map_right = robot_helper.forward_kinematic_single(
            joint = np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32),
            joint_cfgs = self.right_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file["right"],
            with_visuals_map = True
        )
        self.last_joints = (np.zeros((self.left_joint_cfgs.num_joints, ), dtype = np.float32), np.zeros((self.right_joint_cfgs.num_joints, ), dtype = np.float32))

        # read mesh, add into initial scene
        self.model_meshes_left = {}
        self.last_transforms_left = {}
        for link, transform in cur_transforms_left.items():
            for v in self.visuals_map_left[link]:
                if v.geom_param is None: continue
                mesh_name = "left///{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                mesh = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["left"]), v.geom_param))
                mesh.transform(tf)
                mesh.compute_vertex_normals()
                self.model_meshes_left[mesh_name] = mesh
                self.last_transforms_left[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, mesh, self.material)

        self.model_meshes_right = {}
        self.last_transforms_right = {}
        for link, transform in cur_transforms_right.items():
            for v in self.visuals_map_right[link]:
                if v.geom_param is None: continue
                mesh_name = "right///{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                mesh = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["right"]), v.geom_param))
                mesh.transform(tf)
                mesh.compute_vertex_normals()
                self.model_meshes_right[mesh_name] = mesh
                self.last_transforms_right[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, mesh, self.material)

        # set camera projection
        self.renderer.scene.camera.set_projection(intrinsic, near_plane, far_plane, float(width), float(height))
   
    def _update_geometry(self, joints = None):
        # by default, load last joint information
        if joints is None:
            joints = self.last_joints
        left_joint, right_joint = joints

        # forward kinematics of both arms
        cur_transforms_left = robot_helper.forward_kinematic_single(
            joint = left_joint,
            joint_cfgs = self.left_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file["left"],
            with_visuals_map = False
        )
        cur_transforms_right = robot_helper.forward_kinematic_single(
            joint = right_joint,
            joint_cfgs = self.right_joint_cfgs,
            is_rad = True,
            urdf_file = self.urdf_file["right"],
            with_visuals_map = False
        )

        # update mesh in the scene
        self.renderer.scene.clear_geometry()
        for link, transform in cur_transforms_left.items():
            for v in self.visuals_map_left[link]:
                if v.geom_param is None: continue
                mesh_name = "left///{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                self.model_meshes_left[mesh_name].transform(tf @ np.linalg.inv(self.last_transforms_left[mesh_name]))
                self.model_meshes_left[mesh_name].compute_vertex_normals()
                self.last_transforms_left[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, self.model_meshes_left[mesh_name], self.material)
        
        for link, transform in cur_transforms_right.items():
            for v in self.visuals_map_right[link]:
                if v.geom_param is None: continue
                mesh_name = "right///{}///{}".format(link, v.geom_param)
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                self.model_meshes_right[mesh_name].transform(tf @ np.linalg.inv(self.last_transforms_right[mesh_name]))
                self.model_meshes_right[mesh_name].compute_vertex_normals()
                self.last_transforms_right[mesh_name] = tf
                self.renderer.scene.add_geometry(mesh_name, self.model_meshes_right[mesh_name], self.material)
    
    def update_joints(self, left_joint, right_joint):
        self._update_geometry(joints = (left_joint, right_joint))
    
    def update_camera_to_left_base(self, cam_to_left_base, delay_update = False):
        self.cam_to_left_base = cam_to_left_base
        if not delay_update:
            self._update_geometry()
    
    def update_camera_to_right_base(self, cam_to_right_base, delay_update = False):
        self.cam_to_right_base = cam_to_right_base
        if not delay_update:
            self._update_geometry()

    def render_image(self):
        return np.asarray(self.renderer.render_to_image(), dtype = np.uint8)
    
    def render_depth(self):
        return np.asarray(self.renderer.render_to_depth_image(z_in_view_space = True), dtype = np.float32)
    
    def render_mask(self, depth = None):
        if depth is None:
            depth = self.render_depth()
        mask = np.zeros(depth.shape, dtype = np.uint8)
        mask[depth < np.inf] = 255
        return mask
