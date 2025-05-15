"""
Real-time Visualizers.

Authors: Hongjie Fang
"""

import os
import cv2
import hydra
import numpy as np
import kinpy as kp
import open3d as o3d

from pynput import keyboard
from omegaconf import OmegaConf

import airexo.helpers.urdf_robot as robot_helper
import airexo.helpers.urdf_airexo as airexo_helper

from airexo.helpers.constants import *
from airexo.helpers.transform import transform_arm
from airexo.helpers.rotation import xyz_rot_transform
from airexo.helpers.state import airexo_transform_tcp
from airexo.calibration.calib_info import CalibrationInfo
from airexo.helpers.point_cloud import get_point_cloud_open3d


class AirExoVisualizer:
    """
    Visualize AirExo on the in-the-wild AirExo point cloud.
    """
    def __init__(
        self,
        camera_serial,
        left_airexo,
        right_airexo,
        left_calib_cfgs,
        right_calib_cfgs,
        calib_info: CalibrationInfo,
        urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))
        self.visuals_map = self.model_chain.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_airexo = left_airexo
        self.right_airexo = right_airexo
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs

        # Load calibration result
        self.cam_to_base = calib_info.get_camera_to_base(camera_serial)

        # Visualization loop flag
        self.vis_loop = False

    def _on_press(self, key):
        try:
            if key.char == 'q':
                self.vis_loop = False
        except Exception as e:
            pass

    def _on_release(self, key):
        pass

    def run(self):
        # Visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(width = 1280, height = 720)

        # Set visualization camera
        self.o3d_camera = o3d.camera.PinholeCameraParameters()
        self.o3d_camera.extrinsic = np.eye(4)
        self.o3d_camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,
            height = 720,
            fx = self.intrinsic[0, 0],
            fy = self.intrinsic[1, 1],
            cx = self.intrinsic[0, 2],
            cy = self.intrinsic[1, 2]
        )
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(self.o3d_camera, allow_arbitrary = True)

        # Base axis
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
        axis = axis.transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_base)
        vis.add_geometry(axis, reset_bounding_box = False)

        # Get point cloud (initial frame)
        _, color, depth = self.camera.get_rgbd_images()
        depth = depth.astype(np.float32) / 1000.0
        point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
        point_cloud.transform(O3D_RENDER_TRANSFORMATION)
        vis.add_geometry(point_cloud, reset_bounding_box = True)

        # Get joint info (initial frame)
        left_joint = self.left_airexo.get_angle()
        right_joint = self.right_airexo.get_angle()
        joint_states = airexo_helper.convert_joint_states(
            left_joint = left_joint, 
            right_joint = right_joint,
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False
        )

        # Forward kinematic and render (initial frame)
        cur_transforms = self.model_chain.forward_kinematics(joint_states)
        model_meshes = {}
        last_transforms = {}
        for link, transform in cur_transforms.items():
            model_meshes[link] = {}
            last_transforms[link] = {}
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file), v.geom_param))
                model_meshes[link][v.geom_param].transform(tf)
                last_transforms[link][v.geom_param] = tf
                model_meshes[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes[link][v.geom_param], reset_bounding_box = False)
        
        # Calculate tcp (initial frame)
        left_tcp, right_tcp = airexo_transform_tcp(
            left_joint = left_joint,
            right_joint = right_joint,
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False,
            urdf_file = self.urdf_file,
            real_robot_base = False
        )
        
        # Visualize tcp (initial frame)
        left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ left_tcp
        right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ right_tcp
        left_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(left_tcp)
        right_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(right_tcp)
        vis.add_geometry(left_tcp_vis, reset_bounding_box = False)
        vis.add_geometry(right_tcp_vis, reset_bounding_box = False)
        last_left_tcp, last_right_tcp = left_tcp, right_tcp

        # Initialize keyboard listener
        self.vis_loop = True
        listener = keyboard.Listener(on_press = self._on_press, on_release = self._on_release)
        listener.start()
        
        while self.vis_loop:
            # Get point cloud
            _, color, depth = self.camera.get_rgbd_images()
            depth = depth.astype(np.float32) / 1000.0
            vis.remove_geometry(point_cloud, reset_bounding_box = False)
            point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
            point_cloud.transform(O3D_RENDER_TRANSFORMATION)
            vis.add_geometry(point_cloud, reset_bounding_box = False)

            # Get joint info
            left_joint = self.left_airexo.get_angle()
            right_joint = self.right_airexo.get_angle()
            joint_states = airexo_helper.convert_joint_states(
                left_joint = left_joint, 
                right_joint = right_joint,
                left_joint_cfgs = self.left_airexo.joint_cfgs,
                right_joint_cfgs = self.right_airexo.joint_cfgs,
                left_calib_cfgs = self.left_calib_cfgs,
                right_calib_cfgs = self.right_calib_cfgs,
                is_rad = False
            )

            # Forward kinematic and render
            cur_transforms = self.model_chain.forward_kinematics(joint_states)
            for link, transform in cur_transforms.items():
                for v in self.visuals_map[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                    model_meshes[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms[link][v.geom_param]))
                    last_transforms[link][v.geom_param] = tf
                    model_meshes[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes[link][v.geom_param])

            # Calculate tcp
            left_tcp, right_tcp = airexo_transform_tcp(
                left_joint = left_joint,
                right_joint = right_joint,
                left_joint_cfgs = self.left_airexo.joint_cfgs,
                right_joint_cfgs = self.right_airexo.joint_cfgs,
                left_calib_cfgs = self.left_calib_cfgs,
                right_calib_cfgs = self.right_calib_cfgs,
                is_rad = False,
                urdf_file = self.urdf_file,
                real_robot_base = False
            )
            
            # Visualize tcp
            left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ left_tcp
            right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ right_tcp
            left_tcp_vis.transform(left_tcp @ np.linalg.inv(last_left_tcp))
            right_tcp_vis.transform(right_tcp @ np.linalg.inv(last_right_tcp))
            vis.update_geometry(left_tcp_vis)
            vis.update_geometry(right_tcp_vis)
            last_left_tcp, last_right_tcp = left_tcp, right_tcp

            # Update visualization
            vis.poll_events()
            vis.update_renderer()

        # Clean-ups
        listener.stop()
        vis.destroy_window()
        
    def stop(self):
        self.camera.stop()
        self.left_airexo.stop()
        self.right_airexo.stop()


class AirExoRGBVisualizer:
    """
    Visualize AirExo on the in-the-wild AirExo RGB image.
    """
    def __init__(
        self,
        camera_serial,
        left_airexo,
        right_airexo,
        left_calib_cfgs,
        right_calib_cfgs,
        calib_info: CalibrationInfo,
        urdf_file = os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        alpha = 0.4,
        near_plane = 0.1,
        far_plane = 20.0,
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))
        self.visuals_map = self.model_chain.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_airexo = left_airexo
        self.right_airexo = right_airexo
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs

        # Load calibration result
        self.cam_to_base = calib_info.get_camera_to_base(camera_serial)

        # Load renderer parameters
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.alpha = alpha

    def run(self):
        # Set renderer
        renderer = o3d.visualization.rendering.OffscreenRenderer(1280, 720)
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultLit" 

        # Set camera
        renderer.scene.camera.set_projection(self.intrinsic, self.near_plane, self.far_plane, 1280, 720)

        # Create a named window
        window_name = "Real-Time AirExo Render"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        # Get joint info (initial frame)
        left_joint = self.left_airexo.get_angle()
        right_joint = self.right_airexo.get_angle()
        joint_states = airexo_helper.convert_joint_states(
            left_joint = left_joint, 
            right_joint = right_joint,
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False
        )

        # Forward kinematic and render (initial frame)
        cur_transforms = self.model_chain.forward_kinematics(joint_states)
        model_meshes = {}
        last_transforms = {}
        for link, transform in cur_transforms.items():
            model_meshes[link] = {}
            last_transforms[link] = {}
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file), v.geom_param))
                model_meshes[link][v.geom_param].transform(tf)
                last_transforms[link][v.geom_param] = tf
                model_meshes[link][v.geom_param].compute_vertex_normals()
                mesh_name = "{}///{}".format(link, v.geom_param)
                renderer.scene.add_geometry(mesh_name, model_meshes[link][v.geom_param], material)
        
        while True:
            # Get rgb image
            _, image = self.camera.get_rgb_image()

            # Get joint info
            left_joint = self.left_airexo.get_angle()
            right_joint = self.right_airexo.get_angle()
            joint_states = airexo_helper.convert_joint_states(
                left_joint = left_joint, 
                right_joint = right_joint,
                left_joint_cfgs = self.left_airexo.joint_cfgs,
                right_joint_cfgs = self.right_airexo.joint_cfgs,
                left_calib_cfgs = self.left_calib_cfgs,
                right_calib_cfgs = self.right_calib_cfgs,
                is_rad = False
            )

            # Forward kinematic and render
            cur_transforms = self.model_chain.forward_kinematics(joint_states)
            for link, transform in cur_transforms.items():
                for v in self.visuals_map[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                    model_meshes[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms[link][v.geom_param]))
                    last_transforms[link][v.geom_param] = tf
                    model_meshes[link][v.geom_param].compute_vertex_normals()
                    mesh_name = "{}///{}".format(link, v.geom_param)
                    renderer.scene.remove_geometry(mesh_name)
                    renderer.scene.add_geometry(mesh_name, model_meshes[link][v.geom_param], material)

            # Render RGB-D images
            airexo_image = np.asarray(renderer.render_to_image())
            airexo_depth = np.asarray(renderer.render_to_depth_image(z_in_view_space = True))

            # Generate part-overlapped image
            overlapped_image = airexo_image.astype(np.float32) * self.alpha + image.astype(np.float32) * (1 - self.alpha)
            overlapped_image = overlapped_image.astype(np.uint8)
            final_image = np.where(airexo_depth[:, :, np.newaxis] < np.inf, overlapped_image, image)

            # Update visualizations
            cv2.imshow(window_name, final_image[:, :, ::-1])
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'): 
                break
        
        # clean-ups
        cv2.destroyWindow(window_name)
        
    def stop(self):
        self.camera.stop()
        self.left_airexo.stop()
        self.right_airexo.stop()


class AirExoRobotVisualizer:
    """
    Visualize robot on the in-the-wild AirExo point cloud.
    """
    def __init__(
        self,
        camera_serial,
        left_airexo,
        right_airexo,
        left_robot_cfgs,
        right_robot_cfgs,
        left_calib_cfgs,
        right_calib_cfgs,
        calib_info: CalibrationInfo,
        urdf_file = os.path.join("airexo", "urdf_models", "robot", "robot.urdf"),
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))
        self.visuals_map = self.model_chain.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_airexo = left_airexo
        self.right_airexo = right_airexo
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs
        self.left_robot_cfgs = left_robot_cfgs
        self.right_robot_cfgs = right_robot_cfgs

        # Load calibration result
        self.cam_to_base = calib_info.get_camera_to_base(camera_serial)

        # Visualization loop flag
        self.vis_loop = False

    def _on_press(self, key):
        try:
            if key.char == 'q':
                self.vis_loop = False
        except Exception as e:
            pass

    def _on_release(self, key):
        pass

    def run(self):
        # Visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(width = 1280, height = 720)

        # Set visualization camera
        self.o3d_camera = o3d.camera.PinholeCameraParameters()
        self.o3d_camera.extrinsic = np.eye(4)
        self.o3d_camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,
            height = 720,
            fx = self.intrinsic[0, 0],
            fy = self.intrinsic[1, 1],
            cx = self.intrinsic[0, 2],
            cy = self.intrinsic[1, 2]
        )
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(self.o3d_camera, allow_arbitrary = True)

        # Base axis
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
        axis = axis.transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_base)
        vis.add_geometry(axis, reset_bounding_box = False)

        # Get point cloud (initial frame)
        _, color, depth = self.camera.get_rgbd_images()
        depth = depth.astype(np.float32) / 1000.0
        point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
        point_cloud.transform(O3D_RENDER_TRANSFORMATION)
        vis.add_geometry(point_cloud, reset_bounding_box = True)

        # Get joint info (initial frame)
        left_joint = self.left_airexo.get_angle()
        right_joint = self.right_airexo.get_angle()

        left_robot_joint = transform_arm(
            robot_cfgs = self.left_robot_cfgs,
            airexo_cfgs = self.left_airexo.joint_cfgs,
            calib_cfgs = self.left_calib_cfgs,
            data = left_joint
        )
        right_robot_joint = transform_arm(
            robot_cfgs = self.right_robot_cfgs,
            airexo_cfgs = self.right_airexo.joint_cfgs,
            calib_cfgs = self.right_calib_cfgs,
            data = right_joint
        )

        joint_states = robot_helper.convert_joint_states(
            left_joint = left_robot_joint, 
            right_joint = right_robot_joint,
            left_joint_cfgs = self.left_robot_cfgs,
            right_joint_cfgs = self.right_robot_cfgs,
            is_rad = True
        )

        # Forward kinematic and render (initial frame)
        cur_transforms = self.model_chain.forward_kinematics(joint_states)
        model_meshes = {}
        last_transforms = {}
        for link, transform in cur_transforms.items():
            model_meshes[link] = {}
            last_transforms[link] = {}
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file), v.geom_param))
                model_meshes[link][v.geom_param].transform(tf)
                last_transforms[link][v.geom_param] = tf
                model_meshes[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes[link][v.geom_param], reset_bounding_box = False)
        
        # Initialize keyboard listener
        self.vis_loop = True
        listener = keyboard.Listener(on_press = self._on_press, on_release = self._on_release)
        listener.start()
        
        while self.vis_loop:
            # Get point cloud
            _, color, depth = self.camera.get_rgbd_images()
            depth = depth.astype(np.float32) / 1000.0
            vis.remove_geometry(point_cloud, reset_bounding_box = False)
            point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
            point_cloud.transform(O3D_RENDER_TRANSFORMATION)
            vis.add_geometry(point_cloud, reset_bounding_box = False)

            # Get joint info and transform into robot joint info
            left_joint = self.left_airexo.get_angle()
            right_joint = self.right_airexo.get_angle()
            
            left_robot_joint = transform_arm(
                robot_cfgs = self.left_robot_cfgs,
                airexo_cfgs = self.left_airexo.joint_cfgs,
                calib_cfgs = self.left_calib_cfgs,
                data = left_joint
            )
            right_robot_joint = transform_arm(
                robot_cfgs = self.right_robot_cfgs,
                airexo_cfgs = self.right_airexo.joint_cfgs,
                calib_cfgs = self.right_calib_cfgs,
                data = right_joint
            )

            joint_states = robot_helper.convert_joint_states(
                left_joint = left_robot_joint, 
                right_joint = right_robot_joint,
                left_joint_cfgs = self.left_robot_cfgs,
                right_joint_cfgs = self.right_robot_cfgs,
                is_rad = True
            )

            # Forward kinematic and render
            cur_transforms = self.model_chain.forward_kinematics(joint_states)
            for link, transform in cur_transforms.items():
                for v in self.visuals_map[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                    model_meshes[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms[link][v.geom_param]))
                    last_transforms[link][v.geom_param] = tf
                    model_meshes[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes[link][v.geom_param])

            # Update visualization
            vis.poll_events()
            vis.update_renderer()

        # Clean-ups
        listener.stop()
        vis.destroy_window()
        
    def stop(self):
        self.camera.stop()
        self.left_airexo.stop()
        self.right_airexo.stop()


class AirExoAllVisualizer:
    """
    Visualize both AirExo and robot on the in-the-wild AirExo point cloud.
    """
    def __init__(
        self,
        camera_serial,
        left_airexo,
        right_airexo,
        left_robot_cfgs,
        right_robot_cfgs,
        left_calib_cfgs,
        right_calib_cfgs,
        calib_info: CalibrationInfo,
        urdf_file = {
            "airexo": os.path.join("airexo", "urdf_models", "airexo", "airexo.urdf"),
            "robot": os.path.join("airexo", "urdf_models", "robot", "robot.urdf")
        },
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain_airexo = kp.build_chain_from_urdf(open(urdf_file["airexo"]).read().encode('utf-8'))
        self.visuals_map_airexo = self.model_chain_airexo.visuals_map()
        self.model_chain_robot = kp.build_chain_from_urdf(open(urdf_file["robot"]).read().encode('utf-8'))
        self.visuals_map_robot = self.model_chain_robot.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_airexo = left_airexo
        self.right_airexo = right_airexo
        self.left_calib_cfgs = left_calib_cfgs
        self.right_calib_cfgs = right_calib_cfgs
        self.left_robot_cfgs = left_robot_cfgs
        self.right_robot_cfgs = right_robot_cfgs

        # Load calibration result
        self.cam_to_base = calib_info.get_camera_to_base(camera_serial)

        # Visualization loop flag
        self.vis_loop = False

    def _on_press(self, key):
        try:
            if key.char == 'q':
                self.vis_loop = False
        except Exception as e:
            pass

    def _on_release(self, key):
        pass

    def run(self):
        # Visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(width = 1280, height = 720)

        # Set visualization camera
        o3d_camera = o3d.camera.PinholeCameraParameters()
        o3d_camera.extrinsic = np.eye(4)
        o3d_camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,
            height = 720,
            fx = self.intrinsic[0, 0],
            fy = self.intrinsic[1, 1],
            cx = self.intrinsic[0, 2],
            cy = self.intrinsic[1, 2]
        )
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(o3d_camera, allow_arbitrary = True)

        # Base axis
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
        axis = axis.transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_base)
        vis.add_geometry(axis, reset_bounding_box = False)

        # Get point cloud (initial frame)
        _, color, depth = self.camera.get_rgbd_images()
        depth = depth.astype(np.float32) / 1000.0
        point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
        point_cloud.transform(O3D_RENDER_TRANSFORMATION)
        vis.add_geometry(point_cloud, reset_bounding_box = True)

        # Get joint info (initial frame)
        left_joint = self.left_airexo.get_angle()
        right_joint = self.right_airexo.get_angle()

        joint_states_airexo = airexo_helper.convert_joint_states(
            left_joint = left_joint, 
            right_joint = right_joint,
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False
        )

        left_robot_joint = transform_arm(
            robot_cfgs = self.left_robot_cfgs,
            airexo_cfgs = self.left_airexo.joint_cfgs,
            calib_cfgs = self.left_calib_cfgs,
            data = left_joint
        )
        right_robot_joint = transform_arm(
            robot_cfgs = self.right_robot_cfgs,
            airexo_cfgs = self.right_airexo.joint_cfgs,
            calib_cfgs = self.right_calib_cfgs,
            data = right_joint
        )

        joint_states_robot = robot_helper.convert_joint_states(
            left_joint = left_robot_joint, 
            right_joint = right_robot_joint,
            left_joint_cfgs = self.left_robot_cfgs,
            right_joint_cfgs = self.right_robot_cfgs,
            is_rad = True
        )

        # Forward kinematic and render (initial frame)
        cur_transforms_airexo = self.model_chain_airexo.forward_kinematics(joint_states_airexo)
        model_meshes_airexo = {}
        last_transforms_airexo = {}
        for link, transform in cur_transforms_airexo.items():
            model_meshes_airexo[link] = {}
            last_transforms_airexo[link] = {}
            for v in self.visuals_map_airexo[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes_airexo[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["airexo"]), v.geom_param))
                model_meshes_airexo[link][v.geom_param].transform(tf)
                last_transforms_airexo[link][v.geom_param] = tf
                model_meshes_airexo[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes_airexo[link][v.geom_param], reset_bounding_box = False)
        
        cur_transforms_robot = self.model_chain_robot.forward_kinematics(joint_states_robot)
        model_meshes_robot = {}
        last_transforms_robot = {}
        for link, transform in cur_transforms_robot.items():
            model_meshes_robot[link] = {}
            last_transforms_robot[link] = {}
            for v in self.visuals_map_robot[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes_robot[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["robot"]), v.geom_param))
                model_meshes_robot[link][v.geom_param].transform(tf)
                last_transforms_robot[link][v.geom_param] = tf
                model_meshes_robot[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes_robot[link][v.geom_param], reset_bounding_box = False)
        
        # Calculate tcp (initial frame)
        left_tcp, right_tcp = airexo_transform_tcp(
            left_joint = left_joint,
            right_joint = right_joint,
            left_joint_cfgs = self.left_airexo.joint_cfgs,
            right_joint_cfgs = self.right_airexo.joint_cfgs,
            left_calib_cfgs = self.left_calib_cfgs,
            right_calib_cfgs = self.right_calib_cfgs,
            is_rad = False,
            urdf_file = self.urdf_file["airexo"],
            real_robot_base = False
        )
        
        # Visualize tcp (initial frame)
        left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ left_tcp
        right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ right_tcp
        left_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(left_tcp)
        right_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(right_tcp)
        vis.add_geometry(left_tcp_vis, reset_bounding_box = False)
        vis.add_geometry(right_tcp_vis, reset_bounding_box = False)
        last_left_tcp, last_right_tcp = left_tcp, right_tcp

        # Initialize keyboard listener
        self.vis_loop = True
        listener = keyboard.Listener(on_press = self._on_press, on_release = self._on_release)
        listener.start()
        
        while self.vis_loop:
            # Get point cloud
            _, color, depth = self.camera.get_rgbd_images()
            depth = depth.astype(np.float32) / 1000.0
            vis.remove_geometry(point_cloud, reset_bounding_box = False)
            point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
            point_cloud.transform(O3D_RENDER_TRANSFORMATION)
            vis.add_geometry(point_cloud, reset_bounding_box = False)

            # Get joint info
            left_joint = self.left_airexo.get_angle()
            right_joint = self.right_airexo.get_angle()

            joint_states_airexo = airexo_helper.convert_joint_states(
                left_joint = left_joint, 
                right_joint = right_joint,
                left_joint_cfgs = self.left_airexo.joint_cfgs,
                right_joint_cfgs = self.right_airexo.joint_cfgs,
                left_calib_cfgs = self.left_calib_cfgs,
                right_calib_cfgs = self.right_calib_cfgs,
                is_rad = False
            )
            
            left_robot_joint = transform_arm(
                robot_cfgs = self.left_robot_cfgs,
                airexo_cfgs = self.left_airexo.joint_cfgs,
                calib_cfgs = self.left_calib_cfgs,
                data = left_joint
            )
            right_robot_joint = transform_arm(
                robot_cfgs = self.right_robot_cfgs,
                airexo_cfgs = self.right_airexo.joint_cfgs,
                calib_cfgs = self.right_calib_cfgs,
                data = right_joint
            )

            joint_states_robot = robot_helper.convert_joint_states(
                left_joint = left_robot_joint, 
                right_joint = right_robot_joint,
                left_joint_cfgs = self.left_robot_cfgs,
                right_joint_cfgs = self.right_robot_cfgs,
                is_rad = True
            )

            # Forward kinematic and render
            cur_transforms_airexo = self.model_chain_airexo.forward_kinematics(joint_states_airexo)
            for link, transform in cur_transforms_airexo.items():
                for v in self.visuals_map_airexo[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()                
                    model_meshes_airexo[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms_airexo[link][v.geom_param]))
                    last_transforms_airexo[link][v.geom_param] = tf
                    model_meshes_airexo[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes_airexo[link][v.geom_param])
            
            cur_transforms_robot = self.model_chain_robot.forward_kinematics(joint_states_robot)
            for link, transform in cur_transforms_robot.items():
                for v in self.visuals_map_robot[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix() 
                    model_meshes_robot[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms_robot[link][v.geom_param]))
                    last_transforms_robot[link][v.geom_param] = tf
                    model_meshes_robot[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes_robot[link][v.geom_param])

            # Calculate tcp
            left_tcp, right_tcp = airexo_transform_tcp(
                left_joint = left_joint,
                right_joint = right_joint,
                left_joint_cfgs = self.left_airexo.joint_cfgs,
                right_joint_cfgs = self.right_airexo.joint_cfgs,
                left_calib_cfgs = self.left_calib_cfgs,
                right_calib_cfgs = self.right_calib_cfgs,
                is_rad = False,
                urdf_file = self.urdf_file["airexo"],
                real_robot_base = False
            )
            
            # Visualize tcp
            left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ left_tcp
            right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ right_tcp
            left_tcp_vis.transform(left_tcp @ np.linalg.inv(last_left_tcp))
            right_tcp_vis.transform(right_tcp @ np.linalg.inv(last_right_tcp))
            vis.update_geometry(left_tcp_vis)
            vis.update_geometry(right_tcp_vis)
            last_left_tcp, last_right_tcp = left_tcp, right_tcp

            # Update visualization
            vis.poll_events()
            vis.update_renderer()

        # Clean-ups
        listener.stop()
        vis.destroy_window()
        

    def stop(self):
        self.camera.stop()
        self.left_airexo.stop()
        self.right_airexo.stop()


class RobotVisualizer:
    """
    Visualize robot on the robot point cloud.
    """
    def __init__(
        self,
        camera_serial,
        left_robot,
        right_robot,
        calib_info: CalibrationInfo,
        urdf_file = os.path.join("airexo", "urdf_models", "robot", "robot.urdf"),
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain = kp.build_chain_from_urdf(open(urdf_file).read().encode('utf-8'))
        self.visuals_map = self.model_chain.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_robot = left_robot
        self.right_robot = right_robot

        # Load calibration result
        self.cam_to_base = calib_info.get_camera_to_base(camera_serial, real_base = False)

        # Visualization loop flag
        self.vis_loop = False

    def _on_press(self, key):
        try:
            if key.char == 'q':
                self.vis_loop = False
        except Exception as e:
            pass

    def _on_release(self, key):
        pass

    def run(self):
        # Visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(width = 1280, height = 720)

        # Set visualization camera
        o3d_camera = o3d.camera.PinholeCameraParameters()
        o3d_camera.extrinsic = np.eye(4)
        o3d_camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,
            height = 720,
            fx = self.intrinsic[0, 0],
            fy = self.intrinsic[1, 1],
            cx = self.intrinsic[0, 2],
            cy = self.intrinsic[1, 2]
        )
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(o3d_camera, allow_arbitrary = True)

        # Base axis
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
        axis = axis.transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_base)
        vis.add_geometry(axis, reset_bounding_box = False)

        # Get point cloud (initial frame)
        _, color, depth = self.camera.get_rgbd_images()
        depth = depth.astype(np.float32) / 1000.0
        point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
        point_cloud.transform(O3D_RENDER_TRANSFORMATION)
        vis.add_geometry(point_cloud, reset_bounding_box = True)

        # Get joint info (initial frame)
        left_joint = self.left_robot.get_joint_pos()
        left_gripper = self.left_robot.gripper.get_width()
        right_joint = self.right_robot.get_joint_pos()
        right_gripper = self.right_robot.gripper.get_width()
        joint_states = robot_helper.convert_joint_states(
            left_joint = np.concatenate([left_joint, [left_gripper]], axis = -1), 
            right_joint = np.concatenate([right_joint, [right_gripper]], axis = -1),
            left_joint_cfgs = self.left_robot.joint_cfgs,
            right_joint_cfgs = self.right_robot.joint_cfgs,
            is_rad = True
        )

        # Forward kinematic and render (initial frame)
        cur_transforms = self.model_chain.forward_kinematics(joint_states)
        model_meshes = {}
        last_transforms = {}
        for link, transform in cur_transforms.items():
            model_meshes[link] = {}
            last_transforms[link] = {}
            for v in self.visuals_map[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file), v.geom_param))
                model_meshes[link][v.geom_param].transform(tf)
                last_transforms[link][v.geom_param] = tf
                model_meshes[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes[link][v.geom_param], reset_bounding_box = False)
        
        # Get tcp (initialize frame)
        left_tcp = xyz_rot_transform(
            self.left_robot.get_tcp_pose(), 
            from_rep = "quaternion",
            to_rep = "matrix"
        )
        right_tcp = xyz_rot_transform(
            self.right_robot.get_tcp_pose(),
            from_rep = "quaternion",
            to_rep = "matrix"
        )

        # Visualize tcp (initial frame)
        left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ left_tcp
        right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ right_tcp
        left_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(left_tcp)
        right_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(right_tcp)
        vis.add_geometry(left_tcp_vis, reset_bounding_box = False)
        vis.add_geometry(right_tcp_vis, reset_bounding_box = False)
        last_left_tcp, last_right_tcp = left_tcp, right_tcp

        # Initialize keyboard listener
        self.vis_loop = True
        listener = keyboard.Listener(on_press = self._on_press, on_release = self._on_release)
        listener.start()
        
        while self.vis_loop:
            # Get point cloud
            _, color, depth = self.camera.get_rgbd_images()
            depth = depth.astype(np.float32) / 1000.0
            vis.remove_geometry(point_cloud, reset_bounding_box = False)
            point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
            point_cloud.transform(O3D_RENDER_TRANSFORMATION)
            vis.add_geometry(point_cloud, reset_bounding_box = False)

            # Get joint info
            left_joint = self.left_robot.get_joint_pos()
            left_gripper = self.left_robot.gripper.get_width()
            right_joint = self.right_robot.get_joint_pos()
            right_gripper = self.right_robot.gripper.get_width()
            joint_states = robot_helper.convert_joint_states(
                left_joint = np.concatenate([left_joint, [left_gripper]], axis = -1), 
                right_joint = np.concatenate([right_joint, [right_gripper]], axis = -1),
                left_joint_cfgs = self.left_robot.joint_cfgs,
                right_joint_cfgs = self.right_robot.joint_cfgs,
                is_rad = True
            )

            # Forward kinematic and render
            cur_transforms = self.model_chain.forward_kinematics(joint_states)
            for link, transform in cur_transforms.items():
                for v in self.visuals_map[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                    model_meshes[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms[link][v.geom_param]))
                    last_transforms[link][v.geom_param] = tf
                    model_meshes[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes[link][v.geom_param])
            
            # Get tcp
            left_tcp = xyz_rot_transform(
                self.left_robot.get_tcp_pose(), 
                from_rep = "quaternion",
                to_rep = "matrix"
            )
            right_tcp = xyz_rot_transform(
                self.right_robot.get_tcp_pose(),
                from_rep = "quaternion",
                to_rep = "matrix"
            )

            # Visualize tcp
            left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ left_tcp
            right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_base @ ROBOT_PREDEFINED_TRANSFORMATION @ right_tcp
            left_tcp_vis.transform(left_tcp @ np.linalg.inv(last_left_tcp))
            right_tcp_vis.transform(right_tcp @ np.linalg.inv(last_right_tcp))
            vis.update_geometry(left_tcp_vis)
            vis.update_geometry(right_tcp_vis)
            last_left_tcp, last_right_tcp = left_tcp, right_tcp

            # Update visualization
            vis.poll_events()
            vis.update_renderer()

        listener.stop()
        vis.destroy_window()
    
    def stop(self):
        self.camera.stop()
        self.left_robot.stop()
        self.right_robot.stop()


class RobotSeperateVisualizer:
    """
    Visualize single robot on the robot point cloud.
    """
    def __init__(
        self,
        camera_serial,
        left_robot,
        right_robot,
        calib_info: CalibrationInfo,
        urdf_file = {
            "left": os.path.join("airexo", "urdf_models", "robot", "left_robot.urdf"),
            "right": os.path.join("airexo", "urdf_models", "robot", "right_robot.urdf")
        },
        config_camera_path = os.path.join("airexo", "configs", "cameras"),
        **kwargs
    ):
        # Load urdf model chain
        self.urdf_file = urdf_file
        self.model_chain_left = kp.build_chain_from_urdf(open(urdf_file["left"]).read().encode('utf-8'))
        self.visuals_map_left = self.model_chain_left.visuals_map()
        self.model_chain_right = kp.build_chain_from_urdf(open(urdf_file["right"]).read().encode('utf-8'))
        self.visuals_map_right = self.model_chain_right.visuals_map()

        # Initialize hardwares: camera, airexo
        camera_cfg = OmegaConf.load(os.path.join(config_camera_path, "{}.yaml".format(camera_serial)))
        self.camera = hydra.utils.instantiate(camera_cfg)
        self.intrinsic = self.camera.get_intrinsic()
        self.left_robot = left_robot
        self.right_robot = right_robot

        # Load calibration result
        self.cam_to_left_base = calib_info.get_camera_to_robot_left_base(camera_serial, real_base = False)
        self.cam_to_right_base = calib_info.get_camera_to_robot_right_base(camera_serial, real_base = False)

        # Visualization loop flag
        self.vis_loop = False

    def _on_press(self, key):
        try:
            if key.char == 'q':
                self.vis_loop = False
        except Exception as e:
            pass

    def _on_release(self, key):
        pass

    def run(self):
        # Visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(width = 1280, height = 720)

        # Set visualization camera
        o3d_camera = o3d.camera.PinholeCameraParameters()
        o3d_camera.extrinsic = np.eye(4)
        o3d_camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,
            height = 720,
            fx = self.intrinsic[0, 0],
            fy = self.intrinsic[1, 1],
            cx = self.intrinsic[0, 2],
            cy = self.intrinsic[1, 2]
        )
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(o3d_camera, allow_arbitrary = True)

        # Base axis
        left_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1).transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION)
        vis.add_geometry(left_axis, reset_bounding_box = False)

        right_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1).transform(O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION)
        vis.add_geometry(right_axis, reset_bounding_box = False)

        # Get point cloud (initial frame)
        _, color, depth = self.camera.get_rgbd_images()
        depth = depth.astype(np.float32) / 1000.0
        point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
        point_cloud.transform(O3D_RENDER_TRANSFORMATION)
        vis.add_geometry(point_cloud, reset_bounding_box = True)

        # Get joint info (initial frame)
        left_joint = self.left_robot.get_joint_pos()
        left_gripper = self.left_robot.gripper.get_width() 
        right_joint = self.right_robot.get_joint_pos()
        right_gripper = self.right_robot.gripper.get_width()
        left_joint_states, right_joint_states = robot_helper.convert_joint_states(
            left_joint = np.concatenate([left_joint, [left_gripper]], axis = -1), 
            right_joint = np.concatenate([right_joint, [right_gripper]], axis = -1),
            left_joint_cfgs = self.left_robot.joint_cfgs,
            right_joint_cfgs = self.right_robot.joint_cfgs,
            is_rad = True,
            seperate = True
        )

        # Forward kinematic and render (initial frame)
        cur_transforms_left = self.model_chain_left.forward_kinematics(left_joint_states)
        model_meshes_left = {}
        last_transforms_left = {}
        for link, transform in cur_transforms_left.items():
            model_meshes_left[link] = {}
            last_transforms_left[link] = {}
            for v in self.visuals_map_left[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes_left[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["left"]), v.geom_param))
                model_meshes_left[link][v.geom_param].transform(tf)
                last_transforms_left[link][v.geom_param] = tf
                model_meshes_left[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes_left[link][v.geom_param], reset_bounding_box = False)
        
        cur_transforms_right = self.model_chain_right.forward_kinematics(right_joint_states)
        model_meshes_right = {}
        last_transforms_right = {}
        for link, transform in cur_transforms_right.items():
            model_meshes_right[link] = {}
            last_transforms_right[link] = {}
            for v in self.visuals_map_right[link]:
                if v.geom_param is None: continue
                tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                model_meshes_right[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(self.urdf_file["right"]), v.geom_param))
                model_meshes_right[link][v.geom_param].transform(tf)
                last_transforms_right[link][v.geom_param] = tf
                model_meshes_right[link][v.geom_param].compute_vertex_normals()
                vis.add_geometry(model_meshes_right[link][v.geom_param], reset_bounding_box = False)
        
        # Get tcp (initialize frame)
        left_tcp = xyz_rot_transform(
            self.left_robot.get_tcp_pose(), 
            from_rep = "quaternion",
            to_rep = "matrix"
        )
        right_tcp = xyz_rot_transform(
            self.right_robot.get_tcp_pose(),
            from_rep = "quaternion",
            to_rep = "matrix"
        )

        # Visualize tcp (initial frame)
        left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ left_tcp
        right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ right_tcp
        left_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(left_tcp)
        right_tcp_vis = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05).transform(right_tcp)
        vis.add_geometry(left_tcp_vis, reset_bounding_box = False)
        vis.add_geometry(right_tcp_vis, reset_bounding_box = False)
        last_left_tcp, last_right_tcp = left_tcp, right_tcp

        # Initialize keyboard listener
        self.vis_loop = True
        listener = keyboard.Listener(on_press = self._on_press, on_release = self._on_release)
        listener.start()
        
        while self.vis_loop:
            # Get point cloud
            _, color, depth = self.camera.get_rgbd_images()
            depth = depth.astype(np.float32) / 1000.0
            vis.remove_geometry(point_cloud, reset_bounding_box = False)
            point_cloud = get_point_cloud_open3d(color, depth, self.intrinsic)
            point_cloud.transform(O3D_RENDER_TRANSFORMATION)
            vis.add_geometry(point_cloud, reset_bounding_box = False)

            # Get joint info
            left_joint = self.left_robot.get_joint_pos()
            left_gripper = self.left_robot.gripper.get_width() 
            right_joint = self.right_robot.get_joint_pos()
            right_gripper = self.right_robot.gripper.get_width()
            left_joint_states, right_joint_states = robot_helper.convert_joint_states(
                left_joint = np.concatenate([left_joint, [left_gripper]], axis = -1), 
                right_joint = np.concatenate([right_joint, [right_gripper]], axis = -1),
                left_joint_cfgs = self.left_robot.joint_cfgs,
                right_joint_cfgs = self.right_robot.joint_cfgs,
                is_rad = True,
                seperate = True
            )

            # Forward kinematic and render
            cur_transforms_left = self.model_chain_left.forward_kinematics(left_joint_states)
            for link, transform in cur_transforms_left.items():
                for v in self.visuals_map_left[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ LEFT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                    model_meshes_left[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms_left[link][v.geom_param]))
                    last_transforms_left[link][v.geom_param] = tf
                    model_meshes_left[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes_left[link][v.geom_param])

            cur_transforms_right = self.model_chain_right.forward_kinematics(right_joint_states)
            for link, transform in cur_transforms_right.items():
                for v in self.visuals_map_right[link]:
                    if v.geom_param is None: continue
                    tf = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ RIGHT_ROBOT_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
                    model_meshes_right[link][v.geom_param].transform(tf @ np.linalg.inv(last_transforms_right[link][v.geom_param]))
                    last_transforms_right[link][v.geom_param] = tf
                    model_meshes_right[link][v.geom_param].compute_vertex_normals()
                    vis.update_geometry(model_meshes_right[link][v.geom_param])
            
            # Get tcp
            left_tcp = xyz_rot_transform(
                self.left_robot.get_tcp_pose(), 
                from_rep = "quaternion",
                to_rep = "matrix"
            )
            right_tcp = xyz_rot_transform(
                self.right_robot.get_tcp_pose(),
                from_rep = "quaternion",
                to_rep = "matrix"
            )

            # Visualize tcp
            left_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_left_base @ ROBOT_PREDEFINED_TRANSFORMATION @ left_tcp
            right_tcp = O3D_RENDER_TRANSFORMATION @ self.cam_to_right_base @ ROBOT_PREDEFINED_TRANSFORMATION @ right_tcp
            left_tcp_vis.transform(left_tcp @ np.linalg.inv(last_left_tcp))
            right_tcp_vis.transform(right_tcp @ np.linalg.inv(last_right_tcp))
            vis.update_geometry(left_tcp_vis)
            vis.update_geometry(right_tcp_vis)
            last_left_tcp, last_right_tcp = left_tcp, right_tcp

            # Update visualization
            vis.poll_events()
            vis.update_renderer()

        listener.stop()
        vis.destroy_window()
    
    def stop(self):
        self.camera.stop()
        self.left_robot.stop()
        self.right_robot.stop()
