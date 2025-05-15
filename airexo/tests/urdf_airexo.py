import os
import hydra
import numpy as np
import open3d as o3d

from omegaconf import OmegaConf

from airexo.helpers.constants import *
from airexo.helpers.urdf_airexo import forward_kinematic

@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "tests", "urdf"),
    config_name = "airexo"
)
def main(cfg):
    OmegaConf.resolve(cfg)

    cur_transforms, visuals_map = forward_kinematic(
        left_joint = cfg.left_joint,
        right_joint = cfg.right_joint,
        left_joint_cfgs = cfg.airexo_left_joint_cfgs,
        right_joint_cfgs = cfg.airexo_right_joint_cfgs,
        left_calib_cfgs = cfg.left_calib_cfgs,
        right_calib_cfgs = cfg.right_calib_cfgs,
        is_rad = False,
        urdf_file = cfg.urdf_file,
        with_visuals_map = True
    )

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(width = 1280, height = 720)

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
    visualizer.add_geometry(frame)

    model_meshes = {}
    for link, transform in cur_transforms.items():
        model_meshes[link] = {}
        for v in visuals_map[link]:
            if v.geom_param is None: continue
            tf = AIREXO_PREDEFINED_TRANSFORMATION @ transform.matrix() @ v.offset.matrix()
            model_meshes[link][v.geom_param] = o3d.io.read_triangle_mesh(os.path.join(os.path.dirname(cfg.urdf_file), v.geom_param))
            model_meshes[link][v.geom_param].transform(tf)
            model_meshes[link][v.geom_param].compute_vertex_normals()
            visualizer.add_geometry(model_meshes[link][v.geom_param], reset_bounding_box = True)

    visualizer.run()

if __name__ == '__main__':
    main()