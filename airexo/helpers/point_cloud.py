"""
Point Cloud Utilities.

Authors: Hongjie Fang.
"""

import numpy as np
import open3d as o3d


def get_point_cloud_open3d(color, depth, camera_intrinsics, use_mask = True):
    """
    Given the depth image, return the point cloud in open3d format.
    """
    d = depth.copy()
    c = color.copy() / 255
    
    fx, fy = camera_intrinsics[0, 0], camera_intrinsics[1, 1]
    cx, cy = camera_intrinsics[0, 2], camera_intrinsics[1, 2]

    xmap, ymap = np.arange(d.shape[1]), np.arange(d.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)

    points_z = d
    points_x = (xmap - cx) / fx * points_z
    points_y = (ymap - cy) / fy * points_z
    points = np.stack([points_x, points_y, points_z], axis = -1)

    if use_mask:
        mask = ((points_z > 0) & (points_z < 1.6))
        points = points[mask]
        c = c[mask]
    else:
        points = points.reshape((-1, 3))
        c = c.reshape((-1, 3))
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(c)
    return cloud