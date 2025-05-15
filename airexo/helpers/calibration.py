"""
Calibration Functions.

Authors: Hongjie Fang, Hao-Shu Fang.
"""

import cv2
import numpy as np

from cv2 import aruco

from airexo.helpers.rotation import xyz_rot_to_mat


def intrinsics_param2mat(fx, fy, cx, cy):
    """
    Construct intrinsic matrix from parameters.
    """
    return np.array([
        [fx, 0., cx],
        [0., fy, cy],
        [0., 0., 1.]
    ])

def intrinsics_mat2param(camera_intrinsic):
    """
    Extract intrinsic parameters from matrix.
    Return (fx, fy, cx [ppx], cy [ppy])
    """
    camera_intrinsic = np.array(camera_intrinsic).reshape(3, 3)
    return camera_intrinsic[0, 0], camera_intrinsic[1, 1], camera_intrinsic[0, 2], camera_intrinsic[1, 2]

def aruco_detector(
    img,
    aruco_dict,
    marker_length,
    camera_intrinsic,
    vis = True
):
    """
    Args:
    - img: image in BGR format;
    - aruco_dict: aruco dict config;
    - camera_intrinsic: camera intrinsics;
    - vis: whether to enable detection visualization.

    Returns: a dict includes all detected aruco marker pose.
    """
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
    aruco_params = aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
    aruco_params.cornerRefinementWinSize = 5
    dist_coeffs = np.array([[0., 0., 0., 0.]])

    img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = aruco.detectMarkers(
        img_gray,
        aruco_dict,
        parameters = aruco_params
    )

    if ids is None:
        if vis:
            cv2.imshow("Detected markers", img)
            cv2.waitKey(0)
        return {}
    
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
        corners,
        marker_length,
        camera_intrinsic,
        dist_coeffs
    )

    matrices = {}
    for idx, (t, r) in enumerate(zip(tvec, rvec)):
        mat = xyz_rot_to_mat(
            np.concatenate([
                np.array(t).reshape(3) / 1000,
                np.array(r).reshape(3)
            ], axis = -1),
            rotation_rep = "axis_angle"
        )
        aruco_idx = ids[idx][0]
        matrices[aruco_idx] = mat
    
    if vis:
        draw_img = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
        for (t, r) in zip(tvec, rvec):
            draw_img = aruco.drawAxis(draw_img, camera_intrinsic, dist_coeffs, r, t, 100)
        cv2.imshow("Detected markers", draw_img[:, :, ::-1])
        cv2.waitKey(0)

    return matrices
