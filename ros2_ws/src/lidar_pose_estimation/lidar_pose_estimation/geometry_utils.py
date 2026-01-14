#!/usr/bin/env python3
"""
Geometry utilities for 3D transformations and cube generation.
Used for pose visualization.
"""

import math
import numpy as np


def rpy_to_rot(roll, pitch, yaw):
    """
    Convert roll-pitch-yaw angles to rotation matrix.
    
    Args:
        roll: Rotation around X-axis (radians)
        pitch: Rotation around Y-axis (radians)
        yaw: Rotation around Z-axis (radians)
        
    Returns:
        3x3 rotation matrix
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,    0, 1]], dtype=np.float32)
    ry = np.array([[cp, 0, sp],
                   [0,  1, 0],
                   [-sp, 0, cp]], dtype=np.float32)
    rx = np.array([[1, 0,  0],
                   [0, cr, -sr],
                   [0, sr,  cr]], dtype=np.float32)

    return (rz @ ry @ rx).astype(np.float32)


def make_unit_cube(size_m=0.15):
    """
    Generate vertices of a cube centered at origin.
    
    Args:
        size_m: Cube side length in meters (default: 0.15)
        
    Returns:
        8x3 array of cube vertices
    """
    s = size_m / 2.0
    return np.array([
        [-s, -s, -s],  # 0: bottom-front-left
        [ s, -s, -s],  # 1: bottom-front-right
        [ s,  s, -s],  # 2: bottom-back-right
        [-s,  s, -s],  # 3: bottom-back-left
        [-s, -s,  s],  # 4: top-front-left
        [ s, -s,  s],  # 5: top-front-right
        [ s,  s,  s],  # 6: top-back-right
        [-s,  s,  s],  # 7: top-back-left
    ], dtype=np.float32)


def cube_faces(vertices):
    """
    Generate face definitions from cube vertices.
    
    Args:
        vertices: 8x3 array of cube corners
        
    Returns:
        List of 6 faces, each face is 4 vertices
    """
    v = vertices
    return [
        [v[0], v[1], v[2], v[3]],  # Bottom
        [v[4], v[5], v[6], v[7]],  # Top
        [v[0], v[1], v[5], v[4]],  # Front
        [v[1], v[2], v[6], v[5]],  # Right
        [v[2], v[3], v[7], v[6]],  # Back
        [v[3], v[0], v[4], v[7]],  # Left
    ]


def make_train_x_header(num_slices=15):
    """
    Generate CSV header for training data features.
    
    Args:
        num_slices: Number of slices (default: 15)
        
    Returns:
        List of column names
    """
    cols = []
    for i in range(num_slices):
        cols += [
            f"s{i}_z",
            f"s{i}_n_points",
            f"s{i}_x_min", f"s{i}_x_max", f"s{i}_x_mean", f"s{i}_x_std",
            f"s{i}_y_min", f"s{i}_y_max", f"s{i}_y_mean", f"s{i}_y_std",
        ]
    return cols
