#!/usr/bin/env python3
"""
3D visualization utilities for pose estimation results.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .geometry_utils import rpy_to_rot, make_unit_cube, cube_faces


def visualize_pose_cube(pose, cube_size_m=0.15):
    """
    Visualize predicted 6DOF pose as a 3D cube.
    
    Args:
        pose: 6-element array [x, y, z, roll, pitch, yaw]
        cube_size_m: Cube side length in meters (default: 0.15)
    """
    x, y, z, roll, pitch, yaw = [float(v) for v in pose]
    
    # Get rotation matrix
    r = rpy_to_rot(roll, pitch, yaw)
    
    # Generate and transform cube vertices
    verts = make_unit_cube(cube_size_m)
    verts_t = (verts @ r.T) + np.array([x, y, z], dtype=np.float32)
    
    # Create 3D plot
    fig = plt.figure("Predicted Pose")
    ax = fig.add_subplot(111, projection="3d")
    
    # Draw cube faces
    faces = cube_faces(verts_t)
    poly = Poly3DCollection(faces, alpha=0.25, edgecolor="k", linewidths=1.0)
    ax.add_collection3d(poly)
    
    # Draw vertices
    ax.scatter(verts_t[:, 0], verts_t[:, 1], verts_t[:, 2], s=20, c='red')
    
    # Labels and title
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Cube transformed by predicted pose")
    
    # Set equal aspect ratio
    mins = verts_t.min(axis=0)
    maxs = verts_t.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float((maxs - mins).max())
    span = max(span, 0.2)
    
    ax.set_xlim(center[0] - span / 2, center[0] + span / 2)
    ax.set_ylim(center[1] - span / 2, center[1] + span / 2)
    ax.set_zlim(center[2] - span / 2, center[2] + span / 2)
    
    plt.show(block=False)
    plt.pause(0.001)
