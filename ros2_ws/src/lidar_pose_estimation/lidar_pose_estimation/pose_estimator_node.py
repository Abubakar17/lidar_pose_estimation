#!/usr/bin/env python3
"""
Main ROS2 node for 6DOF pose estimation with LiDAR and conveyor.
Captures vertical slices, builds 3D point cloud, and predicts pose using ONNX model.
"""

import os
import csv
import threading
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

from .inference_onnx import PoseEstimator
from .conveyor_control import ConveyorControl
from .geometry_utils import make_train_x_header
from .visualization import visualize_pose_cube


class PoseEstimatorNode(Node):
    """ROS2 node for real-time 6DOF pose estimation"""
    
    def __init__(self):
        super().__init__("pose_estimator_node")
        
        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Load parameters
        self._load_parameters()
        
        # Initialize AI model
        self.estimator = PoseEstimator(self.model_dir, use_gpu=False)
        self.get_logger().info(f"Loaded ONNX model from: {self.model_dir} (CPU)")
        
        # Initialize conveyor control
        self._initialize_conveyor()
        
        # Initialize data storage
        self.latest_scan = None
        self.slice_idx = 0
        self.slice_points_zy = []
        self.all_cloud_points = []
        
        # Setup ROS2 publishers and subscribers
        self._setup_ros_interfaces()
        
        # Ensure CSV files exist
        self._ensure_csv_headers()
        
        # Start keyboard input thread
        t = threading.Thread(target=self.keyboard_loop, daemon=True)
        t.start()
        
        self.get_logger().info("PoseEstimatorNode ready!")
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters"""
        # I/O topics
        self.declare_parameter("input_topic", "/scan_filtered")
        self.declare_parameter("output_cloud_topic", "/cloud_3d")
        
        # Capture parameters
        self.declare_parameter("num_slices", 15)
        self.declare_parameter("slice_step_m", 0.01)
        self.declare_parameter("z0", -0.10)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 10.0)
        
        # File paths
        self.declare_parameter("output_dir", os.path.expanduser("~/iscf/data/captures"))
        self.declare_parameter("captures_x_name", "captures_X.csv")
        self.declare_parameter("captures_y_name", "captures_Y.csv")
        self.declare_parameter("model_dir", os.path.expanduser("~/iscf/model/exports"))
        
        # Conveyor parameters
        self.declare_parameter("conveyor_port", "/dev/ttyUSB1")
        self.declare_parameter("conveyor_servo_id", 5)
        self.declare_parameter("conveyor_degrees_per_cm", 22.0)
        self.declare_parameter("conveyor_step_cm", 1.0)
    
    def _load_parameters(self):
        """Load all parameters into instance variables"""
        self.input_topic = self.get_parameter("input_topic").value
        self.output_cloud_topic = self.get_parameter("output_cloud_topic").value
        self.num_slices = int(self.get_parameter("num_slices").value)
        self.slice_step_m = float(self.get_parameter("slice_step_m").value)
        self.z0 = float(self.get_parameter("z0").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        
        self.output_dir = self.get_parameter("output_dir").value
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.captures_x_path = os.path.join(
            self.output_dir, self.get_parameter("captures_x_name").value
        )
        self.captures_y_path = os.path.join(
            self.output_dir, self.get_parameter("captures_y_name").value
        )
        
        self.model_dir = self.get_parameter("model_dir").value
        self.conveyor_step_cm = float(self.get_parameter("conveyor_step_cm").value)
    
    def _initialize_conveyor(self):
        """Initialize conveyor control"""
        conveyor_port = self.get_parameter("conveyor_port").value
        conveyor_id = int(self.get_parameter("conveyor_servo_id").value)
        degrees_per_cm = float(self.get_parameter("conveyor_degrees_per_cm").value)
        
        try:
            self.conveyor = ConveyorControl(
                port=conveyor_port,
                servo_id=conveyor_id,
                degrees_per_cm=degrees_per_cm
            )
            self.get_logger().info(f"Conveyor initialized: step={self.conveyor_step_cm} cm")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize conveyor: {e}")
            self.conveyor = None
    
    def _setup_ros_interfaces(self):
        """Setup ROS2 publishers and subscribers"""
        # Subscribe to filtered LiDAR scan
        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.on_scan,
            qos_profile=qos_profile_sensor_data,
        )
        
        # Publish 3D point cloud for RViz
        self.pub_cloud = self.create_publisher(
            PointCloud2,
            self.output_cloud_topic,
            10
        )
        
        self.get_logger().info(
            f"ROS Interfaces:\n"
            f"  Sub: {self.input_topic}\n"
            f"  Pub: {self.output_cloud_topic}\n"
            f"  Slices: {self.num_slices}, Step: {self.slice_step_m}m\n"
            f"  Conveyor: {self.conveyor_step_cm} cm/slice"
        )
    
    def _ensure_csv_headers(self):
        """Create CSV files with headers if they don't exist"""
        if not os.path.exists(self.captures_x_path) or os.path.getsize(self.captures_x_path) == 0:
            header_x = make_train_x_header(self.num_slices)
            with open(self.captures_x_path, "w", newline="") as f:
                csv.writer(f).writerow(header_x)
            self.get_logger().info("Created captures_X.csv header")
        
        if not os.path.exists(self.captures_y_path) or os.path.getsize(self.captures_y_path) == 0:
            header_y = ["x", "y", "z", "roll", "pitch", "yaw"]
            with open(self.captures_y_path, "w", newline="") as f:
                csv.writer(f).writerow(header_y)
            self.get_logger().info("Created captures_Y.csv header")
    
    def on_scan(self, msg):
        """Callback for LiDAR scan messages"""
        self.latest_scan = msg
    
    def keyboard_loop(self):
        """Interactive keyboard input loop"""
        print("\nCommands:")
        print("  ENTER  - Capture slice + move conveyor 1cm")
        print("  s      - Save session and predict pose")
        print("  r      - Reset session")
        print()
        
        while rclpy.ok():
            cmd = input(f"[slice {self.slice_idx}/{self.num_slices}] > ").strip().lower()
            
            if cmd == "r":
                self.reset_session()
            elif cmd == "s":
                self.save_and_predict()
            else:
                self.capture_slice_and_move()
    
    def reset_session(self):
        """Reset capture session and clear point cloud"""
        self.slice_idx = 0
        self.slice_points_zy = []
        self.all_cloud_points = []
        
        # Publish empty cloud to clear RViz
        self._publish_empty_cloud()
        print("Session reset. PointCloud cleared.")
    
    def capture_slice_and_move(self):
        """Capture current LiDAR slice and move conveyor"""
        if self.latest_scan is None:
            print("⚠️  No scan received yet.")
            return
        
        if self.slice_idx >= self.num_slices:
            print("✅ All slices captured. Press 's' to save, or 'r' to reset.")
            return
        
        # Extract 2D slice
        z_vals, y_vals = self._scan_to_zy(self.latest_scan)
        self.slice_points_zy.append((z_vals, y_vals))
        
        # Add to 3D point cloud
        slice_z = self.z0 + self.slice_idx * self.slice_step_m
        for i in range(len(z_vals)):
            self.all_cloud_points.append([
                float(z_vals[i]),
                float(y_vals[i]),
                slice_z
            ])
        
        self.slice_idx += 1
        print(f"📸 Slice {self.slice_idx}/{self.num_slices}: {len(z_vals)} points")
        
        # Publish updated point cloud
        self._publish_pointcloud()
        
        # Move conveyor (skip on last slice)
        if self.slice_idx < self.num_slices and self.conveyor is not None:
            print(f"🔄 Moving conveyor {self.conveyor_step_cm} cm...")
            success = self.conveyor.move_cm(self.conveyor_step_cm)
            if success:
                print("✅ Ready for next slice")
            else:
                print("⚠️  Conveyor movement failed")
        
        if self.slice_idx == self.num_slices:
            print("\n🎯 Session complete! Press 's' to save and predict.")
    
    def save_and_predict(self):
        """Save captured data and predict pose"""
        if self.slice_idx != self.num_slices:
            print(f"⚠️  Not ready: {self.slice_idx}/{self.num_slices} slices captured")
            return
        
        # Compute features
        features_row = self._compute_features()
        
        # Save features to CSV
        with open(self.captures_x_path, "a", newline="") as f:
            csv.writer(f).writerow(features_row)
        print(f"💾 Saved features → {self.captures_x_path}")
        
        # Predict pose
        features = np.array(features_row, dtype=np.float32)
        pose = self.estimator.predict(features)
        
        # Display results
        print("\n" + "="*50)
        print("🎯 POSE PREDICTION")
        print("="*50)
        print(f"Position: x={pose[0]:.4f} m, y={pose[1]:.4f} m, z={pose[2]:.4f} m")
        print(f"Angles  : roll={np.degrees(pose[3]):.2f}°, "
              f"pitch={np.degrees(pose[4]):.2f}°, yaw={np.degrees(pose[5]):.2f}°")
        print("="*50 + "\n")
        
        # Save pose to CSV
        pose_row = [float(p) for p in pose]
        with open(self.captures_y_path, "a", newline="") as f:
            csv.writer(f).writerow(pose_row)
        print(f"💾 Saved pose → {self.captures_y_path}")
        
        # Optional visualization
        ans = input("\nVisualize predicted pose? (y/n): ").strip().lower()
        if ans in ("y", "yes"):
            visualize_pose_cube(pose, cube_size_m=0.15)
        
        self.reset_session()
    
    def _scan_to_zy(self, scan):
        """Convert LaserScan to 2D points in Z-Y plane"""
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        
        valid = (
            np.isfinite(ranges) &
            (ranges >= self.min_range) &
            (ranges <= self.max_range) &
            (ranges > 0.0)
        )
        
        r = ranges[valid]
        a = angles[valid]
        
        z = r * np.sin(a)
        y = r * np.cos(a)
        
        return z.astype(np.float32), y.astype(np.float32)
    
    def _compute_features(self):
        """Compute statistical features from all slices"""
        features = []
        
        for i in range(self.num_slices):
            slice_z_pos = self.z0 + i * self.slice_step_m
            z, y = self.slice_points_zy[i]
            n = float(len(z))
            
            if n == 0:
                features += [slice_z_pos, 0.0] + [0.0] * 8
            else:
                features += [
                    slice_z_pos,
                    n,
                    float(z.min()), float(z.max()), float(z.mean()), float(z.std()),
                    float(y.min()), float(y.max()), float(y.mean()), float(y.std()),
                ]
        
        return features
    
    def _publish_pointcloud(self):
        """Publish accumulated 3D point cloud to RViz"""
        if not self.all_cloud_points:
            return
        
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.height = 1
        msg.width = len(self.all_cloud_points)
        
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack points
        buffer = [struct.pack("fff", p[0], p[1], p[2]) for p in self.all_cloud_points]
        msg.data = b"".join(buffer)
        
        self.pub_cloud.publish(msg)
    
    def _publish_empty_cloud(self):
        """Publish empty point cloud to clear RViz"""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.height = 1
        msg.width = 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 0
        msg.is_dense = True
        msg.data = b""
        self.pub_cloud.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        if self.conveyor is not None:
            self.conveyor.close()
            print("Conveyor connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
