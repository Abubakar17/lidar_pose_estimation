# API Documentation

Code documentation for the LiDAR pose estimation system.

---

## Table of Contents

1. [ROS2 Nodes](#ros2-nodes)
2. [Python Modules](#python-modules)
3. [Launch Files](#launch-files)
4. [Parameters](#parameters)
5. [Topics](#topics)

---

## ROS2 Nodes

### `pose_estimator` (Main Node)

**File:** `lidar_pose_estimation/pose_estimator_node.py`

Main node that captures LiDAR slices, builds 3D point cloud, and predicts 6DOF pose.

**Subscribed Topics:**
- `/scan_filtered` (sensor_msgs/LaserScan) - Filtered LiDAR scan data

**Published Topics:**
- `/cloud_3d` (sensor_msgs/PointCloud2) - Accumulated 3D point cloud for visualization

**Parameters:**
```python
input_topic: "/scan_filtered"           # LiDAR input topic
output_cloud_topic: "/cloud_3d"         # Point cloud output topic
num_slices: 15                          # Number of slices to capture
slice_step_m: 0.01                      # Distance between slices (meters)
z0: -0.10                               # Starting Z position (meters)
min_range: 0.05                         # Minimum valid LiDAR range (meters)
max_range: 10.0                         # Maximum valid LiDAR range (meters)
output_dir: "~/iscf/data/captures"      # Output directory for CSV files
model_dir: "~/iscf/model/exports"       # ONNX model directory
conveyor_port: "/dev/ttyUSB1"           # Motor serial port
conveyor_servo_id: 5                    # LSS servo ID
conveyor_degrees_per_cm: 22.0           # Calibration: degrees per cm
conveyor_step_cm: 1.0                   # Conveyor step per slice (cm)
```

**Interactive Commands:**
- `ENTER` - Capture slice and move conveyor
- `s` - Save session and predict pose
- `r` - Reset session

**Output Files:**
- `captures_X.csv` - Feature vectors (150 features per sample)
- `captures_Y.csv` - Predicted poses (6 DOF per sample)

**Usage:**
```bash
ros2 run lidar_pose_estimation pose_estimator
```

---

### `filter_node` (Angle Filter)

**File:** `lidar_pose_estimation/filter_node.py`

Filters LiDAR scan by angle range to isolate the object.

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan) - Raw LiDAR scan

**Published Topics:**
- `/scan_filtered` (sensor_msgs/LaserScan) - Angle-filtered scan

**Parameters:**
```python
angle_min_deg: -10.0     # Minimum angle (degrees)
angle_max_deg: 10.0      # Maximum angle (degrees)
```

**Logic:**
- Keeps points within `[angle_min, angle_max]` range
- Sets all other ranges to `inf` (filtered out)
- Normalizes angles to -180° to +180° range

**Usage:**
```bash
ros2 run lidar_pose_estimation filter_node
```

---

### `angle_filter_tester` (Calibration Tool)

**File:** `lidar_pose_estimation/angle_filter_tester.py`

Interactive tool for calibrating the angle filter in real-time.

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan) - Raw LiDAR scan

**Published Topics:**
- `/scan_filtered` (sensor_msgs/LaserScan) - Test-filtered scan

**Parameters (adjustable at runtime):**
```python
angle_min_deg: 160.0     # Default minimum angle
angle_max_deg: 200.0     # Default maximum angle
```

**Runtime Adjustment:**
```bash
# Change parameters while running
ros2 param set /angle_filter_tester angle_min_deg 170.0
ros2 param set /angle_filter_tester angle_max_deg 190.0

# Check current values
ros2 param get /angle_filter_tester angle_min_deg
```

**Usage:**
```bash
ros2 run lidar_pose_estimation angle_filter_tester
```

---

## Python Modules

### `conveyor_control.py`

**Class:** `ConveyorControl`

Controls Lynxmotion LSS Smart Servo for conveyor movement.

**Constructor:**
```python
ConveyorControl(port='/dev/ttyUSB1', servo_id=5, degrees_per_cm=22.0)
```

**Parameters:**
- `port` (str): Serial port path
- `servo_id` (int): LSS servo ID (default: 5)
- `degrees_per_cm` (float): Calibration value

**Methods:**

#### `send_command(command)`
Send command to servo without response.

**Args:**
- `command` (str): LSS command (e.g., "D900" for position)

**Returns:**
- `bool`: True if successful

**Example:**
```python
conveyor.send_command("D900")  # Move to 90 degrees
```

#### `query(command)`
Send query and read response.

**Args:**
- `command` (str): LSS query command (e.g., "QD" for position)

**Returns:**
- `str` or `None`: Response from servo

**Example:**
```python
response = conveyor.query("QID")  # Query servo ID
```

#### `move_cm(cm)`
Move conveyor by specified distance.

**Args:**
- `cm` (float): Distance to move in centimeters

**Returns:**
- `bool`: True if successful

**Example:**
```python
conveyor.move_cm(1.0)  # Move 1 cm
```

#### `get_position()`
Get current servo position.

**Returns:**
- `float` or `None`: Position in degrees

**Example:**
```python
pos = conveyor.get_position()
print(f"Current position: {pos}°")
```

#### `close()`
Close serial connection.

**Example:**
```python
conveyor.close()
```

---

### `geometry_utils.py`

Geometry utilities for 3D transformations.

#### `rpy_to_rot(roll, pitch, yaw)`
Convert Euler angles to rotation matrix.

**Args:**
- `roll` (float): Rotation around X-axis (radians)
- `pitch` (float): Rotation around Y-axis (radians)
- `yaw` (float): Rotation around Z-axis (radians)

**Returns:**
- `np.ndarray`: 3x3 rotation matrix

**Example:**
```python
R = rpy_to_rot(0.1, 0.2, 0.3)
```

#### `make_unit_cube(size_m=0.15)`
Generate cube vertices centered at origin.

**Args:**
- `size_m` (float): Cube side length in meters

**Returns:**
- `np.ndarray`: 8x3 array of vertices

**Example:**
```python
vertices = make_unit_cube(0.15)  # 15cm cube
```

#### `cube_faces(vertices)`
Generate face definitions from vertices.

**Args:**
- `vertices` (np.ndarray): 8x3 vertex array

**Returns:**
- `list`: 6 faces, each with 4 vertices

**Example:**
```python
faces = cube_faces(vertices)
```

#### `make_train_x_header(num_slices=15)`
Generate CSV header for training features.

**Args:**
- `num_slices` (int): Number of slices

**Returns:**
- `list`: Column names (150 features for 15 slices)

**Example:**
```python
header = make_train_x_header(15)
# ['s0_z', 's0_n_points', 's0_x_min', 's0_x_max', ...]
```

---

### `visualization.py`

3D visualization for pose estimation results.

#### `visualize_pose_cube(pose, cube_size_m=0.15)`
Display predicted pose as a 3D cube using matplotlib.

**Args:**
- `pose` (np.ndarray): 6-element array [x, y, z, roll, pitch, yaw]
- `cube_size_m` (float): Cube side length

**Example:**
```python
pose = np.array([0.1, 0.2, 0.05, 0.0, 0.0, 0.785])
visualize_pose_cube(pose)  # Opens matplotlib window
```

**Note:** Blocks until window is closed (unless `block=False`).

---

### `inference_onnx.py`

**Class:** `PoseEstimator`

ONNX-based pose estimation inference.

**Constructor:**
```python
PoseEstimator(model_dir, use_gpu=False)
```

**Parameters:**
- `model_dir` (str): Directory containing ONNX model and scalers
- `use_gpu` (bool): Use GPU acceleration (default: False)

**Required Files in `model_dir`:**
- `pose_model.onnx` - Neural network model
- `scaler_X.joblib` - Input feature scaler
- `scaler_Y.joblib` - Output pose scaler

**Methods:**

#### `predict(features)`
Predict 6DOF pose from input features.

**Args:**
- `features` (np.ndarray): 150-element feature vector

**Returns:**
- `np.ndarray`: 6-element pose [x, y, z, roll, pitch, yaw]
  - Position in meters
  - Angles in radians

**Example:**
```python
estimator = PoseEstimator('model/exports/')
features = np.random.randn(150).astype(np.float32)
pose = estimator.predict(features)
print(f"Position: {pose[:3]}")
print(f"Angles: {np.degrees(pose[3:])}")
```

---

## Launch Files

### `full_pipeline.launch.py`

**File:** `launch/full_pipeline.launch.py`

Launches complete system: LiDAR driver, angle filter, and pose estimator.

**Nodes Launched:**
1. YDLidar driver (from `ydlidar_ros2_driver` package)
2. Angle filter node
3. Pose estimator node (in separate terminal)

**Usage:**
```bash
ros2 launch lidar_pose_estimation full_pipeline.launch.py
```

**Customization:**
Edit the launch file to change node parameters:
```python
pose_estimator = Node(
    package='lidar_pose_estimation',
    executable='pose_estimator',
    name='pose_estimator_node',
    parameters=[{
        'num_slices': 20,           # Change number of slices
        'conveyor_step_cm': 0.5,    # Change step size
    }]
)
```

---

## Parameters

### Global System Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_slices` | int | 15 | Number of vertical slices to capture |
| `slice_step_m` | float | 0.01 | Distance between slices (meters) |
| `z0` | float | -0.10 | Starting Z position (meters) |
| `min_range` | float | 0.05 | Minimum valid LiDAR range (meters) |
| `max_range` | float | 10.0 | Maximum valid LiDAR range (meters) |

### Conveyor Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `conveyor_port` | str | "/dev/ttyUSB1" | Serial port for motor |
| `conveyor_servo_id` | int | 5 | LSS servo ID |
| `conveyor_degrees_per_cm` | float | 22.0 | Motor rotation per cm |
| `conveyor_step_cm` | float | 1.0 | Conveyor movement per slice |

### Angle Filter Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `angle_min_deg` | float | -10.0 | Minimum angle to keep (degrees) |
| `angle_max_deg` | float | 10.0 | Maximum angle to keep (degrees) |

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | Raw LiDAR scan (360°) |
| `/scan_filtered` | sensor_msgs/LaserScan | Angle-filtered scan |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan_filtered` | sensor_msgs/LaserScan | Filtered scan for pose estimation |
| `/cloud_3d` | sensor_msgs/PointCloud2 | Accumulated 3D point cloud |

### Topic Details

#### LaserScan Message Structure
```
header:
  stamp: Current time
  frame_id: "laser_frame"
angle_min: -3.14159 (rad)
angle_max: 3.14159 (rad)
angle_increment: 0.00872 (rad)
range_min: 0.12 (m)
range_max: 10.0 (m)
ranges: [float array of distances]
```

#### PointCloud2 Message Structure
```
header:
  stamp: Current time
  frame_id: "laser_frame"
height: 1
width: N (number of points)
fields: [x, y, z] (FLOAT32)
data: [binary point data]
```

---

## Feature Vector Format

### Input Features (150 dimensions)

For each of 15 slices:
- `z` - Slice Z position (1 value)
- `n_points` - Number of valid points (1 value)
- `x_min`, `x_max`, `x_mean`, `x_std` - X statistics (4 values)
- `y_min`, `y_max`, `y_mean`, `y_std` - Y statistics (4 values)

**Total:** 15 slices × 10 features = 150 dimensions

### Output Pose (6 dimensions)

- `x`, `y`, `z` - Position in meters
- `roll`, `pitch`, `yaw` - Orientation in radians

---

## Data Flow
```
LiDAR (/scan)
    ↓
Angle Filter
    ↓
/scan_filtered
    ↓
Pose Estimator Node
    ├→ Capture 15 slices
    ├→ Extract features (150D)
    ├→ ONNX inference
    ├→ Predict pose (6D)
    └→ Publish /cloud_3d
```

---

## Example Usage

### Complete Workflow
```python
import rclpy
from rclpy.node import Node
from lidar_pose_estimation.conveyor_control import ConveyorControl
from lidar_pose_estimation.inference_onnx import PoseEstimator
import numpy as np

# Initialize ROS2
rclpy.init()

# Initialize conveyor
conveyor = ConveyorControl('/dev/ttyUSB1', servo_id=5, degrees_per_cm=22.0)

# Initialize pose estimator
estimator = PoseEstimator('model/exports/', use_gpu=False)

# Move conveyor
conveyor.move_cm(1.0)

# Predict pose (assuming features already extracted)
features = np.random.randn(150).astype(np.float32)
pose = estimator.predict(features)

print(f"Predicted pose:")
print(f"  Position: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}")
print(f"  Angles: roll={np.degrees(pose[3]):.1f}°, "
      f"pitch={np.degrees(pose[4]):.1f}°, yaw={np.degrees(pose[5]):.1f}°")

# Cleanup
conveyor.close()
rclpy.shutdown()
```

---

## LSS Servo Commands

### Common Commands

| Command | Description | Example |
|---------|-------------|---------|
| `D<pos>` | Move to position (0.1° units) | `D900` = 90° |
| `SD<speed>` | Set speed (0.1°/s units) | `SD300` = 30°/s |
| `QD` | Query position | Returns `*5QD900` |
| `QID` | Query ID | Returns `*5QID5` |
| `L` | Limp mode (disable torque) | |
| `H` | Hold position | |

### Protocol Format
```
Command: #<ID><COMMAND>\r
Response: *<ID><COMMAND><VALUE>\r
```

---

## Coordinate Frames

### Coordinate System
```
Z (up)
|
|
|___Y (away from LiDAR)
/
/
X (right)
```

**Origin:** LiDAR sensor location

**Cube Pose:**
- Position relative to LiDAR
- Rotation follows right-hand rule (ZYX Euler angles)

---

## Error Codes

### Conveyor Control

- Motor not responding → Check power supply and USB connection
- Position query fails → Driver not loaded (`sudo modprobe ch341`)
- Serial timeout → Check baud rate (115200)

### Pose Estimator

- Model file not found → Check `model_dir` parameter
- Feature dimension mismatch → Verify 150 features (15 slices × 10)
- Scaler file missing → Ensure `scaler_X.joblib` and `scaler_Y.joblib` exist

---

## Performance Notes

- **Inference time:** ~5-10ms on Jetson Orin Nano (CPU)
- **Slice capture:** ~100ms per slice (LiDAR + processing)
- **Total scan time:** ~15 seconds for 15 slices (including conveyor movement)
- **Memory usage:** ~200MB for full pipeline

---

## Development Notes

### Adding Custom Features

To add new features per slice, modify `compute_features()` in `pose_estimator_node.py`:
```python
def _compute_features(self):
    features = []
    for i in range(self.num_slices):
        z, y = self.slice_points_zy[i]
        
        # Add your custom features here
        custom_feature = np.percentile(z, 95)  # Example: 95th percentile
        
        features += [
            # ... existing features ...
            custom_feature,
        ]
    return features
```

**Important:** Update `make_train_x_header()` accordingly and retrain the model.

---

## References

- [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)
- [LSS Protocol](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2/lynxmotion-smart-servo/lss-communication-protocol/)
- [ONNX Runtime](https://onnxruntime.ai/docs/)
- [ROS2 Humble](https://docs.ros.org/en/humble/)

---

**Last Updated:** January 2025
