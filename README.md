# 6DOF Pose Estimation with LiDAR and Deep Learning

Real-time 6DOF pose estimation using a vertical LiDAR scanner, conveyor belt, and deep learning on NVIDIA Jetson Orin Nano.

## What it does

This system figures out the exact position and rotation of a 15cm cube:
1. LiDAR mounted vertically scans the object
2. Conveyor moves the cube through the scan area
3. Captures 15 slices (1cm apart) to build a 3D point cloud
4. Extracts features from each slice
5. Neural network predicts the 6DOF pose

**Accuracy:**
- Position: < 3mm error
- Rotation: < 1.5° error

---

## Repository Structure
```
├── README.md
├── SETUP.md
├── USAGE.md
├── ros2_ws/
│   └── src/
│       └── lidar_pose_estimation/
│           ├── launch/
│           ├── lidar_pose_estimation/
│           └── config/
├── model/
│   ├── exports/          # ONNX model + scalers
│   ├── outputs/          # Training results
│   └── training_data/    # Dataset
├── scripts/
│   ├── install_dependencies.sh
│   ├── test_lidar.sh
│   └── test_conveyor.sh
└── docs/
```

---

## Hardware Requirements

- **Computer:** NVIDIA Jetson Orin Nano (8GB)
- **LiDAR:** YDLidar G2
- **Motor:** Lynxmotion LSS Smart Servo
- **Conveyor:** Custom belt system
- **Power:** 12V supply for motor

## Software Requirements

- Ubuntu 22.04 (Jetpack 6.0)
- ROS2 Humble
- Python 3.10
- ONNX Runtime

---

## Quick Start
```bash
# Clone
git clone https://github.com/YOUR_USERNAME/lidar-pose-estimation.git
cd lidar-pose-estimation

# Install dependencies
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh

# Build ROS2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch lidar_pose_estimation full_pipeline.launch.py
```

See [SETUP.md](SETUP.md) for detailed installation and [USAGE.md](USAGE.md) for operation guide.

---

## How It Works
```
LiDAR → Angle Filter → Capture 15 slices (1cm apart) 
    → Extract features (min/max/mean/std per slice)
    → ONNX Neural Network → 6DOF Pose (x,y,z,roll,pitch,yaw)
```

**Details:**
- Conveyor moves 1cm between slices (22° motor rotation = 1cm)
- Each slice: 10 statistical features (x/y min, max, mean, std)
- Total: 150 features → Neural network → 6 DOF output

---

## Project Info

**Made at:** UTC - Master ISCF  
**Team:** 5 students  
**Year:** 2025-2026

---

## Contributing

Feel free to fork and improve! Open issues or PRs if you find bugs or have ideas.

---

---

## Contact

**GitHub:** [@andri_10]

Open an issue if you have questions!

---

## Useful Links

- [YDLidar G2 Docs](https://www.ydlidar.com/products/view/5.html)
- [LSS Servo Guide](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2/lynxmotion-smart-servo/)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
