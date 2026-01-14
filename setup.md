# Setup Guide

Step-by-step instructions for setting up the 6DOF pose estimation system.

---

## What You Need

### Hardware
- NVIDIA Jetson Orin Nano (8GB)
- YDLidar G2
- Lynxmotion LSS Smart Servo
- Conveyor belt
- 12V power supply
- USB cables (USB-A for LiDAR, USB-C for motor)

### Mounting
1. Mount LiDAR **vertically** (90° to ground)
2. Position 20-30cm above conveyor belt
3. Ensure scan plane is perpendicular to conveyor direction
4. Connect LSS servo to belt drive mechanism

### Connections
```
Jetson Orin Nano
├── USB Port 1  → YDLidar G2 (/dev/ttyUSB0)
├── USB-C Port  → LSS Motor (/dev/ttyUSB1)
└── 12V Supply  → Motor Power
```

---

## Software Installation

### Prerequisites

Ubuntu 22.04 with Jetpack 6.0 on Jetson:
```bash
lsb_release -a
```

### Install ROS2 Humble
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

### Install Python Dependencies
```bash
sudo apt install -y python3-pip python3-serial python3-numpy build-essential

pip3 install pyserial numpy onnxruntime matplotlib pandas scikit-learn
```

### Install YDLidar Driver
```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Driver Configuration

### USB Permissions
```bash
sudo usermod -a -G dialout $USER
```

**Log out and log back in** for this to take effect.

### CH341 Driver (for Motor)
```bash
# Install tools
sudo apt install build-essential linux-headers-$(uname -r)

# Build driver
cd ~
git clone https://github.com/juliagoda/CH341SER.git
cd CH341SER
make
sudo make install
sudo modprobe ch341

# Auto-load on boot
echo "ch34x" | sudo tee -a /etc/modules
```

### Disable brltty

This service interferes with USB serial devices:
```bash
sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
```

### Verify Devices

Plug in both devices and check:
```bash
ls /dev/ttyUSB*
```

Should see:
```
/dev/ttyUSB0  # LiDAR
/dev/ttyUSB1  # Motor
```

If not, check with:
```bash
sudo dmesg | tail -20
lsusb
```

---

## Calibration

### Conveyor Belt

**Default: 22° motor rotation = 1cm belt movement**

To recalibrate:
```bash
cd ~/lidar-pose-estimation
python3 scripts/calibrate_conveyor.py
```

Follow prompts:
1. Mark starting position on belt
2. Press ENTER 10 times
3. Measure total distance traveled
4. Calculate: `degrees_per_cm = (10 × 22) / distance_cm`

Update in `pose_estimator_node.py`:
```python
self.declare_parameter("conveyor_degrees_per_cm", 22.0)  # Your value
```

### LiDAR Angle Filter

Find the correct angle range to capture only the object:
```bash
cd ~/lidar-pose-estimation/ros2_ws
source install/setup.bash
ros2 launch lidar_pose_estimation full_pipeline.launch.py
```

In another terminal, adjust angles:
```bash
ros2 param set /angle_filter_tester angle_min_deg 170.0
ros2 param set /angle_filter_tester angle_max_deg 190.0
```

Watch RViz2 - adjust until only the cube is visible (no background/table).

Update `filter_node.py` with final values:
```python
self.angle_min_deg = 170.0  # Your calibrated min
self.angle_max_deg = 190.0  # Your calibrated max
```

---

## Testing

### Test LiDAR
```bash
./scripts/test_lidar.sh
```

Should see scan data streaming.

### Test Motor
```bash
./scripts/test_conveyor.sh
```

Motor should respond with servo ID.

### Test Full System
```bash
cd ~/lidar-pose-estimation/ros2_ws
source install/setup.bash
ros2 launch lidar_pose_estimation full_pipeline.launch.py
```

Should see:
- LiDAR connected
- Conveyor connected: servo ID 5, 22.0°/cm
- RViz2 opens with filtered scan

### Test Pose Estimation
1. Place 15cm cube on conveyor
2. Press ENTER 15 times (captures slices)
3. Type `s` to predict pose
4. Check output shows position (x,y,z) and angles (roll,pitch,yaw)

---

## Troubleshooting

**USB devices not appearing:**
- Check driver installation
- Verify brltty is disabled
- Try different USB ports

**LiDAR not connecting:**
- Check baud rate: 230400
- Verify port permissions
- Check USB cable

**Motor not responding:**
- Ensure 12V power supply is ON
- Load CH341 driver: `sudo modprobe ch341`
- Check USB-C connection

**Angles wrong in RViz:**
- Recalibrate angle filter
- Try angle_filter_tester node

---

## Next Steps

After setup:
- See [API.md](docs/API.md) for code documentation
- Check model training scripts in `model/` folder
- Read inline comments in Python files

---

✅ **Setup Complete!**

Your system is ready for pose estimation.
