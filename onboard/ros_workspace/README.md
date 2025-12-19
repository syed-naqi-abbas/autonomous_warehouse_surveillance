# Warehouse Robot Project

A ROS2-based autonomous warehouse robot system with QR code scanning, object detection, and mecanum drive capabilities.

---

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Package Descriptions](#package-descriptions)
- [Project Structure](#project-structure)
- [Database Schema](#database-schema)
- [Troubleshooting](#troubleshooting)

---

## Overview

This project implements an autonomous warehouse robot capable of:
- **QR Code Scanning**: Identifies and logs warehouse items using QR codes
- **Object Detection**: Uses YOLO for real-time object detection
- **Autonomous Navigation**: Mecanum drive with sensor fusion (LiDAR, IMU)
- **Database Management**: SQLite-based inventory tracking
- **Real-time Monitoring**: Live database viewer for scanned items

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Warehouse Robot System                    │
├─────────────────────────────────────────────────────────────┤
│  Hardware Layer                                              │
│  ├── YDLidar (2D Scanning)                                  │
│  ├── BNO055 IMU (Orientation)                               │
│  ├── USB Camera (Vision)                                    │
│  ├── ESP32 (Motor Control via Serial)                       │
│  └── Stepper Motor (Lifting Mechanism)                      │
├─────────────────────────────────────────────────────────────┤
│  ROS2 Control Layer                                          │
│  ├── hardware (Hardware Interface)                          │
│  ├── sensors (IMU Node)                                     │
│  ├── stepper_motor (Lift Control)                           │
│  └── warehouse_scanning (Vision Processing)                 │
├─────────────────────────────────────────────────────────────┤
│  Application Layer                                           │
│  ├── QR Detection + YOLO Object Detection                   │
│  ├── SQLite Database Management                             │
│  └── Live Database Viewer                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Features

### Vision System
- **Dual Detection**: Simultaneous YOLO object detection and QR code scanning
- **Real-time Processing**: 30+ FPS performance on standard hardware
- **Automatic Logging**: QR codes automatically parsed and stored in database

### Navigation
- **Mecanum Drive**: Omnidirectional movement with ESP32 control
- **Sensor Fusion**: LiDAR + IMU for robust localization
- **ROS2 Control**: Standardized control interfaces

### Data Management
- **SQLite Database**: Lightweight, file-based inventory system
- **Live Viewer**: Real-time table display with auto-refresh
- **Structured Data**: Parsed QR codes (Rack_ID, Shelf_ID, Item_Code)

---

## Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Main Computer** | Ubuntu 24.04, 8GB RAM | For onboard processing |
| **Camera** | USB Camera | Mounted at `/dev/video0` |
| **LiDAR** | YDLidar (G2, X2, etc.) | 360° 2D scanning |
| **IMU** | BNO055 | 9-DOF orientation |
| **Motor Controller** | ESP32 | Serial communication |
| **Stepper Motor** | NEMA 17/23 | For lifting mechanism |
| **Base** | Mecanum Wheels | 4-wheel omnidirectional |

---

## Software Requirements

- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy
- **Python**: 3.12.3 or newer
- **OpenCV**: 4.10.0+
- **YOLO**: Ultralytics YOLOv8
- **PyTorch**: 2.0.0+

---

## Installation

### 1. ROS2 Jazzy Installation

#### Set Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### Enable Required Repositories
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

#### Install ROS2 Base (No GUI)
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-dev-tools
sudo apt install ros-jazzy-ros-base
```

#### Setup Environment
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. System Dependencies Installation

#### Development Tools and Libraries
```bash
# Boost libraries for serial communication
sudo apt-get install libboost-all-dev

# Build tools
sudo apt install cmake pkg-config swig

# ROS2 packages
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-twist-stamper
sudo apt install ros-jazzy-v4l2-camera
sudo apt install ros-jazzy-teleop-twist-keyboard
```

### 3. YDLidar SDK Installation

```bash
cd ~/Downloads
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

### 4. Clone Repository and Setup Workspace

```bash
# Clone the repository
git clone https://github.com/shlok165/autonomous_warehouse_surveillance.git
cd autonomous_warehouse_surveillance

# Remove offboard components (not needed for onboard setup)
rm -rf offboard

# Navigate to onboard ROS workspace
cd onboard/ros_workspace
```

### 5. Python Virtual Environment Setup

#### Create Virtual Environment
```bash
python3 -m venv ros_env
source ros_env/bin/activate

# Prevent colcon from building the virtual environment
touch ros_env/COLCON_IGNORE
```

#### Install Python Dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

#### Add Virtual Environment Activation to Workspace Setup
```bash
echo "source $(pwd)/ros_env/bin/activate" >> ~/.bashrc
source ~/.bashrc
```
## ⚠️ CRITICAL: PORT CONFIGURATION REQUIRED

**BEFORE RUNNING THE SYSTEM**, you must configure the correct serial ports. The default configurations won't work with your specific hardware.

### Files to Update:
1. `src/ydlidar_ros2_driver/params/ydlidar.yaml`
2. `src/stepper_motor/src/stepper_motor_node.cpp`
3. `src/warehouse_robot_description/urdf/mobile_base.ros2_control.xacro`

### Quick Steps:
1. **Unplug all USB devices**
2. **Plug in ONE device at a time**
3. **Check port:** `ls /dev/ttyUSB* /dev/ttyACM*`
4. **Note which port appears**
5. **Repeat for each device**
6. **Update the 3 files above with your specific ports**

**Default examples may all show `/dev/ttyUSB0` - change each to match your actual device ports.**

---

## Usage

### 6. Camera and Serial Device Permissions

#### Set Camera Permissions   
```bash
sudo usermod -a -G video $USER
sudo chmod 777 /dev/video*
```

#### Set Serial Device Permissions
```bash
# For your specific device (adjust as needed)
sudo chmod 777 /dev/ttyUSB0

# Make permanent by creating udev rule
sudo nano /etc/udev/rules.d/99-usb-ports.rules 
```

Add these lines to the file:
```
SUBSYSTEM=="tty", KERNELS=="4-1", SYMLINK+="esp32", MODE="666"
SUBSYSTEM=="tty", KERNELS=="2-1", SYMLINK+="lidar", MODE="666"
```

Then reload rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Replug your serial devices once for the rules to take effect
# After this, permissions will be automatic on every future plug-in
# Your devices will now be accessible as /dev/esp32 and /dev/lidar
```
### 7. Build ROS2 Workspace

```bash
cd ~/autonomous_warehouse_surveillance/onboard/ros_workspace
colcon build
source install/setup.bash

# Add to bashrc for automatic sourcing
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

### 8. Database Setup

The SQLite database will be created automatically on first run. To verify:
```bash
cd ~/autonomous_warehouse_surveillance/onboard/ros_workspace
sqlite3 qr_data.db
.tables
.quit
```

### 9. Environment Variables (Optional)

Add to `~/.bashrc` for convenience:
```bash
# ROS2 Workspace
export ROS_WORKSPACE=~/autonomous_warehouse_surveillance/onboard/ros_workspace

# Device selection for YOLO
export DEVICE=cpu  # or 'cuda' if GPU available

# ROS Domain ID (if using multiple robots)
export ROS_DOMAIN_ID=0
```

---

## Usage

### Start the Robot System

#### 1. Launch All Sensors and Hardware
```bash
ros2 launch warehouse_robot_bringup onboard.launch.py
```

#### 2. Start QR Scanning Node
```bash
# In a new terminal
source ~/.bashrc
ros2 launch warehouse_scanning qr_detection.launch.py
```


### Quick Start Commands

```bash
# Activate environment
source ~/.bashrc

# Launch all sensors
ros2 launch warehouse_robot_bringup onboard.launch.py

# Run QR scanning (in new terminal)
ros2 launch warehouse_scanning qr_detection.launch.py

# View database (in new terminal)
python3 src/warehouse_scanning/warehouse_scanning/database_viewer.py
```

---

## Package Descriptions


### `hardware`
**Hardware interface using ros2_control**
- ESP32 communication via serial
- Mecanum drive kinematics
- Wheel velocity control

### `sensors`
**Sensor integration**
- `bno055_node.py`: IMU orientation publisher

### `stepper_motor`
**Lifting mechanism control**
- Position and velocity control
- Teleoperation support

### `warehouse_robot_bringup`
**Launch files**
- `onboard.launch.py`: Starts all onboard systems

### `warehouse_robot_description`
**URDF models**
- Robot description with ros2_control hardware interfaces

---

## Project Structure

```
ros_workspace/
├── requirements.txt                    # Python dependencies
├── ros_env/                            # Python virtual environment
├── qr_data.db                          # SQLite database (auto-created)
└── src/
    ├── hardware/                       # Hardware interface
    │   ├── include/
    │   │   └── hardware/
    │   │       ├── esp_comms.h         # ESP32 serial communication
    │   │       ├── mecanumdrive_esp.h  # Mecanum drive controller
    │   │       └── wheel.h             # Wheel model
    │   └── src/
    ├── sensors/                        # Sensor nodes
    │   ├── launch/
    │   │   └── sensors.launch.py
    │   └── sensors/
    │       └── bno055_node.py          # IMU node
    ├── stepper_motor/                  # Lifting mechanism
    │   ├── config/
    │   ├── launch/
    │   └── src/
    ├── warehouse_robot_bringup/        # Launch files
    │   ├── config/
    │   │   └── my_robot_controllers.yaml
    │   └── launch/
    │       └── onboard.launch.py
    ├── warehouse_robot_description/    # URDF models
    │   └── urdf/
    ├── warehouse_scanning/             # Vision processing
    │   ├── warehouse_scanning/
    │   │   └── qr_detection.py         # Main scanning node
    │   │   
    │   └── launch/
    │       └── qr_scanning.launch.py
    └── ydlidar_ros2_driver/            # LiDAR driver
        ├── params/
        └── launch/
```

---

## Database Schema

### Table: `qr_scans`

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER PRIMARY KEY | Auto-incrementing ID |
| `timestamp` | TEXT | ISO format timestamp |
| `qr_string` | TEXT UNIQUE | Raw QR code data |
| `rack_id` | TEXT | Parsed rack identifier |
| `shelf_id` | TEXT | Parsed shelf identifier |
| `item_code` | TEXT | Parsed item code |

### QR Code Format
Expected format: `RACK_SHELF_ITEM`

Example: `R01_S03_ITEM12345`
- Rack ID: R01
- Shelf ID: S03
- Item Code: ITEM12345

---

## Troubleshooting

### Camera Not Working
```bash
# Check available cameras
v4l2-ctl --list-devices

# Test camera
ffplay /dev/video2

# Update camera index in qr_detection.py if needed
self.cap = cv2.VideoCapture("/dev/video2")
```

### Serial Permission Denied
```bash
# Temporary fix
sudo chmod 777 /dev/ttyUSB0

# Permanent fix - add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### Database Locked
```bash
# Close all database connections
pkill -f database_viewer.py

# If persistent, remove and recreate
rm qr_data.db
# Restart qr_detection.py to recreate database
```

### Python Module Not Found
```bash
# Ensure virtual environment is activated
source ros_env/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

### ROS2 Build Errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Low FPS Performance
- Reduce YOLO confidence threshold
- Use smaller YOLO model (YOLOv8n instead of YOLOv8m/l)
- Reduce camera resolution
- Consider GPU acceleration (PyTorch with CUDA support)

### Camera Permission Issues
```bash
# List available cameras
v4l2-ctl --list-devices

# Set permissions
sudo usermod -a -G video $USER
sudo chmod 666 /dev/video*

# Reboot for group changes to take effect
sudo reboot
```

### Serial Device Issues
```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check user groups
groups $USER

# Should include: dialout, video
```

### Testing Individual Components

#### Test Camera
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video2"
```

#### Test Lidar
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

#### Test QR Scanning Node
```bash
source ros_env/bin/activate
ros2 run warehouse_scanning qr_detection
```

#### Test Database Viewer
```bash
source ros_env/bin/activate
python3 src/warehouse_scanning/warehouse_scanning/database_viewer.py
```


