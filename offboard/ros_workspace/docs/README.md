# Offboard (companion) setup — README

This document describes how to prepare an offboard (companion) computer running Ubuntu 24.04 and ROS 2 Jazzy for offboard computation and GUI (RViz). Important: complete the onboard setup first (see "Onboard setup" below).

## Links
- Ubuntu 24.04: https://releases.ubuntu.com/24.04/
- ROS 2 Jazzy installation (Deb packages): https://docs.ros.org/en/jazzy/Installation.html

## High-level flow
1. Onboard setup (required) — make sure the vehicle and companion computer (onboard) have required software, network, and serial configuration.
2. Offboard (this machine) — install Ubuntu 24.04, ROS 2 Jazzy Desktop (includes RViz), required packages and tools, then configure environment.

## 1) Onboard setup (DO THIS FIRST)
Before configuring the offboard workstation, ensure the onboard/computer that interfaces directly with the vehicle is configured. Minimum tasks:
- Install supported OS (Ubuntu 24.04) on the onboard machine (or ensure compatibility).
- Install ROS 2 Jazzy on the onboard machine if it will run ROS nodes (follow same ROS install link above).
- Install ROS on the onboard device if communicating with autopilot:
- Configure serial/USB permissions and udev rules so autopilot serial devices are accessible to ROS processes.
- Configure networking (Wi‑Fi / Ethernet / ad-hoc / ROS2 DDS discovery settings) so offboard and onboard machines can communicate.
- Verify basic comms: ping between machines, ros2 topic list (if ROS is running),.

Refer README.md in odboard directory for above setup.

Only after onboard is reachable and verified proceed with the offboard steps below.


## 2) Offboard (companion workstation) — Ubuntu 24.04 + ROS 2 Jazzy

Follow the same ROS 2 Jazzy installation process that you used for the onboard machine (see link above). IMPORTANT: on the offboard install the Desktop variant (ros-jazzy-desktop) — do NOT install the base (ros-jazzy-base). The offboard requires GUI tools such as RViz.

Example condensed tasks you still need to perform on the offboard:

1. Install development tools and ROS Desktop (after adding the same repos/keys as onboard)
```bash
sudo apt install -y build-essential git python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
# Desktop metapackage (includes RViz)
sudo apt install -y ros-jazzy-desktop
```

2. Install the required package
```bash
sudo apt-get install -y libboost-all-dev
sudo apt install -y ros-jazzy-tf-transformations
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

3. Environment and workspace build
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd autonomous_warehouse_surveillance/offboard/ros_workspace
colcon build --symlink-install
source install/setup.bash
```
4. Launch
```bash
ros2 launch warehouse_robot_bringup offboard.launch.py

```
- In another terminal configure and activate the slam_toolbox
```bash
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate 
```


## 4) Troubleshooting checklist
- Can you ping the onboard IP from offboard?
- Does `ros2 node list` / `ros2 topic list` show expected topics when onboard is running nodes?
- Is RViz able to connect and display robot TF and sensor topics?
- Check logs in `~/.ros` and system journal (`journalctl`) for service errors.

## 5) Useful links
- ROS 2 Jazzy install (Deb packages): https://docs.ros.org/en/jazzy/Installation.html
- ROS 2 tutorials and tools: https://docs.ros.org/en/jazzy/Tutorials.html