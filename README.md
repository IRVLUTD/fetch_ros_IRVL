# fetch_ros_IRVL

ROS 1 (Noetic) software stack for the **Fetch Mobile Manipulator** customized by the **Intelligent Robotics and Vision Lab (IRVL)** at the University of Texas at Dallas.

This repository provides robot descriptions, MoveIt motion planning configurations, teleoperation nodes, and autonomous manipulation pipelines for the Fetch robot equipped with:
- **16-DOF LEAP Dexterous Hand** (Ethernet / OpenRB-150 bridge)
- **ATI-Gamma 6-Axis Force/Torque Sensor** (NetFT UDP stream)
- **UMI Deformable Fingers** (parallel jaw alternate)

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Repository Ecosystem](#repository-ecosystem)
- [Environment & Prerequisites](#environment--prerequisites)
- [Hardware Network Configuration](#hardware-network-configuration)
- [Building the Workspace](#building-the-workspace)
- [Running in Simulation (Gazebo)](#running-in-simulation-gazebo)
- [Running on the Physical Robot](#running-on-the-physical-robot)
- [PS4 Controller Teleoperation Guide](#ps4-controller-teleoperation-guide)
- [Utility Scripts](#utility-scripts)
- [Troubleshooting & Maintenance](#troubleshooting--maintenance)
- [Previous IRVL Documentation (ATI FT & UMI Fingers)](#previous-irvl-documentation-ati-gamma-ft--umi-deformable-fingers)

## Repository Ecosystem

This workspace is modularized across three repositories:

| Repository | Purpose | Location in Workspace |
| :--- | :--- | :--- |
| **`fetch_ros_IRVL`** *(this repo)* | URDF descriptions, MoveIt configuration, joystick teleoperation, demo and homing scripts. | `catkin_ws/src/fetch_ros_IRVL` |
| **`leap_inhand`** | Hardware Ethernet driver node (`leap_ethernet`), action server bridge, and OpenRB-150 Arduino/Teensy firmware. | `catkin_ws/src/leap_ethernet` |
| **`fetch_gazebo`** | Gazebo simulation worlds, Fetch + LEAP robot spawn files, and simulation controller configurations. | `catkin_ws/src/fetch_gazebo` |

---

## Environment & Prerequisites

All packages are developed and validated on **Ubuntu 20.04** running **ROS Noetic** via Docker.

### Docker Setup

```bash
# Start container with host networking and input device passthrough
docker run -it --net=host \
  --device=/dev/input \
  -v /home/$USER/Projects:/root/Projects \
  noetic-foxglove bash
```

To attach a second terminal to an already-running container:
```bash
docker exec -it <container_name_or_id> bash
```

---

## Hardware Network Configuration

The Fetch robot and peripheral devices communicate over the Fetch internal Ethernet subnet (`10.42.42.0/24`):

| Device | Interface | IP Address | Port / Protocol |
| :--- | :--- | :--- | :--- |
| **Fetch Internal Computer** | Internal Ethernet | `10.42.42.1` | - |
| **OpenRB-150 (LEAP Hand)** | Ethernet Bridge | `10.42.42.50` | `8888` (UDP) |
| **ATI NetFT Sensor** | Ethernet Wrist F/T | `10.42.42.41` | UDP / RDT |

### Network Verification
Before running physical robot drivers, verify communication with the LEAP hand:
```bash
# 1. Ping the OpenRB-150 Ethernet board
ping 10.42.42.50

# 2. Test UDP reachability (Ctrl+C to exit)
nc -u 10.42.42.50 8888
```

---

## Building the Workspace

Inside your ROS workspace (`catkin_ws`):

```bash
cd ~/Projects/catkin_ws

# Symlink or clone packages into src/
# Ensure fetch_ros_IRVL, fetch_gazebo, and leap_ethernet are present in src/

# Build all packages
catkin_make

# Source environment
source devel/setup.bash
```

> [!NOTE]
> `fetch_ikfast_plugin` and `fetch_depth_layer` include `CATKIN_IGNORE` flags to bypass deprecated dependencies in ROS Noetic.

---

## Running in Simulation (Gazebo)

### 1. Launch Gazebo Simulation
Starts Gazebo with the Fetch robot mounted with the 16-DOF LEAP hand and loads the standard manipulation environment:
```bash
roslaunch fetch_gazebo simulation.launch robot:=fetch_leap
```

### 2. Launch MoveIt Motion Planning
Starts the `move_group` node configured for `fetch_leap`:
```bash
roslaunch fetch_moveit_config move_group.launch robot:=fetch_leap
```

### 3. Run Script
#### Option 1: Autonomous Pick and Place Demo
Executes inverse kinematics, table scene collision generation, Cartesian approach, enveloping grasp, lift, place, and retreat:
```bash
python3 /root/Projects/fetch_ros_IRVL/scripts/leap_pick_place_demo.py
```

#### Option 2: Teleoperation
Drive the simulated base, torso, head, and arm with a PS4 controller:
```bash
roslaunch fetch_teleop teleop.launch robot:=fetch_leap
```

---

## Running on the Physical Robot

### 1. Power and Hardware Preparation
1. Ensure the Fetch robot is powered on and safety breakers are disengaged.
2. Verify the OpenRB-150 Ethernet bridge on the LEAP hand has an active link light (`10.42.42.50`).
3. Ensure the arm has clearance to move safely.

### 2. Launch LEAP Hand Hardware Driver & MoveIt
This starts the Ethernet UDP bridge to the physical hand, publishes `/joint_states`, and brings up the MoveIt planning pipeline:
```bash
roslaunch fetch_moveit_config fetch_leap_real.launch hand_ip:=10.42.42.50
```

### 3. Launch Joystick Teleoperation
```bash
roslaunch fetch_teleop teleop.launch joy_dev:=/dev/input/js0
```

### 4. Home the Robot (Straight Forward + Open Hand)
Run the diagnostic homing script to position the arm straight forward and open the hand:
```bash
python3 /root/Projects/fetch_ros_IRVL/scripts/send_zero_arm.py
```

---

## PS4 Controller Teleoperation Guide

The teleoperation node `fetch_teleop/src/joystick_teleop.cpp` provides full control over the mobile base, torso, head, arm Cartesian velocity, and LEAP hand grasps.

### Control Layout

| Function | Controls | Requirement |
| :--- | :--- | :--- |
| **Base Drive** | Left Stick (Linear / Angular) | Hold **L1 (Deadman)** |
| **Torso Up / Down** | D-Pad Up / D-Pad Down | Hold **L1 (Deadman)** |
| **Head Pan / Tilt** | Left Stick (Pan: Horiz / Tilt: Vert) | Hold **SHARE (Button 8)** |
| **Arm Translation (X / Y)** | Left Stick (X: Vert / Y: Horiz) | Hold **L1 (Deadman)** + Hold **OPTIONS** |
| **Arm Translation (Z)** | Right Stick (Vertical) | Hold **L1 (Deadman)** + Hold **OPTIONS** |
| **Arm Rotation (Roll / Pitch)** | Right Stick (Roll: Horiz / Pitch: Vert) | Hold **L1 (Deadman)** + Hold **R1** |
| **Arm Rotation (Yaw)** | Left Stick (Horizontal) | Hold **L1 (Deadman)** + Hold **R1** |
| **LEAP Hand: Enveloping Grasp** | Press **Square** | Hold **L1 (Deadman)** |
| **LEAP Hand: Open Hand** | Press **Cross (X)** | Hold **L1 (Deadman)** |

> [!TIP]
> - **Head Control Override:** Holding **SHARE** automatically inhibits base driving so the robot remains stationary while panning and tilting the head camera with the Left Stick.
> - **Smooth Hardware Interpolation:** The teleoperation node streams commands at 50 Hz along a 1.2-second smooth trajectory for both open and close actions. This eliminates motor current spikes and prevents Dynamixel overload faults on the physical hand.

---

## Utility Scripts

- **`scripts/leap_pick_place_demo.py`**:
  Full autonomous pick-and-place demonstration using MoveIt Cartesian trajectory generation, Gazebo object tracking, dynamic scene collision management, and 50 Hz interpolated hand closure.
- **`scripts/send_zero_arm.py`**:
  Safe homing script that commands all 7 arm joints to 0.0 (straight forward) and commands the LEAP hand to an open opposition posture. Compatible with both simulation and physical hardware.

---

## Troubleshooting & Maintenance

### Motor Fault Detection & Recovery
If any Dynamixel motor on the LEAP hand detects an error condition (such as communication timeout, overload, voltage spike, or hardware error), the driver enters a protected **HALTED** state to prevent motor burnout or gear stripping:

```text
[ERROR] Motor fault detected: <ERROR_TYPE> on motor <ID>. Hand movement halted to prevent damage.
[WARN]  Hand is HALTED due to motor fault. Awaiting reboot via /leap_hand/reboot...
```

**Recovery Procedure:**
1. Check the hand to ensure fingers are not physically caught or blocked.
2. Trigger the reboot recovery service:
   ```bash
   rosservice call /leap_hand/reboot
   ```
   Or execute the recovery script from `leap_inhand`:
   ```bash
   python3 /root/Projects/leap_inhand/leap_ethernet/scripts/leap_ethernet_reboot.py
   ```
3. The OpenRB-150 board will reinitialize motor parameters and restore normal operation without needing a full node restart.

### PS4 Controller Not Responding
1. Verify device recognition:
   ```bash
   ls -la /dev/input/js*
   ```
2. Test joystick button and axis events directly:
   ```bash
   jstest /dev/input/js0
   ```
3. If using Docker, ensure the container was started with `--device=/dev/input` and appropriate permissions.

---
---

# Previous IRVL Documentation (ATI-Gamma F/T & UMI Deformable Fingers)

> [!NOTE]
> The documentation below preserves the historical instructions for the Fetch robot configuration with the **ATI-Gamma Force/Torque Sensor**, calibration workflows, and **UMI Deformable Fingers**.

ROS Components for Robots from Fetch Robotics used at the Intelligent Robotics and Vision Lab at the University of Texas at Dallas. Our Fetch Robot uses an ATI-Gamma F/T sensor at the wrist; in this forked repository you will find the updated components for our robot, such as the new .urdf, moveit configuration, and a ros package to interface with the sensor. 

### Table of Contents (Legacy)

- [New .Urdf](#new-urdf)
  - [Moveit Configuration](#moveit-configuration-legacy)
  - [Robot Calibration](#robot-calibration)
    - [Notes](#notes)
- [fetch\_netft package](#fetch_netft-package)
  - [Notes](#notes-1)
- [New Deformable Fingers](#new-deformable-fingers)
- [fetch\_ros Original Readme.md](#fetch_ros-original-readmemd)

## New .Urdf 

As mentioned before, our Fetch robot is equipped with an ATI-Gamma force and torque sensor at the wrist. Consequently, the original .urdf can no longer be used for the proper control of the robot. We created a new .urdf [fetch.urdf](/fetch_description/robots/fetch.urdf) to replace the original at the fetch_description base ros package located within the '/opt/ros/' directory. 

<p align="center">
<img src="./media/robot.jpeg"  height="500" alt="IRVL Fetch Robot">
</p>

Replacing the original .urdf file will make the robot boot with the new robot representation which will respect the new joint limits when using teleoperation. In the case that the robot has been calibrated the change will only take effect if the calibration is reseted to it's factory model. You can do this by running:

```Shell
    calibrate_robot --reset
```

Check if the change took effect by running 
```Shell
    calibrate_robot --date
```
The output should be 'uncalibrated'.

### Moveit Configuration (Legacy)
Since we have updated the .urdf file we also made modifications to the [fetch_moveit_config](/fetch_moveit_config/) package. First, we added the new 'ati_link' to the [fetch.srdf](/fetch_moveit_config/config/fetch.srdf) for the simplification of the MoveIt path planning. Second, we reduced the joint velocity limits in [joint_limits.yaml](/fetch_moveit_config/config/joint_limits.yaml) to more reasonable speeds.

After the changes have been made, ensure they are loaded by restarting the roscore service. NOTE: The fetch_drivers will reset; place the fetch robot in a resting position! (arm tucked, torso down) before running the command.

```Shell
    sudo service roscore restart
```

### Robot Calibration
To calibrate our updated robot we had to make some adjustments to the files used in the calibration process referred to at the [Fetch and Freight manual](https://docs.fetchrobotics.com/calibration.html). The calibration robot process uses the RGB-D camera to detect the positions of 4 LEDs in the Gripper while performing 99 different poses. To calibrate the new robot model we had to: 

1) Modify the calibration poses in [calibration_poses.bag](/fetch_calibration/config/calibration_poses.bag) to ensure that the LEDs are visible by the RGB-D camera. This was done using the script [modify_poses.py](/fetch_calibration/config/modify_poses.py) by adding an offset to the *wrist_roll_joint*. 
2) The 4 LED positions were updated in [capture.yaml](/fetch_calibration/config/capture.yaml). The original positions are with respect to the original *wrist_roll_joint*; we used a simple transformation to devise the new positions. [point_angle_rotation.py](/fetch_calibration/config/point_angle_rotation.py)
3) After modifying the files you can follow the instructions in the [Fetch and Freight manual](https://docs.fetchrobotics.com/calibration.html) to calibrate. Note that after running the command:
```Shell
    calibrate_robot --arm --install --velocity-factor=0.5
```
If you have sudo privilege the robot drivers will restart immediately after performing the calibration. i.e. the arm will become unactuated and fall. We also recommend using the *velocity-factor* argument to reduce the speed of the robot.

#### Notes
- The tuck_arm routine [tuck_arm.py](/fetch_teleop/scripts/tuck_arm.py) was modified to account for our new robot model and to ensure no collisions with the installed equipment. For our robot the script was located within the '/opt/ros/melodic/lib/fetch_teleop/' directory.
- We included our calibrated model in [calibrated_files](/fetch_description/robots/calibrated/).

## fetch_netft package
To interface with the ATI sensor NetFT device, we developed a ROS package that let's you activate the NetFT UDP/RDT data stream and publish the values to a rostopic. We assigned the NetFT the IP address: 10.42.42.41 on the Fetch internal network. The [sensor.py](/fetch_netft/scripts/sensor.py) script connects to the NetFT and publishes the wrench values as `WrenchStamped` in the 4 different rostopics. You can initialize the sensor data stream by running the launch file:

```Shell
roslaunch fetch_netft netft.launch
```

The published Wrench values are with respect to our fetch.urdf `ati_link`.

<p align="center">
<video src="media/sensor_video.mp4" controls></video>
</p>

The four rostopics are:
- `gripper/ft_sensor/raw` for raw values sensed by the force torque sensor (software bias is set when initialized).
- `gripper/ft_sensor/absolute` uses the gripper information to publish the absolute force and torque sensed by the force sensor. 
- `gripper/ft_sensor/external` the calculated external forces on the sensor after accounting for the gripper weight and CoG at the current gripper pose.
- `gripper/ft_sensor/external_imu` calculated external forces using the gripper IMU directly.

### Notes

- You can find the UDP interface documentation in the [NetFT Manual](https://www.ati-ia.com/app_content/documents/9610-05-1022.pdf).

## New Deformable Fingers
We have also changed the original Fetch fingers to a adapted version of the [UMI deformable fingers](https://umi-gripper.github.io/). Both .urdfs modeling the original fingers and the new deformable fingers are included in the [fetch_description](/fetch_description/robots/) package. 

<p align="center">
<img src="./media/fingers.jpeg" height="400" alt="Deformable Fingers Image">
</p>

For easily changing the urdf model in the ROS environment we added a launch file on the `fetch_moveit_config` package. This file, [moveit.launch](./fetch_moveit_config/launch/moveit.launch), receives as input the robot urdf to load in the ROS environment, rewrites the robot_description in the system and launches moveit [move_group.launch](./fetch_moveit_config/launch/move_group.launch) from the original fetch_ros. Sample command is shown below:

```Shell
roslaunch fetch_moveit_config moveit.launch robot:=fetch_original
```


# fetch_ros Original Readme.md

Open ROS Components for Robots from Fetch Robotics

## Documentation

Please refer to our documentation page: http://docs.fetchrobotics.com/

## ROS Buildfarm Development Branches

Fetch Package | Indigo Devel | Kinetic Devel | Melodic Devel | ROS 1 (Noetic) | Dashing Devel
------------- | ------------ | ------------- | ------------- | -------------- | -------------
fetch_ros |  EOL | :negative_squared_cross_mark: not supported | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__fetch_ros__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__fetch_ros__ubuntu_bionic_amd64/) | :hammer_and_wrench: forthcoming :hammer_and_wrench: | :construction: in planning :construction:

Kinetic support, has been skipped. Ubuntu 18.04 and 20.04 support both ROS1 and ROS2, so it was decided that effort will go towards ROS2 support in the future instead of supporting Kinetic. Noetic support is in the works, also.

## ROS Buildfarm Release

Fetch Package | Indigo | Kinetic * | Melodic Source | Melodic Debian
------------- | ------ | --------- | -------------- | --------------
fetch_calibration | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_calibration__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_calibration__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_calibration__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_calibration__ubuntu_bionic_amd64__binary/)| 
fetch_depth_layer | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_depth_layer__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_depth_layer__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_depth_layer__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_depth_layer__ubuntu_bionic_amd64__binary/) |
fetch_description | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_description__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_description__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_description__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_description__ubuntu_bionic_amd64__binary/) |
fetch_ikfast_plugin | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_ikfast_plugin__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_ikfast_plugin__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_ikfast_plugin__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_ikfast_plugin__ubuntu_bionic_amd64__binary/) |
fetch_maps | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_maps__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_maps__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_maps__ubuntu_bionic__source)](http://build.ros.org/job/Mbin_uB64__fetch_maps__ubuntu_bionic__source/) | :negative_squared_cross_mark: | 
fetch_moveit_config | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_moveit_config__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_moveit_config__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_moveit_config__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_moveit_config__ubuntu_bionic_amd64__binary/) | 
fetch_navigation | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_navigation__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_navigation__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_navigation__ubuntu_bionic_amd64__binary/) |
fetch_teleop | EOL | :negative_squared_cross_mark: | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__fetch_teleop__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__fetch_teleop__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__fetch_teleop__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__fetch_teleop__ubuntu_bionic_amd64__binary/) |
