# Robotiq 2F-140 ROS 2 Control Stack

## Overview

This repository provides a complete ROS 2 Humble control stack for a custom Robotiq 2F-140 parallel gripper, including:

- Parametric URDF/Xacro model
- ros2_control integration
- Custom SystemInterface hardware implementation
- Support for both real hardware and dummy (simulation) driver
- Standalone bringup launch
- RViz integration for visualization and debugging

The stack is designed for research-grade robotic manipulation pipelines and can be seamlessly integrated into larger robotic systems (e.g., UR-series manipulators with MoveIt 2).

This implementation enables full position-based control of the gripper through ROS 2 controllers while maintaining modularity and clean separation between description, control, and hardware layers.

---

## Repository Structure

robotiq-2f-140-ros2-control/
├── robotiq_2f_140_custom_description/
│   ├── meshes/
│   ├── urdf/
│   │   ├── robotiq_2f_140_custom.urdf.xacro
│   │   └── robotiq_2f_140_custom.ros2_control.xacro
│   ├── rviz/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robotiq_2f_140_custom_bringup/
│   ├── config/
│   │   └── ros2_controllers.yaml
│   ├── launch/
│   │   └── bringup.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robotiq_2f_140_custom_hardware/
│   ├── include/
│   ├── src/
│   ├── CMakeLists.txt
│   └── package.xml
│
└── README.md

---

## Architecture

The system follows the standard ROS 2 control architecture:

URDF/Xacro
   ↓
ros2_control SystemInterface
   ↓
Controller Manager
   ↓
ForwardCommandController (position)
   ↓
Hardware Driver (real or dummy)

Key components:

- Custom prismatic joint model with symmetric mimic behavior
- ros2_control SystemInterface implementation
- ForwardCommandController for position control
- JointStateBroadcaster for state publication
- Real hardware serial communication (via driver layer)
- FakeDriver for development without hardware
- 100 Hz control update loop

---

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble
- ros2_control
- ros2_controllers
- xacro
- rviz2
- colcon

---

## Build Instructions

Clone the repository inside a ROS 2 workspace:

cd ~/ros2_ws/src
git clone https://github.com/<YOUR_USERNAME>/robotiq-2f-140-ros2-control.git

Build:

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

---

## Running in Simulation Mode (Dummy Driver)

The stack supports a dummy hardware driver for development without physical hardware.

Launch the system:

ros2 launch robotiq_2f_140_custom_bringup bringup.launch.py

Verify controllers:

ros2 control list_controllers

Expected output:

- joint_state_broadcaster (active)
- gripper_position_controller (active)

Test closing the gripper:

ros2 topic pub --once \
/gripper_position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.07]}"

Test opening the gripper:

ros2 topic pub --once \
/gripper_position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.0]}"

---

## Running with Real Hardware

To enable real hardware communication:

1. Set the following parameter in robotiq_2f_140_custom.ros2_control.xacro:

<param name="use_dummy">false</param>

2. Configure:
   - Serial port
   - Slave address
   - Speed multiplier
   - Force multiplier

3. Rebuild and relaunch.

---

## Model Description

The gripper is modeled using:

- robotiq_2f_140_base_link
- robotiq_2f_140_left_finger_link
- robotiq_2f_140_right_finger_link

Active joint:

- robotiq_2f_140_left_finger_joint (prismatic)

Mimic joint:

- robotiq_2f_140_right_finger_joint

Default stroke: 0.07 m (70 mm)

Joint limits are fully configurable via Xacro parameters.

---

## Parameters

The following Xacro arguments are exposed:

- stroke (default: 0.07)
- mesh_scale (default: 0.001)
- use_collision (default: true)
- use_dummy (default: true)

---

## Integration

This stack is designed to integrate with:

- UR5e + MoveIt 2 pipelines
- Industrial robotic manipulation frameworks
- Multi-end-effector robotic systems
- Research prototypes requiring modular end-effector control

The modular design allows seamless embedding into composite robot descriptions.

---

## Known Limitations

- Real-time scheduling requires proper OS configuration
- Serial communication parameters must match hardware configuration
- No trajectory controller is included (position-only control)

---

## License

This project is released under the Apache License 2.0.
