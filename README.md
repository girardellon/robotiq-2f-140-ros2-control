# Robotiq 2F-140 ROS2 Control

ROS 2 Humble + ros2_control integration for the Robotiq 2F-140 gripper, including custom hardware interface and position-based controller with real and dummy driver support.

---

## Overview

This repository provides a complete ROS 2 control stack for the Robotiq 2F-140 parallel gripper.

It includes:

- Parametric URDF/xacro model
- ros2_control SystemInterface hardware plugin
- ForwardCommand position controller
- Dummy driver for simulation and development
- Real hardware driver support
- Standalone bringup launch file

The package is designed for research and industrial manipulation pipelines and is fully compatible with ROS 2 Humble.

---

## Repository Structure

```
robotiq_2f_140_custom/
├── robotiq_2f_140_custom_description/
├── robotiq_2f_140_custom_hardware/
└── robotiq_2f_140_custom_bringup/
```

### robotiq_2f_140_custom_description
- Parametric URDF/xacro model
- ros2_control wrapper file
- Custom STL meshes

### robotiq_2f_140_custom_hardware
- Custom ros2_control SystemInterface implementation
- Driver abstraction layer
- Dummy (fake) driver support
- Real serial-based hardware driver support

### robotiq_2f_140_custom_bringup
- ros2_control_node launch
- Controller spawners
- RViz integration

---

## System Requirements

- ROS 2 Humble
- ros2_control
- ros2_controllers
- xacro
- robot_state_publisher
- rviz2

---

## Build Instructions

Clone the repository inside your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/girardellon/robotiq-2f-140-ros2-control.git
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch (Dummy Mode)

The dummy driver allows full controller testing without real hardware:

```bash
ros2 launch robotiq_2f_140_custom_bringup bringup.launch.py
```

---

## Available Controllers

After launch:

```bash
ros2 control list_controllers
```

Expected controllers:

- `gripper_position_controller`
- `joint_state_broadcaster`

---

## Command the Gripper

Close the gripper:

```bash
ros2 topic pub --once \
/gripper_position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.07]}"
```

Open the gripper:

```bash
ros2 topic pub --once \
/gripper_position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

The command corresponds to the prismatic joint position in meters, bounded by the configured stroke parameter.

---

## Real Hardware Mode

To connect to real hardware:

Set:

```
use_dummy:=false
```

inside the ros2_control xacro wrapper and configure the appropriate serial parameters (e.g., slave address, baud rate, port) according to your system.

---

## Integration

This control stack is designed to integrate seamlessly with:

- MoveIt 2 manipulation pipelines
- UR5e arm configurations
- Modular robotic manipulation systems
- Research and industrial automation workflows

---

## License

Apache License 2.0
