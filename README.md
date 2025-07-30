# mycobot320_ros2

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy?logo=ubuntu)

![ROS_2](https://img.shields.io/ros/v/humble/rclcpp?logo=ros)

![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python)

![License](https://img.shields.io/badge/License-BSD--3--Clause-green)

---

This ROS 2 package provides symbolic and numeric tools to analyze the kinematics and singularities 
of the myCobot320 robot. It combines the flexibility of the Python Robotics Toolbox with ROS 2 tools 
like RViz, joint state publishing, and MoveIt-based collision validation.

---

## Features

- Forward and inverse kinematics (symbolic + numeric)
- Jacobian and singularity analysis
- Nullspace trajectory generation
- Visualization via RViz (`joint_state_publisher`)
- Integration-ready with MoveIt for collision checking

---

## Setup installation

### 1. Install ROS and dependencies

Make sure ROS2 (Humble) is installed and sourced.

Install system dependencies:

```bash
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
`rosdep` scans the `.xml` files to determine which dependencies need to be installed from apt.

### 2. Install PyPi requirements
```bash
pip install -r requirements.txt
```
These are not resolvable via `rosdep` as they come from `PyPI` (e.g. roboticstoolbox-python).

## Launch commands

### URDF slider check

```bash
ros2 launch mycobot320_description display_.launch.py
```

### Configurations

In order to see 1 pose reached with the 8 configurations run:
```bash
ros2 launch mycobot320_analysis show_configurations.launch.py
```

### Singularities

#### Shoulder
```bash
ros2 launch mycobot320_analysis analyze_singularity_shoulder.launch.py
```

#### Elbow
```bash
ros2 launch mycobot320_analysis analyze_singularity_elbow.launch.py
```

#### Wrist
```bash
ros2 launch mycobot320_analysis analyze_singularity_wrist.launch.py
```

### CLI menu

Ir order to have a complete analysis and understanding of the MyCobot320.py

```bash
./run_analysis_menu.py 
```

## License

This project is licensed under the BSD-3-Clause License.

## Maintainer

Tomás de Aguirre
tomas.a.de.aguirre@gmail.com