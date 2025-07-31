# mycobot320_description

This package contains the URDF model and associated resources for the myCobot 320 robot arm.

It provides the robot's kinematic structure, 3D description for visualization in RViz and simulation in MoveIt.

## Contents

- Improved Xacro-based URDF model and meshes for the myCobot 320 M5 version, adapted from Elephant Robotics  
  (source: https://github.com/elephantrobotics/mycobot_ros2/blob/humble/mycobot_description/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf)
- Joint definitions, link geometries, and coordinate transforms.
- RViz-friendly configuration for robot visualization.

## Usage

To launch a simple RViz visualization with interactive sliders:

```bash
ros2 launch mycobot320_description display.launch.py
```

This will display the robot using the joint_state_publisher_gui, allowing you to adjust joint angles manually.