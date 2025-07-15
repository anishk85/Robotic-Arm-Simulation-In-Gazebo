# MyCobot 280 ROS2 MoveIt Workspace

This repository contains a complete ROS2 workspace for controlling the MyCobot 280 robotic arm using MoveIt motion planning framework and Gazebo simulation.

## Overview

The workspace includes:
- **MyCobot 280 URDF/Xacro description** with gripper
- **MoveIt configuration** for motion planning
- **Gazebo simulation** environment
- **ROS2 controllers** for joint and gripper control
- **Teleop key control** for manual robot operation

## Demo Video

Watch the demonstration of the MyCobot 280 ROS2 MoveIt Workspace in action:

[![Demo Video](https://img.youtube.com/vi/QXOHVTIC47E/0.jpg)](https://www.youtube.com/watch?v=QXOHVTIC47E)

### Installation

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update && sudo apt install ros-humble-desktop

# Install MoveIt2 and Gazebo dependencies
sudo apt install \
    ros-humble-moveit \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf-transformations \
    ros-humble-moveit-msgs
```

## Workspace Structure

```
ros2_mycobot_ws/
├── src/
│   ├── gazebo/                          # Gazebo launch files
│   │   └── launch/
│   │       └── demo_classic.launch.py   # Main demo launch file
│   ├── mycobot_description/             # Robot URDF/Xacro files
│   ├── mycobot_moveit_config_new/       # MoveIt configuration
│   │   ├── config/
│   │   │   ├── mycobot_280.srdf         # Semantic robot description
│   │   │   ├── mycobot_280.urdf.xacro   # Robot description
│   │   │   ├── moveit.rviz              # RViz configuration
│   │   │   └── controllers.yaml         # Robot controllers config
│   │   └── worlds/
│   │       └── empty.world              # Gazebo world file
│   └── mycobot_teleop/                  # Teleop control package
│       ├── mycobot_teleop/
│       │   └── mycobot_teleop_key.py    # Keyboard teleop node
│       └── launch/
│           └── ros_teleop.launch.py     # Teleop launch file
└── readme.md                            # This file
```

## Building the Workspace

```bash
# Clone and build the workspace
cd ~/ros2_mycobot_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Launch Gazebo Simulation with MoveIt

Start the complete simulation environment:

```bash
ros2 launch gazebo demo_classic.launch.py
```

This launches:
- **Gazebo** with the MyCobot 280 robot
- **RViz** with MoveIt motion planning interface
- **MoveIt move_group** node for motion planning
- **Joint controllers** for robot control
- **Gripper controller** for gripper operation

### 2. Motion Planning in RViz

1. In RViz, select the **MotionPlanning** plugin
2. Set planning group to **"robotarm"**
3. Choose motion planner (e.g., **CHOMP** or **OMPL**)
4. Drag the interactive markers to set goal poses
5. Click **"Plan & Execute"** to move the robot

### 3. Keyboard Teleop Control (TODO)

Launch the teleop node for manual control:

```bash
# In a new terminal
ros2 run mycobot_teleop mycobot_teleop_key
```

#### Teleop Controls

**Joint Control:**
- `q/a` : Joint 1 (Base) +/-
- `w/s` : Joint 2 (Shoulder) +/-
- `e/d` : Joint 3 (Elbow) +/-
- `r/f` : Joint 4 (Wrist 1) +/-
- `t/g` : Joint 5 (Wrist 2) +/-
- `y/h` : Joint 6 (Wrist 3) +/-

**Control:**
- `+/=` : Increase step size
- `-/_` : Decrease step size

**Utility:**
- `p` : Print current joint positions
- `0` : Move to home position
- `x` : Exit

## Robot Specifications

### MyCobot 280 M5Stack

- **6-DOF** articulated robotic arm
- **Working radius:** 280mm
- **Payload:** 250g
- **Repeatability:** ±0.5mm
- **Joint ranges:** ±160° (most joints)

### Included Gripper

- **Adaptive gripper** with parallel jaws
- **Opening range:** 0-20mm
- **Controlled via:** GripperActionController

## Package Details

### mycobot_description
Contains URDF/Xacro files describing the robot's physical structure:
- Links and joints definitions
- Visual and collision meshes
- Inertial properties
- Gazebo-specific properties

### mycobot_moveit_config_new
MoveIt configuration package including:
- **SRDF** (Semantic Robot Description Format)
- **Planning groups** definitions
- **Controller configuration**
- **Kinematics solvers** setup
- **Planning algorithms** configuration

### mycobot_teleop
Keyboard teleop control package:
- Direct joint trajectory control
- Real-time robot manipulation
- Safety limits and error handling

## Controllers

The robot uses the following ROS2 controllers:

1. **joint_state_broadcaster**: Publishes joint states
2. **robot_controller**: JointTrajectoryController for arm joints
3. **gripper_action_controller**: GripperActionController for gripper

## Topics

### Important Topics

- `/joint_states` - Current robot joint positions
- `/robot_controller/joint_trajectory` - Joint trajectory commands
- `/move_group/goal` - MoveIt planning goals
- `/gripper_action_controller/gripper_cmd` - Gripper commands

### Monitoring Robot State

```bash
# View current joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# Monitor trajectory execution
ros2 topic echo /robot_controller/controller_state
```

## Troubleshooting

### Robot Not Moving

1. **Check controllers are active:**
   ```bash
   ros2 control list_controllers
   ```

2. **Verify joint trajectory topic:**
   ```bash
   ros2 topic list | grep trajectory
   ```

3. **Test manual trajectory:**
   ```bash
   ros2 topic pub --once /robot_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."
   ```

### MoveIt Planning Issues

1. **Set planning context to CHOMP** in RViz
2. **Check planning group** is set to "robotarm"
3. **Increase planning time** if planning fails
4. **Verify robot model** loads correctly in RViz

### Gazebo Issues

1. **Check Gazebo plugins** are loaded
2. **Verify URDF/Xacro** files are valid
3. **Ensure controllers** are spawned correctly

## Development

### Adding New Features

1. **Custom end-effectors:** Modify URDF and SRDF files
2. **New planning algorithms:** Update MoveIt configuration
3. **Additional controllers:** Extend controllers.yaml

### Testing

```bash
# Build and test
colcon build --packages-select mycobot_teleop
colcon test --packages-select mycobot_teleop

# Check for issues
colcon build --symlink-install
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS2 and MoveIt documentation
3. Open an issue on the repository

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [Gazebo Classic Documentation](http://gazebosim.org/tutorials)
- [MyCobot 280 Official Documentation](https://docs.elephantrobotics.com/)
