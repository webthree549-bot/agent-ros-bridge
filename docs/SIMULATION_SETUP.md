# Simulation Setup Guide

This document provides complete instructions for setting up and using the Agent ROS Bridge simulation environment.

## Overview

The simulation environment provides a safe, reproducible testing ground for developing and validating Agent ROS Bridge components before real-world deployment.

## Prerequisites

- Ubuntu 22.04 (recommended)
- ROS2 Humble
- Gazebo Ignition Fortress
- Docker (optional, for containerized development)

## Quick Start

### Option 1: Docker Development Environment (Recommended)

```bash
# Navigate to the docker directory
cd docker/

# Build the development image
docker-compose build

# Start the simulation container
docker-compose up -d

# Enter the container
docker-compose exec sim bash

# Inside the container, source ROS and run simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simulation sim_bringup.launch.py
```

### Option 2: Native Installation

```bash
# Install ROS2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install Gazebo Ignition Fortress
sudo apt-get update
sudo apt-get install -y ignition-fortress ros-humble-gazebo-ros-pkgs

# Install additional dependencies
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-moveit2

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Launch simulation
ros2 launch simulation sim_bringup.launch.py
```

## Directory Structure

```
simulation/
├── worlds/              # Gazebo world files
│   └── empty_warehouse.sdf    # 20m x 20m test environment
├── models/              # Robot and object models
│   ├── turtlebot3_waffle/     # Mobile robot with camera and lidar
│   └── ur5_arm/               # 6-DOF manipulator arm
├── scenarios/           # Test scenario definitions
│   └── basic_navigation.yaml  # Sample navigation test
└── launch/              # ROS2 launch files
    └── sim_bringup.launch.py  # Main simulation launcher
```

## Running the Simulation

### Basic Launch

```bash
# Launch Gazebo with empty warehouse world and TurtleBot3
ros2 launch simulation sim_bringup.launch.py
```

This will:
- Start Gazebo Ignition with the empty warehouse world
- Spawn a TurtleBot3 at position (-8, -8)
- Start the ROS bridge (WebSocket on port 9090)
- Start the Agent ROS Bridge (WebSocket on port 8765)

### Viewing Topics

```bash
# List all topics
ros2 topic list

# View camera feed
ros2 topic echo /camera/image_raw

# View lidar scans
ros2 topic echo /scan

# View odometry
ros2 topic echo /odom

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### Teleoperation

```bash
# Install teleop package
sudo apt-get install ros-humble-teleop-twist-keyboard

# Run keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Adding New Models

### 1. Create Model Directory

```bash
mkdir -p simulation/models/my_robot
touch simulation/models/my_robot/model.config
touch simulation/models/my_robot/model.sdf
```

### 2. Define Model Configuration

Create `model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  <description>Description of your robot</description>
</model>
```

### 3. Define Model SDF

Create `model.sdf` with:
- Links (visual and collision geometries)
- Joints (if articulated)
- Sensors (cameras, lidar, IMU, etc.)
- Plugins (diff drive, joint control, etc.)

See existing models (`turtlebot3_waffle`, `ur5_arm`) for examples.

### 4. Spawn in Launch File

Add to `sim_bringup.launch.py`:

```python
spawn_my_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'my_robot',
        '-file', os.path.join(model_path, 'my_robot', 'model.sdf'),
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.1'
    ],
    output='screen'
)
```

## Creating Test Scenarios

Scenarios are YAML files that define test conditions and success criteria.

### Scenario Structure

```yaml
scenario:
  name: "Test Name"
  description: "What this test validates"
  world: "world_file.sdf"
  duration: 60
  
  robots:
    - name: "robot_1"
      model: "turtlebot3_waffle"
      spawn_pose:
        x: 0.0
        y: 0.0
        z: 0.1
        yaw: 0.0
  
  goals:
    - robot: "robot_1"
      type: "navigate_to_pose"
      target:
        x: 5.0
        y: 5.0
  
  success_criteria:
    - type: "goal_reached"
      robot: "robot_1"
      timeout: 30
    - type: "no_collision"
      robot: "robot_1"
```

### Running Scenarios

```bash
# Run a specific scenario
ros2 run simulation scenario_runner --scenario basic_navigation.yaml

# Run all scenarios
ros2 run simulation scenario_runner --all
```

## CI/CD Integration

The simulation tests are automatically run on GitHub Actions:

```yaml
# .github/workflows/ci.yml
- name: Run simulation tests
  run: |
    source install/setup.bash
    pytest tests/simulation/ -v
```

### Local CI Testing

```bash
# Run the same tests locally
act -j simulation-test
```

## Troubleshooting

### Gazebo won't start

```bash
# Check if Gazebo is already running
pkill -f gazebo

# Check display (if using Docker)
echo $DISPLAY
xhost +local:docker
```

### Models not loading

```bash
# Verify model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/simulation/models

# Check model validity
gz model --verbose -m turtlebot3_waffle
```

### ROS topics not appearing

```bash
# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Verify nodes are running
ros2 node list
```

## Advanced Configuration

### Custom Worlds

1. Copy `empty_warehouse.sdf` as a template
2. Add obstacles, lighting, and features
3. Update launch file to use new world

### Multi-Robot Simulation

Modify `sim_bringup.launch.py` to spawn multiple robots with unique namespaces:

```python
spawn_robot_1 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'robot_1',
        '-x', '-5.0',
        '-robot_namespace', 'robot_1'
    ]
)

spawn_robot_2 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'robot_2',
        '-x', '5.0',
        '-robot_namespace', 'robot_2'
    ]
)
```

### Physics Parameters

Adjust in world SDF file:

```xml
<physics name="default_physics" default="true" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Performance Optimization

### Headless Mode

```bash
# Run Gazebo without GUI (faster, uses less resources)
gazebo --headless-rendering worlds/empty_warehouse.sdf
```

### Reducing Sensor Resolution

Edit model SDF files to reduce camera/lidar resolution for faster simulation.

### Parallel Simulation

Run multiple independent simulations in parallel for batch testing:

```bash
# Terminal 1
GAZEBO_MASTER_URI=http://localhost:11345 ros2 launch simulation sim_bringup.launch.py

# Terminal 2
GAZEBO_MASTER_URI=http://localhost:11346 ros2 launch simulation sim_bringup.launch.py
```

## Next Steps

1. Review [SIMULATION_FIRST_STRATEGY.md](SIMULATION_FIRST_STRATEGY.md) for testing philosophy
2. See [V061_SPRINT_PLAN.md](V061_SPRINT_PLAN.md) for development roadmap
3. Check [V060_BASELINE.md](V060_BASELINE.md) for system architecture

## Support

For issues or questions:
- Check existing scenarios for examples
- Review Gazebo/ROS2 documentation
- Open an issue in the project repository
