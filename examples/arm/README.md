# Arm Robot Example

**Robotic arm manipulation demo.**

## What It Does

Demonstrates control of UR, xArm, or Franka robotic arms with joint and Cartesian control.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh --arm-type ur --demo pick_place
```

Options:
- `--arm-type ur|xarm|franka`
- `--ros-version ros1|ros2`
- `--demo pick_place|interactive|state`

## Test

```bash
# In another terminal
wscat -c ws://localhost:8772

# Move joints
{"command": {"action": "arm.move_joints", "parameters": {"joints": [0, -1.57, 0, -1.57, 0, 0]}}}

# Cartesian move
{"command": {"action": "arm.move_cartesian", "parameters": {"x": 0.5, "y": 0.2, "z": 0.1}}}

# Control gripper
{"command": {"action": "arm.gripper", "parameters": {"position": 0.8}}}
```

## What's Happening

This demonstrates:
- **Joint Control**: Move individual joints
- **Cartesian Control**: Move end-effector to position
- **Gripper Control**: Open/close gripper
- **Trajectory Execution**: Multi-waypoint paths

## Supported Arms

| Arm | Type | ROS Versions |
|-----|------|--------------|
| Universal Robots | UR5, UR10, UR3e | ROS1/ROS2 |
| UFACTORY | xArm6, xArm7 | ROS2 |
| Franka | Panda | ROS2 |

## Next Steps

- Read [User Manual - Arm Robots](../../docs/USER_MANUAL.md#robot-arms)
- Connect to real arm hardware
- Try MoveIt integration
