# Motion Primitive Library

## Overview

The Motion Primitive Library is a collection of pre-verified, parameterized motion templates that serve as building blocks for complex robot motions. Each primitive is formally verified for safety and includes explicit parameter schemas, pre/post conditions, and expected duration estimates.

---

## Primitive Catalog

### 1. Navigation Primitives

#### 1.1 `navigate_to_pose`
Navigate the robot to a specific pose in the map frame.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `target_pose` | geometry_msgs/PoseStamped | required | Goal pose in map frame | Must be within mapped area |
| `max_speed` | float64 | 0.5 m/s | Maximum linear velocity | 0.0 - 1.5 m/s |
| `clearance` | float64 | 0.3 m | Minimum obstacle clearance | 0.1 - 1.0 m |
| `behavior_tree` | string | "default" | Nav2 behavior tree to use | Predefined BT names |

**Pre-conditions:**
- Robot localization is valid (AMCL confidence > 0.7)
- Target pose is within mapped area
- Path exists to target (no permanent obstacles blocking)

**Post-conditions:**
- Robot pose is within 0.3m and 0.1 rad of target
- Robot velocity is zero (stopped)

**Expected Duration:**
```
duration = distance / avg_speed + rotation_time + planning_overhead
avg_speed = min(max_speed * 0.7, 1.0)
rotation_time = abs(delta_yaw) / max_angular_velocity
planning_overhead = 2.0 seconds
```

**Safety Bounds:**
- Max linear velocity: 1.5 m/s (hard limit)
- Max angular velocity: 1.0 rad/s (hard limit)
- Min obstacle clearance: 0.1 m (hard limit)

**Integration:** Uses Nav2 `NavigateToPose` action

---

#### 1.2 `follow_path`
Follow a predefined path with velocity profiling.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `waypoints` | geometry_msgs/PoseStamped[] | required | Path waypoints | Min 2 points |
| `velocity_profile` | string | "trapezoid" | Profile type | trapezoid, cubic, constant |
| `max_acceleration` | float64 | 0.5 m/s² | Max acceleration | 0.1 - 1.0 m/s² |

**Pre-conditions:**
- Waypoints form a feasible path
- Robot is near first waypoint (< 1.0m)

**Post-conditions:**
- Robot has traversed all waypoints
- Final pose matches last waypoint within tolerance

**Expected Duration:**
```
duration = sum(segment_distance / segment_velocity) + acceleration_time
```

**Integration:** Uses Nav2 `FollowPath` action

---

#### 1.3 `rotate_in_place`
Rotate the robot to a specific orientation.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `target_yaw` | float64 | required | Target orientation (rad) | -π to π |
| `angular_velocity` | float64 | 0.5 rad/s | Rotation speed | 0.1 - 1.0 rad/s |
| `clockwise` | bool | auto | Rotation direction | auto-detected |

**Pre-conditions:**
- Area around robot is clear (360° scan)
- Robot is stationary

**Post-conditions:**
- Robot orientation is within 0.05 rad of target
- Robot is stationary

**Expected Duration:**
```
duration = abs(delta_yaw) / angular_velocity + settling_time
settling_time = 0.5 seconds
```

**Integration:** Direct cmd_vel publication with obstacle monitoring

---

#### 1.4 `dock`
Dock with a charging station or docking target.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `dock_pose` | geometry_msgs/PoseStamped | required | Dock location | - |
| `approach_velocity` | float64 | 0.1 m/s | Final approach speed | 0.05 - 0.2 m/s |
| `dock_type` | string | "charge" | Dock type | charge, deliver, pickup |

**Pre-conditions:**
- Dock is visible (AR marker or known location)
- Battery level allows docking maneuver
- Dock is unoccupied

**Post-conditions:**
- Robot is physically docked
- Charging contact established (if charge dock)
- Dock sensor confirms connection

**Expected Duration:**
```
duration = approach_distance / approach_velocity + alignment_time + connection_time
approach_distance = 0.5 m (typical)
alignment_time = 3.0 seconds
connection_time = 2.0 seconds
```

**Integration:** Uses Nav2 `Dock` action or custom docking server

---

#### 1.5 `wait`
Wait in place for a specified duration.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `duration` | float64 | required | Wait time in seconds | 0.0 - 3600.0 |
| `monitor_obstacles` | bool | true | Watch for obstacles | - |

**Pre-conditions:**
- Robot is stationary
- Area is safe (or monitoring enabled)

**Post-conditions:**
- Specified duration has elapsed
- Robot remains at same pose

**Expected Duration:** = `duration` parameter

**Integration:** Timer-based with optional obstacle monitoring

---

### 2. Manipulation Primitives

#### 2.1 `pick_object`
Pick up an object using the robot's gripper.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `object_pose` | geometry_msgs/PoseStamped | required | Object location | Within workspace |
| `grip_force` | float64 | 5.0 N | Grip force | 0.5 - 20.0 N |
| `approach_offset` | float64 | 0.1 m | Pre-grasp offset | 0.05 - 0.2 m |
| `lift_height` | float64 | 0.05 m | Post-grasp lift | 0.02 - 0.1 m |

**Pre-conditions:**
- Object is within arm workspace
- Gripper is empty
- Object pose is known with sufficient accuracy (< 2cm)

**Post-conditions:**
- Object is grasped (force feedback confirms)
- Object is lifted clear of surface
- Gripper is holding object

**Expected Duration:**
```
duration = approach_time + grasp_time + lift_time + settling_time
approach_time = approach_offset / approach_velocity
grasp_time = 1.0 second
lift_time = lift_height / lift_velocity
settling_time = 0.5 seconds
```

**Safety Bounds:**
- Max grip force: 20.0 N (hard limit)
- Min grip force: 0.5 N (to ensure hold)
- Workspace bounds: Joint limits enforced

**Integration:** Uses MoveIt2 `Pick` action or custom pick server

---

#### 2.2 `place_object`
Place a held object at a target location.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `place_pose` | geometry_msgs/PoseStamped | required | Placement location | Within workspace |
| `release_height` | float64 | 0.02 m | Height above target | 0.0 - 0.05 m |
| `release_type` | string | "gentle" | Release style | gentle, drop, place |

**Pre-conditions:**
- Gripper is holding an object
- Place pose is within workspace
- Place location is clear

**Post-conditions:**
- Object is released at target location
- Gripper is empty
- Object is stable (if gentle release)

**Expected Duration:**
```
duration = approach_time + place_time + release_time + retreat_time
approach_time = distance / approach_velocity
place_time = 1.0 second
release_time = 0.5 seconds
retreat_time = 0.5 seconds
```

**Integration:** Uses MoveIt2 `Place` action or custom place server

---

#### 2.3 `move_arm`
Move the arm to specified joint positions.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `joint_positions` | float64[] | required | Target joint angles | Within joint limits |
| `max_velocity` | float64 | 0.5 rad/s | Max joint velocity | 0.1 - 1.0 rad/s |
| `planning_time` | float64 | 5.0 s | Planning timeout | 1.0 - 30.0 s |

**Pre-conditions:**
- Joint positions are within limits
- Path is collision-free (checked by planner)

**Post-conditions:**
- Arm joints are at target positions (within tolerance)
- Arm is stationary

**Expected Duration:**
```
duration = planning_time + execution_time
execution_time = max(joint_delta / max_velocity) + settling_time
```

**Integration:** Uses MoveIt2 `MoveGroup` action

---

#### 2.4 `move_cartesian`
Move the end effector along a Cartesian path.

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `end_effector_pose` | geometry_msgs/PoseStamped | required | Target pose | Within workspace |
| `linear_velocity` | float64 | 0.1 m/s | Linear speed | 0.01 - 0.2 m/s |
| `waypoint_resolution` | float64 | 0.01 m | Path resolution | 0.001 - 0.05 m |

**Pre-conditions:**
- Target pose is reachable
- Cartesian path is collision-free

**Post-conditions:**
- End effector is at target pose
- Path was followed within tolerance

**Expected Duration:**
```
duration = cartesian_distance / linear_velocity + acceleration_time
```

**Integration:** Uses MoveIt2 `MoveGroup` with Cartesian path planning

---

#### 2.5 `gripper_action`
Control the gripper (open/close).

**Parameters:**
| Parameter | Type | Default | Description | Bounds |
|-----------|------|---------|-------------|--------|
| `action` | string | required | "open" or "close" | open, close, reset |
| `position` | float64 | 0.0 | Target position (if open) | 0.0 - 0.1 m |
| `force` | float64 | 5.0 N | Grip force (if close) | 0.0 - 20.0 N |

**Pre-conditions:**
- Gripper is operational
- No collision expected during motion

**Post-conditions:**
- Gripper is in target state
- Position/force feedback confirms state

**Expected Duration:**
```
duration = 1.0 second (typical)
```

**Integration:** Uses gripper action server or topic

---

### 3. Composite Primitives

#### 3.1 `approach_and_grasp`
Approach an object and grasp it in one motion.

**Components:**
1. `move_cartesian` to pre-grasp pose
2. `move_cartesian` to grasp pose
3. `gripper_action` (close)
4. `move_cartesian` to lift pose

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `object_pose` | geometry_msgs/PoseStamped | required | Object location |
| `approach_offset` | float64 | 0.1 m | Pre-grasp offset |
| `grip_force` | float64 | 5.0 N | Grip force |
| `lift_height` | float64 | 0.05 m | Post-grasp lift |

**Expected Duration:**
```
duration = approach_time + grasp_time + lift_time
```

---

#### 3.2 `place_and_retreat`
Place an object and retreat the arm.

**Components:**
1. `move_cartesian` to pre-place pose
2. `move_cartesian` to place pose
3. `gripper_action` (open)
4. `move_cartesian` to retreat pose

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `place_pose` | geometry_msgs/PoseStamped | required | Placement location |
| `release_height` | float64 | 0.02 m | Height above target |
| `retreat_offset` | float64 | 0.1 m | Post-place retreat |

**Expected Duration:**
```
duration = approach_time + place_time + release_time + retreat_time
```

---

#### 3.3 `navigate_and_dock`
Navigate to a dock location and dock.

**Components:**
1. `navigate_to_pose` to pre-dock pose
2. `dock` to complete docking

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dock_pose` | geometry_msgs/PoseStamped | required | Dock location |
| `pre_dock_offset` | float64 | 0.5 m | Distance before dock |
| `dock_type` | string | "charge" | Dock type |

**Expected Duration:**
```
duration = navigation_time + docking_time
```

---

#### 3.4 `scan_area`
Rotate in place and capture sensor data.

**Components:**
1. `rotate_in_place` (360°)
2. Sensor capture at intervals

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `center_pose` | geometry_msgs/PoseStamped | required | Scan center |
| `capture_interval` | float64 | 45° | Degrees between captures |
| `sensor_topic` | string | "/camera" | Sensor to trigger |

**Expected Duration:**
```
duration = 360° / angular_velocity + capture_time * num_captures
```

---

## Parameter Schemas

### Common Parameter Types

```yaml
# geometry_msgs/PoseStamped
pose_stamped:
  header:
    stamp: time
    frame_id: string
  pose:
    position: {x: float64, y: float64, z: float64}
    orientation: {x: float64, y: float64, z: float64, w: float64}

# velocity_profile
velocity_profile:
  type: string  # trapezoid, cubic, constant
  max_velocity: float64
  max_acceleration: float64
  max_deceleration: float64

# safety_bounds
safety_bounds:
  max_linear_velocity: float64
  max_angular_velocity: float64
  max_acceleration: float64
  min_obstacle_clearance: float64
  workspace_bounds:
    min: {x: float64, y: float64, z: float64}
    max: {x: float64, y: float64, z: float64}
```

---

## Pre/Post Conditions

### Pre-condition Checking

All primitives validate pre-conditions before execution:

```python
class PreconditionChecker:
    def check_navigate_to_pose(self, params, robot_state, world_state):
        checks = {
            'localization_valid': robot_state.localization.confidence > 0.7,
            'target_in_map': world_state.is_in_mapped_area(params.target_pose),
            'path_exists': world_state.path_exists(robot_state.pose, params.target_pose),
            'no_imminent_collision': world_state.clearance(robot_state.pose) > 0.3
        }
        return all(checks.values()), checks
```

### Post-condition Verification

All primitives verify post-conditions after execution:

```python
class PostconditionVerifier:
    def verify_navigate_to_pose(self, params, robot_state, expected_pose):
        checks = {
            'at_target': distance(robot_state.pose, expected_pose) < 0.3,
            'correct_orientation': angle_diff(robot_state.pose, expected_pose) < 0.1,
            'stopped': robot_state.velocity.linear < 0.01
        }
        return all(checks.values()), checks
```

---

## Duration Estimation

### Duration Calculation Framework

```python
class DurationEstimator:
    def estimate_navigate_to_pose(self, params, robot_state, world_state):
        # Calculate distance
        distance = self.calculate_distance(robot_state.pose, params.target_pose)
        
        # Calculate rotation
        target_yaw = quaternion_to_yaw(params.target_pose.orientation)
        current_yaw = quaternion_to_yaw(robot_state.pose.orientation)
        rotation = abs(normalize_angle(target_yaw - current_yaw))
        
        # Estimate components
        avg_speed = min(params.max_speed * 0.7, 1.0)  # Account for acceleration
        translation_time = distance / avg_speed
        rotation_time = rotation / self.max_angular_velocity
        planning_overhead = 2.0  # seconds
        
        # Add safety margin
        safety_margin = 1.2  # 20% buffer
        
        return (translation_time + rotation_time + planning_overhead) * safety_margin
```

---

## Integration with Nav2 and MoveIt2

### Nav2 Integration

| Primitive | Nav2 Action/Topic | Parameters Mapped |
|-----------|-------------------|-------------------|
| `navigate_to_pose` | `/navigate_to_pose` | goal, behavior_tree |
| `follow_path` | `/follow_path` | waypoints, velocity_profile |
| `rotate_in_place` | `/cmd_vel` (direct) | angular_velocity |
| `dock` | `/dock` or custom | dock_pose, approach_velocity |

### MoveIt2 Integration

| Primitive | MoveIt2 Action/Service | Parameters Mapped |
|-----------|------------------------|-------------------|
| `pick_object` | `/pick` or `MoveGroup` | object_pose, grip_force |
| `place_object` | `/place` or `MoveGroup` | place_pose, release_height |
| `move_arm` | `/move_action` | joint_positions, max_velocity |
| `move_cartesian` | `/move_action` (Cartesian) | end_effector_pose, linear_velocity |
| `gripper_action` | Gripper action server | action, position, force |

---

## Safety Certificate Requirements

All motion primitives require safety certificates for execution. The certificate includes:

1. **Primitive validation:** Pre-verified library membership
2. **Parameter bounds check:** All parameters within safe ranges
3. **World state validation:** Current world state compatible with primitive
4. **Temporal validity:** Certificate expires after 30 seconds

```yaml
safety_certificate:
  primitive_id: string
  validation_id: string (UUID)
  valid_from: time
  valid_until: time
  parameters_hash: string
  world_state_hash: string
  constraints_checked: string[]
  signature: string (cryptographic)
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-03-06 | Initial catalog definition |

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Complete
