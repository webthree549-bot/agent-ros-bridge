# Nav2 Integration Specification

## Overview

This document specifies the integration between Agent ROS Bridge's motion planning system and the Nav2 (Navigation2) stack for ROS2. Nav2 provides production-grade navigation capabilities including path planning, obstacle avoidance, and recovery behaviors.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         NAV2 INTEGRATION ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────┐         ┌─────────────────────┐                   │
│  │  /ai/motion_planner │         │  /ai/execution_     │                   │
│  │                     │         │     monitor         │                   │
│  └──────────┬──────────┘         └──────────┬──────────┘                   │
│             │                               │                               │
│             │ MotionPlan                    │ ExecutionStatus               │
│             ▼                               ▼                               │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │              Nav2Integration Layer                   │                   │
│  │                                                      │                   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │                   │
│  │  │   Navigate  │  │   Follow    │  │   Recovery  │  │                   │
│  │  │   To Pose   │  │    Path     │  │   Manager   │  │                   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │                   │
│  │         │                │                │         │                   │
│  └─────────┼────────────────┼────────────────┼─────────┘                   │
│            │                │                │                              │
│            ▼                ▼                ▼                              │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │                    NAV2 STACK                        │                   │
│  │                                                      │                   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌────────┐  │                   │
│  │  │ Planner │  │ Controller│ │  BT Navigator │  │ Recovery│  │                   │
│  │  │ Server  │  │ Server  │  │             │  │ Server  │  │                   │
│  │  │(NavFn/  │  │ (DWB/   │  │             │  │         │  │                   │
│  │  │ SMAC)   │  │  TEB)   │  │             │  │         │  │                   │
│  │  └────┬────┘  └────┬────┘  └──────┬──────┘  └────┬───┘  │                   │
│  │       │            │              │              │      │                   │
│  └───────┼────────────┼──────────────┼──────────────┼──────┘                   │
│          │            │              │              │                          │
│          ▼            ▼              ▼              ▼                          │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │              ROBOT HARDWARE INTERFACE                │                   │
│  │                                                      │                   │
│  │         cmd_vel ←──→ Controller ←──→ Motors         │                   │
│  │         scan    ←──→ Lidar                          │                   │
│  │         odom    ←──→ Encoders/IMU                   │                   │
│  └─────────────────────────────────────────────────────┘                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Service/Action Interfaces

### Nav2 Actions

| Action | Type | Purpose | Input | Output |
|--------|------|---------|-------|--------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Navigate to goal pose | `PoseStamped goal`, `string behavior_tree` | `bool success`, `string error_msg` |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Navigate through waypoints | `PoseStamped[] poses` | `bool success`, `string error_msg` |
| `/follow_path` | `nav2_msgs/action/FollowPath` | Follow predefined path | `Path path`, `string controller_id` | `bool success`, `string error_msg` |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Compute path to goal | `PoseStamped goal`, `string planner_id` | `Path path`, `bool success` |
| `/spin` | `nav2_msgs/action/Spin` | Rotate in place | `float32 target_yaw` | `bool success` |
| `/backup` | `nav2_msgs/action/BackUp` | Back up linearly | `float32 distance` | `bool success` |
| `/wait` | `nav2_msgs/action/Wait` | Wait in place | `float32 duration` | `bool success` |

### Nav2 Services

| Service | Type | Purpose | Input | Output |
|---------|------|---------|-------|--------|
| `/clear_costmap` | `nav2_msgs/srv/ClearCostmap` | Clear costmap | `bool reset` | `bool success` |
| `/get_costmap` | `nav2_msgs/srv/GetCostmap` | Get costmap data | - | `Costmap costmap` |
| `/is_path_valid` | `nav2_msgs/srv/IsPathValid` | Check path validity | `Path path` | `bool valid` |
| `/change_state` | `lifecycle_msgs/srv/ChangeState` | Lifecycle state | `uint8 transition` | `bool success` |
| `/get_state` | `lifecycle_msgs/srv/GetState` | Get current state | - | `uint8 state` |

### Agent ROS Bridge Services

| Service | Type | Purpose |
|---------|------|---------|
| `/ai/navigate` | `agent_ros_bridge_msgs/srv/Navigate` | High-level navigation interface |
| `/ai/follow_path` | `agent_ros_bridge_msgs/srv/FollowPath` | Path following interface |
| `/ai/dock` | `agent_ros_bridge_msgs/srv/Dock` | Docking interface |

---

## Parameter Mappings

### Motion Primitive to Nav2 Parameters

#### navigate_to_pose

| Primitive Param | Nav2 Param | Default | Description |
|-----------------|------------|---------|-------------|
| `target_pose` | `goal` | required | Goal pose in map frame |
| `max_speed` | `max_vel_x` | 0.5 m/s | Maximum linear velocity |
| `max_angular_speed` | `max_vel_theta` | 1.0 rad/s | Maximum angular velocity |
| `clearance` | `inflation_radius` | 0.3 m | Obstacle inflation radius |
| `behavior_tree` | `behavior_tree` | "navigate_to_pose_w_replanning_and_recovery.xml" | BT XML file |

#### follow_path

| Primitive Param | Nav2 Param | Default | Description |
|-----------------|------------|---------|-------------|
| `waypoints` | `path` | required | Path waypoints |
| `velocity_profile` | `controller_id` | "FollowPath" | Controller to use |
| `max_acceleration` | `max_acc_x` | 0.5 m/s² | Maximum linear acceleration |

#### rotate_in_place

| Primitive Param | Nav2 Param | Default | Description |
|-----------------|------------|---------|-------------|
| `target_yaw` | `target_yaw` | required | Target orientation |
| `angular_velocity` | `max_vel_theta` | 0.5 rad/s | Rotation speed |

---

## Behavior Tree Integration

### Default Behavior Trees

| BT File | Use Case | Description |
|---------|----------|-------------|
| `navigate_to_pose_w_replanning_and_recovery.xml` | Standard navigation | Replanning + recovery behaviors |
| `navigate_to_pose_w_replanning_and_recovery_safety.xml` | High-safety navigation | Extra safety checks + slower speeds |
| `follow_path.xml` | Path following | Direct path execution |
| `dock.xml` | Docking | Precise approach + docking |

### Custom Behavior Tree Nodes

```xml
<!-- Safety-enhanced navigation BT -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- Pre-navigation safety check -->
      <SafetyCheck clearance="0.5"/>
      
      <!-- Clear costmap for fresh planning -->
      <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
      <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
      
      <!-- Compute path -->
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      
      <!-- Validate path with safety validator -->
      <ValidatePath path="{path}" certificate="{safety_cert}"/>
      
      <!-- Follow path with monitoring -->
      <FollowPath path="{path}" controller_id="FollowPath"/>
      
      <!-- Post-navigation verification -->
      <GoalReached goal="{goal}" tolerance="0.3"/>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## Error Handling

### Nav2 Error Codes

| Error Code | Value | Cause | Recovery Strategy |
|------------|-------|-------|-------------------|
| `NAVIGATION_CANCELLED` | 1 | User cancelled | Report cancellation, clean up |
| `NAVIGATION_FAILED` | 2 | Planning failed | Replan with relaxed constraints |
| `NAVIGATION_TIMEOUT` | 3 | Execution timeout | Escalate to human operator |
| `COLLISION_AVOIDED` | 4 | Obstacle detected | Replan around obstacle |
| `INVALID_GOAL` | 5 | Goal not valid | Report invalid goal |
| `PLANNING_FAILED` | 6 | No path found | Try alternative planner |
| `CONTROLLER_FAILED` | 7 | Controller error | Switch controller, retry |

### Error Recovery Flow

```
Error Detected
      │
      ▼
┌─────────────┐
│ Log Error   │
│ with Context│
└─────────────┘
      │
      ▼
┌─────────────┐
│ Classify    │
│ Error Type  │
└─────────────┘
      │
      ▼
┌─────────────┐
│ Can Retry?  │
└─────────────┘
      │
   ┌──┴──┐
   ▼     ▼
┌─────┐ ┌─────┐
│ Yes │ │ No  │
└──┬──┘ └──┬──┘
   │       │
   ▼       ▼
┌─────────┐ ┌─────────┐
│ Retry   │ │ Escalate│
│ with    │ │ to Human│
│ Recovery│ │         │
└─────────┘ └─────────┘
```

---

## Timeout Configurations

### Planning Timeouts

```yaml
nav2_planning:
  planner_server:
    ros__parameters:
      expected_planner_frequency: 20.0
      planner_plugins: ["GridBased"]
      GridBased:
        plugin: "nav2_navfn_planner/NavfnPlanner"
        tolerance: 0.5
        use_astar: true
        allow_unknown: false
        planning_timeout: 5.0  # seconds
```

### Controller Timeouts

```yaml
nav2_controller:
  controller_server:
    ros__parameters:
      controller_frequency: 20.0
      min_x_velocity_threshold: 0.001
      min_y_velocity_threshold: 0.5
      min_theta_velocity_threshold: 0.001
      failure_tolerance: 0.3
      progress_checker_plugin: "progress_checker"
      goal_checker_plugin: "goal_checker"
      controller_plugins: ["FollowPath"]
      
      progress_checker:
        plugin: "nav2_controller::SimpleProgressChecker"
        required_movement_radius: 0.5
        movement_time_allowance: 10.0  # seconds
        
      goal_checker:
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.25
        yaw_goal_tolerance: 0.25
        stateful: true
```

### Behavior Tree Timeouts

```yaml
nav2_bt_navigator:
  bt_navigator:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      odom_topic: /odom
      default_bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"
      plugin_lib_names:
        - nav2_compute_path_to_pose_action_bt_node
        - nav2_follow_path_action_bt_node
        - nav2_clear_costmap_service_bt_node
      
      # Timeouts
      navigate_to_pose:
        timeout: 300.0  # 5 minutes
      navigate_through_poses:
        timeout: 600.0  # 10 minutes
```

---

## Safety Integration

### Safety Certificate Validation

Before executing any navigation command, the safety certificate must be validated:

```python
class Nav2SafetyIntegration:
    def validate_navigation(self, motion_plan, safety_certificate):
        # Check certificate validity
        if not self.verify_certificate(safety_certificate):
            return ValidationResult(
                valid=False,
                reason="Invalid safety certificate"
            )
        
        # Check path against certificate constraints
        path = motion_plan.trajectories[0]
        if not self.check_path_constraints(path, safety_certificate):
            return ValidationResult(
                valid=False,
                reason="Path violates safety constraints"
            )
        
        # Check velocity limits
        max_speed = motion_plan.primitives[0].get_param('max_speed')
        if max_speed > safety_certificate.max_velocity:
            return ValidationResult(
                valid=False,
                reason="Velocity exceeds safety limit"
            )
        
        return ValidationResult(valid=True)
```

### Emergency Stop Integration

```python
class Nav2EmergencyStop:
    def __init__(self):
        self.e_stop_pub = node.create_publisher(
            Bool, '/emergency_stop', 10
        )
        self.nav2_cancel_pub = node.create_publisher(
            Empty, '/navigate_to_pose/cancel', 10
        )
    
    def trigger_emergency_stop(self):
        # Stop robot immediately
        self.e_stop_pub.publish(Bool(data=True))
        
        # Cancel current navigation
        self.nav2_cancel_pub.publish(Empty())
        
        # Clear velocity commands
        cmd_vel_pub.publish(Twist())  # Zero velocity
```

---

## Configuration Examples

### TurtleBot3 Configuration

```yaml
# nav2_turtlebot3.yaml
nav2_navigation:
  amcl:
    ros__parameters:
      min_particles: 500
      max_particles: 2000
      laser_min_range: 0.12
      laser_max_range: 3.5
      
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
        
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      
  controller_server:
    ros__parameters:
      controller_frequency: 10.0
      controller_plugins: ["FollowPath"]
      FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        debug_trajectory_details: true
        min_vel_x: 0.0
        max_vel_x: 0.26
        min_vel_y: 0.0
        max_vel_y: 0.0
        max_vel_theta: 1.0
        min_speed_xy: 0.0
        max_speed_xy: 0.26
        min_speed_theta: 0.0
        acc_lim_x: 2.5
        acc_lim_y: 0.0
        acc_lim_theta: 3.2
        decel_lim_x: -2.5
        decel_lim_y: 0.0
        decel_lim_theta: -3.2
```

---

## Performance Monitoring

### Metrics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/ai/nav2_metrics/planning_time` | `Float64` | Path planning duration |
| `/ai/nav2_metrics/execution_time` | `Float64` | Navigation execution duration |
| `/ai/nav2_metrics/path_length` | `Float64` | Planned path length |
| `/ai/nav2_metrics/recovery_count` | `Int32` | Number of recovery behaviors |
| `/ai/nav2_metrics/costmap_update_rate` | `Float64` | Costmap update frequency |

### Performance Requirements

| Metric | Target | Measurement |
|--------|--------|-------------|
| Planning time | <5s | Time to compute path |
| Controller rate | 10Hz | Control loop frequency |
| Costmap update | 5Hz | Local costmap update rate |
| Goal tolerance | 0.3m | Position accuracy |
| Orientation tolerance | 0.1 rad | Heading accuracy |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-03-06 | Initial Nav2 integration spec |

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Complete
