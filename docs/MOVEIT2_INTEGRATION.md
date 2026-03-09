# MoveIt2 Integration Specification

## Overview

This document specifies the integration between Agent ROS Bridge's motion planning system and MoveIt2 for ROS2. MoveIt2 provides advanced manipulation capabilities including motion planning, collision checking, and trajectory execution for robotic arms.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       MOVEIT2 INTEGRATION ARCHITECTURE                      │
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
│  │            MoveIt2Integration Layer                  │                   │
│  │                                                      │                   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │                   │
│  │  │   Move Arm  │  │   Pick/     │  │   Cartesian │  │                   │
│  │  │  (Joint)    │  │   Place     │  │   Motion    │  │                   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │                   │
│  │         │                │                │         │                   │
│  └─────────┼────────────────┼────────────────┼─────────┘                   │
│            │                │                │                              │
│            ▼                ▼                ▼                              │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │                  MOVEIT2 STACK                       │                   │
│  │                                                      │                   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐  │                   │
│  │  │  Move    │ │ Planning │ │  Scene   │ │ Traj   │  │                   │
│  │  │  Group   │ │ Pipeline │ │ Monitor  │ │ Exec   │  │                   │
│  │  │  Node    │ │ (OMPL/   │ │          │ │        │  │                   │
│  │  │          │ │ CHOMP/   │ │          │ │        │  │                   │
│  │  │          │ │ PILZ)    │ │          │ │        │  │                   │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └───┬────┘  │                   │
│  │       │            │            │           │       │                   │
│  └───────┼────────────┼────────────┼───────────┼───────┘                   │
│          │            │            │           │                           │
│          ▼            ▼            ▼           ▼                           │
│  ┌─────────────────────────────────────────────────────┐                   │
│  │              ROBOT ARM HARDWARE                      │                   │
│  │                                                      │                   │
│  │    Joint commands ←──→ Controller ←──→ Servos      │                   │
│  │    Joint states   ←──→ Sensors                        │                   │
│  │    Force/Torque   ←──→ F/T Sensor                    │                   │
│  │    Gripper        ←──→ Gripper Controller            │                   │
│  └─────────────────────────────────────────────────────┘                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Service/Action Interfaces

### MoveIt2 Actions

| Action | Type | Purpose | Input | Output |
|--------|------|---------|-------|--------|
| `/move_action` | `moveit_msgs/action/MoveGroup` | Execute motion plan | `MotionPlanRequest` | `MotionPlanResponse` |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | Execute trajectory | `RobotTrajectory` | `bool success` |
| `/pick` | `moveit_msgs/action/Pick` | Pick object | `PickGoal` | `PickResult` |
| `/place` | `moveit_msgs/action/Place` | Place object | `PlaceGoal` | `PlaceResult` |

### MoveIt2 Services

| Service | Type | Purpose | Input | Output |
|---------|------|---------|-------|--------|
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` | Plan motion | `MotionPlanRequest` | `MotionPlanResponse` |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Get scene | `PlanningSceneComponents` | `PlanningScene` |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Update scene | `PlanningScene` | `bool success` |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Check state | `RobotState` | `bool valid` |
| `/get_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | Cartesian path | `GetCartesianPathRequest` | `RobotTrajectory` |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | Forward kinematics | `RobotState`, `link_names` | `PoseStamped[]` |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Inverse kinematics | `PoseStamped`, `group_name` | `RobotState` |

### Agent ROS Bridge Services

| Service | Type | Purpose |
|---------|------|---------|
| `/ai/move_arm` | `agent_ros_bridge_msgs/srv/MoveArm` | High-level arm motion |
| `/ai/pick_object` | `agent_ros_bridge_msgs/srv/PickObject` | Pick operation |
| `/ai/place_object` | `agent_ros_bridge_msgs/srv/PlaceObject` | Place operation |
| `/ai/move_cartesian` | `agent_ros_bridge_msgs/srv/MoveCartesian` | Cartesian motion |
| `/ai/gripper_control` | `agent_ros_bridge_msgs/srv/GripperControl` | Gripper operation |

---

## Parameter Mappings

### Motion Primitive to MoveIt2 Parameters

#### move_arm (Joint Space)

| Primitive Param | MoveIt2 Param | Default | Description |
|-----------------|---------------|---------|-------------|
| `joint_positions` | `joint_constraints` | required | Target joint values |
| `max_velocity` | `max_velocity_scaling_factor` | 0.5 | Velocity scaling (0-1) |
| `planning_time` | `allowed_planning_time` | 5.0 s | Planning timeout |
| `planning_group` | `group_name` | "manipulator" | Planning group |
| `planner_id` | `planner_id` | "RRTConnect" | Planner algorithm |

#### move_cartesian (Cartesian Space)

| Primitive Param | MoveIt2 Param | Default | Description |
|-----------------|---------------|---------|-------------|
| `end_effector_pose` | `waypoints` | required | Target pose(s) |
| `linear_velocity` | `max_velocity_scaling_factor` | 0.1 | Velocity scaling |
| `waypoint_resolution` | `jump_threshold` | 0.0 | Jump detection |
| `planning_group` | `group_name` | "manipulator" | Planning group |
| `eef_link` | `link_name` | "tool0" | End effector link |

#### pick_object

| Primitive Param | MoveIt2 Param | Default | Description |
|-----------------|---------------|---------|-------------|
| `object_pose` | `target_pose` | required | Object location |
| `grip_force` | `grasp_posture` | 5.0 N | Grasp effort |
| `approach_offset` | `pre_grasp_approach` | 0.1 m | Pre-grasp offset |
| `lift_height` | `post_grasp_retreat` | 0.05 m | Post-grasp lift |
| `object_id` | `object_id` | required | Object identifier |

#### place_object

| Primitive Param | MoveIt2 Param | Default | Description |
|-----------------|---------------|---------|-------------|
| `place_pose` | `place_pose` | required | Placement location |
| `release_height` | `post_place_retreat` | 0.02 m | Release offset |
| `place_id` | `place_id` | required | Place location ID |

#### gripper_action

| Primitive Param | MoveIt2 Param | Default | Description |
|-----------------|---------------|---------|-------------|
| `action` | `grasp_posture` | required | open/close |
| `position` | `position` | 0.0 m | Target position |
| `force` | `effort` | 5.0 N | Max effort |

---

## Planning Pipeline Configuration

### OMPL Configuration

```yaml
planning_pipeline:
  ompl:
    planning_plugin: "ompl_interface/OMPLPlanner"
    request_adapters: >
      default_planner_request_adapters/AddTimeOptimalParameterization
      default_planner_request_adapters/ResolveConstraintFrames
      default_planner_request_adapters/FixWorkspaceBounds
      default_planner_request_adapters/FixStartStateBounds
      default_planner_request_adapters/FixStartStateCollision
      default_planner_request_adapters/FixStartStatePathConstraints
    
    planner_configs:
      - RRTConnect
      - RRTstar
      - PRM
      - CHOMP
      
    RRTConnect:
      type: geometric::RRTConnect
      range: 0.0
      
    RRTstar:
      type: geometric::RRTstar
      range: 0.0
      goal_bias: 0.05
      delay_collision_checking: 1
```

### CHOMP Configuration

```yaml
planning_pipeline:
  chomp:
    planning_plugin: "chomp_interface/CHOMPPlanner"
    
    chomp:
      planning_time_limit: 10.0
      max_iterations: 200
      max_iterations_after_collision_free: 5
      smoothness_cost_weight: 0.1
      obstacle_cost_weight: 1.0
      learning_rate: 0.01
      animate_path: true
      add_randomness: false
      smoothness_cost_velocity: 0.0
      smoothness_cost_acceleration: 1.0
      smoothness_cost_jerk: 0.0
      hmc_discretization: 0.01
      hmc_stochasticity: 0.01
      hmc_annealing_factor: 0.99
      use_pseudo_inverse: false
      pseudo_inverse_ridge_factor: 1e-4
      animate_endeffector: false
      animate_endeffector_segment: ""
      joint_update_limit: 0.1
      collision_clearence: 0.2
      collision_threshold: 0.07
      random_jump_amount: 1.0
      use_stochastic_descent: true
      enable_failure_recovery: true
      max_recovery_attempts: 5
```

---

## Error Handling

### MoveIt2 Error Codes

| Error Code | Value | Cause | Recovery Strategy |
|------------|-------|-------|-------------------|
| `PLANNING_FAILED` | -1 | No valid plan found | Retry with different planner |
| `INVALID_MOTION_PLAN` | -2 | Plan validation failed | Replan with stricter constraints |
| `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT` | -3 | Scene changed | Replan with updated scene |
| `CONTROL_FAILED` | -4 | Controller error | Check controller, retry |
| `UNABLE_TO_AQUIRE_SENSOR_DATA` | -5 | Sensor failure | Use last known state, escalate |
| `TIMED_OUT` | -6 | Planning timeout | Increase timeout, simplify goal |
| `PREEMPTED` | -7 | Preempted by user | Clean up, report |
| `INVALID_GOAL` | -8 | Goal not valid | Check goal constraints |
| `INVALID_GROUP_NAME` | -9 | Group name invalid | Check configuration |
| `INVALID_ROBOT_STATE` | -10 | Robot state invalid | Re-initialize |
| `INVALID_LINK_NAME` | -11 | Link name invalid | Check URDF |
| `INVALID_OBJECT_NAME` | -12 | Object name invalid | Check scene |
| `START_STATE_VIOLATES_PATH_CONSTRAINTS` | -31 | Start state invalid | Adjust start state |
| `START_STATE_IN_COLLISION` | -32 | Start in collision | Move to safe state |

### Error Recovery Flow

```
Error Detected
      │
      ▼
┌─────────────┐
│ Log Error   │
│ with Context│
│ (State,Goal)│
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
│ Is Scene    │
│ Changed?    │
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
│ Update  │ │ Can     │
│ Scene & │ │ Retry?  │
│ Replan  │ │         │
└─────────┘ └────┬────┘
            ┌────┴────┐
            ▼         ▼
        ┌───────┐  ┌───────┐
        │ Yes   │  │ No    │
        │ Retry │  │Escalate│
        │ Plan  │  │       │
        └───────┘  └───────┘
```

---

## Timeout Configurations

### Planning Timeouts

```yaml
move_group:
  ros__parameters:
    planning_plugin: "ompl_interface/OMPLPlanner"
    planning_scene_monitor_options:
      publish_planning_scene: true
      publish_geometry_updates: true
      publish_state_updates: true
      publish_transforms_updates: true
      
    planning_request_adapter_parameters:
      default_planner_request_adapters/AddTimeOptimalParameterization:
        time_parameterization_algorithm: "time_optimal_trajectory_generation"
        resample_dt: 0.1
        
    # Planning timeouts
    planning_timeout: 5.0  # seconds
    max_safe_planning_cost: 100.0
    
    # Controller timeouts
    trajectory_execution:
      allowed_execution_duration_scaling: 1.2
      allowed_goal_duration_margin: 0.5
      allowed_start_tolerance: 0.01
      execution_duration_monitoring: true
      execution_velocity_scaling: 1.0
      wait_for_trajectory_completion: true
```

### Trajectory Execution Timeouts

```yaml
trajectory_execution:
  ros__parameters:
    # Execution monitoring
    execution_monitoring:
      enabled: true
      rate: 50.0  # Hz
      
    # Timeout settings
    allowed_execution_duration_scaling: 1.2  # 20% margin
    allowed_goal_duration_margin: 0.5  # seconds
    allowed_start_tolerance: 0.01  # joint tolerance
    
    # Controller management
    controller_names:
      - joint_trajectory_controller
      
    joint_trajectory_controller:
      controller_action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: true
      joints:
        - joint1
        - joint2
        - joint3
        - joint4
        - joint5
        - joint6
```

---

## Safety Integration

### Safety Certificate Validation

Before executing any manipulation command, the safety certificate must be validated:

```python
class MoveIt2SafetyIntegration:
    def validate_manipulation(self, motion_plan, safety_certificate):
        # Check certificate validity
        if not self.verify_certificate(safety_certificate):
            return ValidationResult(
                valid=False,
                reason="Invalid safety certificate"
            )
        
        # Check trajectory against certificate constraints
        trajectory = motion_plan.trajectories[0]
        if not self.check_trajectory_constraints(trajectory, safety_certificate):
            return ValidationResult(
                valid=False,
                reason="Trajectory violates safety constraints"
            )
        
        # Check joint velocity limits
        max_velocity = motion_plan.primitives[0].get_param('max_velocity')
        if max_velocity > safety_certificate.max_joint_velocity:
            return ValidationResult(
                valid=False,
                reason="Joint velocity exceeds safety limit"
            )
        
        # Check workspace bounds
        if not self.check_workspace_bounds(trajectory, safety_certificate):
            return ValidationResult(
                valid=False,
                reason="Trajectory exceeds workspace bounds"
            )
        
        return ValidationResult(valid=True)
```

### Emergency Stop Integration

```python
class MoveIt2EmergencyStop:
    def __init__(self):
        self.e_stop_pub = node.create_publisher(
            Bool, '/emergency_stop', 10
        )
        self.move_group_cancel_pub = node.create_publisher(
            Empty, '/move_action/cancel', 10
        )
        self.gripper_stop_pub = node.create_publisher(
            Bool, '/gripper/stop', 10
        )
    
    def trigger_emergency_stop(self):
        # Stop arm immediately
        self.e_stop_pub.publish(Bool(data=True))
        
        # Cancel current motion
        self.move_group_cancel_pub.publish(Empty())
        
        # Stop gripper
        self.gripper_stop_pub.publish(Bool(data=True))
        
        # Send zero velocity to all joints
        for joint_pub in self.joint_publishers:
            joint_pub.publish(Float64(data=0.0))
```

---

## Configuration Examples

### UR5 Configuration

```yaml
# ur5_moveit_config.yaml
move_group:
  ros__parameters:
    # Robot description
    robot_description: "robot_description"
    robot_description_semantic: "robot_description_semantic"
    
    # Planning groups
    planning_group_names:
      - manipulator
      - endeffector
      
    manipulator:
      default_planner_config: RRTConnect
      planner_configs:
        - RRTConnect
        - RRTstar
        - CHOMP
      
    # Joint limits
    joint_limits:
      shoulder_pan_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 3.15
      shoulder_lift_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 3.15
      elbow_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 3.15
      wrist_1_joint:
        has_velocity_limits: true
        max_velocity: 6.28
        has_acceleration_limits: true
        max_acceleration: 6.28
      wrist_2_joint:
        has_velocity_limits: true
        max_velocity: 6.28
        has_acceleration_limits: true
        max_acceleration: 6.28
      wrist_3_joint:
        has_velocity_limits: true
        max_velocity: 6.28
        has_acceleration_limits: true
        max_acceleration: 6.28
```

---

## Performance Monitoring

### Metrics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/ai/moveit2_metrics/planning_time` | `Float64` | Motion planning duration |
| `/ai/moveit2_metrics/execution_time` | `Float64` | Trajectory execution duration |
| `/ai/moveit2_metrics/path_quality` | `Float64` | Path smoothness metric |
| `/ai/moveit2_metrics/collision_checks` | `Int32` | Number of collision checks |
| `/ai/moveit2_metrics/ik_solutions` | `Int32` | IK solver iterations |

### Performance Requirements

| Metric | Target | Measurement |
|--------|--------|-------------|
| Planning time | <5s | Time to compute motion plan |
| IK solve rate | >100Hz | Inverse kinematics frequency |
| Collision check | >1000Hz | Collision checking frequency |
| Trajectory execution | Real-time | Controller tracking accuracy |
| Joint tolerance | 0.01 rad | Position accuracy |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-03-06 | Initial MoveIt2 integration spec |

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Complete
