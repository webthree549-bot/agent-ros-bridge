# Planning Architecture V1

## Overview

This document describes the motion planning architecture for Agent ROS Bridge v0.6.1. The architecture consists of two primary ROS nodes: `/ai/motion_planner` and `/ai/execution_monitor`, integrated with Nav2 for navigation and MoveIt2 for manipulation.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MOTION PLANNING SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  /ai/motion_planner Node                                            │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │   │
│  │  │  Intent         │    │  Motion         │    │  SMT-based      │  │   │
│  │  │  Parser         │───→│  Primitive      │───→│  Verification   │  │   │
│  │  │  Interface      │    │  Library        │    │  Engine         │  │   │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │   │
│  │           │                      │                      │            │   │
│  │           ▼                      ▼                      ▼            │   │
│  │  ┌─────────────────────────────────────────────────────────────┐    │   │
│  │  │              Safety Certificate Generator                    │    │   │
│  │  │  - Formal proof of constraint satisfaction                   │    │   │
│  │  │  - Valid time window (30 seconds)                           │    │   │
│  │  │  - Unique validation ID for traceability                    │    │   │
│  │  └─────────────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼ (Validated Motion Plan)                 │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  /safety/validator Node (Independent, Hardware-Enforced)            │   │
│  │                                                                      │   │
│  │  - Verify safety certificate authenticity                           │   │
│  │  - Check against hardware-enforced limits                           │   │
│  │  - <10ms response time (hard real-time)                             │   │
│  │  - CANNOT be overridden by AI layer                                 │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼ (If Approved)                           │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  EXECUTION LAYER                                                    │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │   │
│  │  │  Nav2           │    │  MoveIt2        │    │  Controller     │  │   │
│  │  │  Navigator      │    │  Move Group     │    │  Server         │  │   │
│  │  │  (Navigation)   │    │  (Manipulation) │    │  (Low-level)    │  │   │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
│                                    ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  /ai/execution_monitor Node                                         │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │   │
│  │  │  Progress       │    │  Anomaly        │    │  Recovery       │  │   │
│  │  │  Tracking       │    │  Detection      │    │  Strategies     │  │   │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Details

### 1. /ai/motion_planner Node

**Purpose:** Generate verified motion plans from high-level intents

**Responsibilities:**
- Receive intent + constraints from context manager
- Select appropriate motion primitives from library
- Generate candidate motion plans
- Verify plans using SMT solver
- Generate safety certificates for valid plans
- Publish planning metrics

**ROS Interfaces:**

| Interface | Type | Name | Description |
|-----------|------|------|-------------|
| Action Server | `PlanMotion` | `/ai/plan_motion` | Long-running planning with feedback |
| Service | `QuickMotionPlan` | `/ai/quick_motion_plan` | Quick planning for simple motions |
| Publisher | `PlanningMetrics` | `/ai/planning_metrics` | Planning performance data |
| Subscriber | `WorldState` | `/world_state` | Current world state |
| Service Client | `ValidateMotion` | `/safety/validate_motion` | Safety validation |

**Internal Components:**

```python
class MotionPlannerNode:
    def __init__(self):
        self.primitive_library = PreVerifiedPrimitiveLibrary()
        self.verifier = SMTVerificationEngine()
        self.certificate_generator = SafetyCertificateGenerator()
        self.world_state_cache = WorldStateCache()
```

**Planning Pipeline:**

1. **Intent Parsing** (10% progress)
   - Parse intent and constraints
   - Validate input parameters

2. **Primitive Selection** (30% progress)
   - Select motion primitives from library
   - Compose primitives for complex motions

3. **Plan Generation** (50% progress)
   - Generate candidate motion plan
   - Compute trajectories

4. **Formal Verification** (80% progress)
   - Verify with SMT solver
   - Check safety constraints

5. **Certificate Generation** (95% progress)
   - Generate safety certificate
   - Sign and timestamp

6. **Completion** (100% progress)
   - Return validated plan

---

### 2. /ai/execution_monitor Node

**Purpose:** Monitor motion execution and handle anomalies

**Responsibilities:**
- Monitor execution progress of motion plans
- Detect anomalies (deviation, obstacles, stuck conditions)
- Track execution metrics
- Trigger recovery behaviors
- Report execution status

**ROS Interfaces:**

| Interface | Type | Name | Description |
|-----------|------|------|-------------|
| Publisher | `ExecutionStatus` | `/ai/execution_status` | Current execution status |
| Publisher | `AnomalyReport` | `/ai/anomaly_report` | Detected anomalies |
| Subscriber | `MotionPlan` | `/ai/approved_motion_plan` | Approved plans to monitor |
| Subscriber | `Odometry` | `/odom` | Robot odometry |
| Subscriber | `JointState` | `/joint_states` | Arm joint states |

**Internal Components:**

```python
class ExecutionMonitorNode:
    def __init__(self):
        self.progress_tracker = ProgressTracker()
        self.anomaly_detector = AnomalyDetector()
        self.recovery_manager = RecoveryManager()
        self.state_estimator = RobotStateEstimator()
```

**Monitoring Loop (10Hz):**

1. Get current robot state
2. Update executed trajectory
3. Calculate progress percentage
4. Calculate deviation from expected trajectory
5. Detect anomalies
6. Publish execution status
7. Handle anomalies if detected

---

## Integration with Safety Validator

### Safety Certificate Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Motion Planner │     │  Safety Validator│     │  Execution Layer │
│                 │     │                 │     │                 │
│  Generate Plan  │────→│  Validate Plan  │────→│  Execute Plan   │
│                 │     │                 │     │                 │
│  Create Cert    │←────│  Approve/Reject │     │  Monitor        │
│  (if approved)  │     │                 │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

### Safety Certificate Structure

```yaml
SafetyCertificate:
  plan_hash: string           # SHA-256 hash of plan
  validation_id: string       # UUID for traceability
  valid_from: time            # Certificate start time
  valid_until: time           # Certificate expiry (30s window)
  constraints_checked:        # List of verified constraints
    - collision_free
    - velocity_limits
    - workspace_bounds
  planner_version: string     # Motion planner version
  verification_engine: string # "SMT" or other
  world_state_hash: string    # Hash of world state at validation
  signature: string           # Cryptographic signature
```

### Validation Requirements

1. **Timing:** <10ms response time
2. **Independence:** Cannot be overridden by AI layer
3. **Hardware-enforced:** Uses independent safety limits
4. **Traceability:** All validations logged with unique IDs

---

## Integration with Nav2

### Service/Action Interfaces

| Nav2 Interface | Type | Purpose | Mapping |
|----------------|------|---------|---------|
| `/navigate_to_pose` | Action | Navigate to goal | `navigate_to_pose` primitive |
| `/follow_path` | Action | Follow waypoints | `follow_path` primitive |
| `/compute_path_to_pose` | Service | Path planning | Pre-validation |
| `/clear_costmap` | Service | Clear obstacles | Recovery behavior |

### Parameter Mappings

| Motion Primitive | Nav2 Parameter | Default | Description |
|------------------|----------------|---------|-------------|
| `max_speed` | `max_vel_x` | 0.5 m/s | Maximum linear velocity |
| `clearance` | `inflation_radius` | 0.3 m | Obstacle inflation |
| `behavior_tree` | `behavior_tree` | "default" | BT XML file |

### Error Handling

| Error Code | Cause | Recovery |
|------------|-------|----------|
| `NAVIGATION_CANCELLED` | User cancelled | Report cancellation |
| `NAVIGATION_FAILED` | Planning failed | Replan with relaxed constraints |
| `NAVIGATION_TIMEOUT` | Execution timeout | Escalate to human |
| `COLLISION_AVOIDED` | Obstacle detected | Replan around obstacle |

### Timeout Configurations

```yaml
nav2_timeouts:
  planning_timeout: 5.0        # seconds
  execution_timeout: 300.0     # seconds (5 minutes max)
  controller_timeout: 1.0      # seconds
  recovery_timeout: 30.0       # seconds
```

---

## Integration with MoveIt2

### Service/Action Interfaces

| MoveIt2 Interface | Type | Purpose | Mapping |
|-------------------|------|---------|---------|
| `/move_action` | Action | Execute motion | `move_arm`, `move_cartesian` |
| `/plan_kinematic_path` | Service | Plan motion | Pre-validation |
| `/execute_trajectory` | Action | Execute trajectory | Motion execution |
| `/get_planning_scene` | Service | Get scene | Collision checking |

### Parameter Mappings

| Motion Primitive | MoveIt2 Parameter | Default | Description |
|------------------|-------------------|---------|-------------|
| `max_velocity` | `max_velocity_scaling_factor` | 0.5 | Velocity scaling |
| `planning_time` | `allowed_planning_time` | 5.0 s | Planning timeout |
| `joint_positions` | `joint_constraints` | - | Target joint values |

### Error Handling

| Error Code | Cause | Recovery |
|------------|-------|----------|
| `PLANNING_FAILED` | No valid plan found | Retry with different planner |
| `INVALID_MOTION_PLAN` | Plan validation failed | Replan with stricter constraints |
| `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT` | Scene changed | Replan with updated scene |
| `CONTROL_FAILED` | Controller error | Escalate to human |

### Timeout Configurations

```yaml
moveit2_timeouts:
  planning_timeout: 5.0        # seconds
  execution_timeout: 60.0      # seconds
  scene_update_timeout: 1.0    # seconds
  controller_timeout: 5.0      # seconds
```

---

## Recovery Strategies Overview

### Recovery Strategy Library

| Anomaly Type | Recovery Strategies | Priority |
|--------------|---------------------|----------|
| `STUCK` | ClearCostmap → Rotate → BackUp → Escalate | High |
| `DEVIATION` | Replan → ReturnToPath → Continue | Medium |
| `OBSTACLE` | Wait → Replan → AskHuman | High |
| `TIMEOUT` | Continue → Escalate | Medium |
| `SENSOR_FAILURE` | Retry → FallbackSensor → Escalate | High |
| `COLLISION_RISK` | EmergencyStop | Critical |

### Recovery Execution Flow

```
Anomaly Detected
       │
       ▼
┌──────────────┐
│  Classify    │
│  Anomaly     │
└──────────────┘
       │
       ▼
┌──────────────┐
│  Select      │
│  Strategy    │
└──────────────┘
       │
       ▼
┌──────────────┐     ┌──────────────┐
│  Execute     │────→│  Success?    │
│  Recovery    │     │              │
└──────────────┘     └──────────────┘
                            │
                    ┌───────┴───────┐
                    ▼               ▼
              ┌─────────┐     ┌─────────┐
              │  Yes    │     │   No    │
              │ Resume  │     │ Next    │
              │         │     │ Strategy│
              └─────────┘     └─────────┘
```

---

## Node Communication Flow

### Complete Execution Flow

```
User: "Go to the kitchen slowly"
         │
         ▼
┌────────────────────┐
│ /ai/intent_parser  │ ──→ Intent: NAVIGATE, target: kitchen, speed: 0.3 m/s
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ /ai/context_manager│ ──→ Resolved: kitchen → (5.2, 3.1, 0.0) in map frame
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ /ai/motion_planner │ ──→ Generate candidate trajectory
│                    │     Verify with SMT solver
│                    │     Generate safety certificate
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ /safety/validator  │ ──→ Validate certificate
│ (Hardware-enforced)│     Check against hard limits
│                    │     Approve/Reject (<10ms)
└────────────────────┘
         │
         ▼ (If Approved)
┌────────────────────┐
│ Nav2 / MoveIt2     │ ──→ Execute motion plan
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ /ai/execution_     │ ──→ Monitor progress, detect anomalies,
│     monitor        │     trigger recovery if needed
└────────────────────┘
```

---

## Configuration

### Motion Planner Parameters

```yaml
/ai/motion_planner:
  planning_timeout_sec: 5.0
  max_planning_attempts: 3
  require_safety_certificate: true
  verification_engine: "SMT"
  smt_timeout_ms: 1000
  certificate_validity_sec: 30
```

### Execution Monitor Parameters

```yaml
/ai/execution_monitor:
  position_tolerance: 0.1        # meters
  orientation_tolerance: 0.1     # radians
  stuck_timeout_sec: 10.0
  progress_check_interval_sec: 1.0
  anomaly_detection_rate: 10.0   # Hz
  max_recovery_attempts: 3
```

---

## Performance Requirements

| Metric | Target | Measurement |
|--------|--------|-------------|
| Planning latency | <5s | Time from request to plan |
| Verification latency | <1s | SMT solver timeout |
| Safety validation | <10ms | Hardware validator response |
| Monitoring rate | 10Hz | Execution status updates |
| Recovery trigger | <100ms | Anomaly to recovery start |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-03-06 | Initial architecture definition |

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Complete
