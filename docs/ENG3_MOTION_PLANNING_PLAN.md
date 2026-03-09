# Motion Planning & Integration Architecture Plan
## Engineer #3: Motion Planning & Integration (Track A - Planning & Execution)
### Agent ROS Bridge v0.6.1-v0.7.0

**Version:** 1.0  
**Date:** 2026-03-06  
**Status:** Draft for Review

---

## 1. Executive Summary

This document outlines the architecture and implementation plan for the motion planning and execution monitoring systems in Agent ROS Bridge v0.6.1-v0.7.0. The plan covers:

- **/ai/motion_planner** node with SMT-based formal verification
- **/ai/execution_monitor** node for progress tracking and anomaly detection
- Integration with Nav2 (navigation) and MoveIt2 (manipulation)
- Safety certificate generation for motion plans

### Key Design Principles

1. **Safety-First:** All motion plans must pass formal verification before execution
2. **Deterministic Core:** Rule-based motion primitives with bounded LLM assistance
3. **ROS-Native:** Full integration with ROS2 ecosystem (Nav2, MoveIt2)
4. **Observable:** All decisions logged, replayable, and introspectable
5. **Hardware-Enforced:** Safety layer independent and cannot be overridden

---

## 2. Architecture Overview

### 2.1 Motion Planning System Architecture

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

### 2.2 Component Interactions

```
User Input: "Go to the kitchen slowly"
         │
         ▼
┌────────────────────┐
│ /ai/intent_parser  │ ──→ Intent: NAVIGATE, target: kitchen, speed: 0.3 m/s
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

## 3. Motion Primitive Library

### 3.1 Pre-Verified Motion Primitives

The motion primitive library contains a set of pre-verified, parameterized motion templates that serve as building blocks for complex motions.

#### 3.1.1 Navigation Primitives

| Primitive | Parameters | Safety Bounds | Verification |
|-----------|------------|---------------|--------------|
| `navigate_to_pose` | goal_pose, max_speed, clearance | max_speed ≤ 1.5 m/s, clearance ≥ 0.3m | Collision-free path check |
| `follow_path` | waypoints, velocity_profile | acceleration ≤ 0.5 m/s² | Dynamic feasibility |
| `dock` | dock_pose, approach_velocity | approach_velocity ≤ 0.1 m/s | Final pose precision |
| `rotate_in_place` | angle, angular_velocity | angular_velocity ≤ 1.0 rad/s | Obstacle clearance |
| `wait` | duration | N/A | Timeout enforcement |

#### 3.1.2 Manipulation Primitives

| Primitive | Parameters | Safety Bounds | Verification |
|-----------|------------|---------------|--------------|
| `pick_object` | object_pose, grip_force | grip_force ≤ max_payload | Grasp stability check |
| `place_object` | place_pose, release_height | release_height ≥ safe_drop | Collision check |
| `move_arm` | joint_positions, max_velocity | joint_vel ≤ joint_limits | Joint limit check |
| `move_cartesian` | end_effector_pose, linear_vel | linear_vel ≤ 0.1 m/s | Workspace bounds |
| `push_object` | direction, force, distance | force ≤ max_push_force | Object stability |

#### 3.1.3 Composite Primitives

| Primitive | Components | Use Case |
|-----------|------------|----------|
| `approach_and_grasp` | approach → grasp → lift | Pick operations |
| `place_and_retreat` | approach → place → retreat | Place operations |
| `navigate_and_dock` | navigate → align → dock | Charging station |
| `scan_area` | rotate → capture → rotate | 360° scanning |

### 3.2 Motion Primitive Definition Schema

```python
# agent_ros_bridge_msgs/msg/MotionPrimitive.msg

# Primitive identification
string primitive_id          # Unique identifier (UUID)
string primitive_type        # e.g., "navigate_to_pose", "pick_object"
string version               # Semantic version

# Parameters
Parameter[] parameters       # Key-value parameter list

# Safety bounds (enforced)
SafetyBounds hard_bounds     # Cannot be exceeded
SafetyBounds soft_bounds     # Warning threshold

# Verification metadata
bool pre_verified            # True if from verified library
string verification_proof    # Hash of verification certificate

# Execution constraints
duration max_execution_time  # Timeout for execution
float64 max_energy_consumption  # Battery limit
```

### 3.3 Pre-Verification Process

All motion primitives in the library undergo formal verification:

```python
class MotionPrimitiveVerifier:
    """
    Formal verification of motion primitives using SMT solver.
    """
    
    def verify_primitive(self, primitive: MotionPrimitive) -> VerificationResult:
        """
        Verify a motion primitive against safety constraints.
        
        Checks:
        1. Kinematic feasibility (joint limits, workspace)
        2. Dynamic feasibility (velocity, acceleration limits)
        3. Collision avoidance (obstacle clearance)
        4. Energy constraints (battery consumption)
        5. Temporal constraints (execution time)
        
        Returns:
            VerificationResult with proof or counterexample
        """
        # Encode constraints as SMT formulas
        constraints = self._encode_constraints(primitive)
        
        # Solve with SMT solver (Z3, CVC5, etc.)
        solver = SMTSolver()
        solver.add(constraints)
        
        if solver.check() == sat:
            return VerificationResult(
                verified=True,
                proof=solver.proof(),
                model=solver.model()
            )
        else:
            return VerificationResult(
                verified=False,
                counterexample=solver.counterexample(),
                violated_constraint=solver.unsat_core()
            )
```

---

## 4. Motion Planner Node (/ai/motion_planner)

### 4.1 Node Architecture

```python
# agent_ros_bridge/nodes/motion_planner.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from agent_ros_bridge_msgs.action import GenerateMotionPlan
from agent_ros_bridge_msgs.msg import MotionPlan, SafetyCertificate

class MotionPlannerNode(Node):
    """
    ROS-native motion planner with SMT-based formal verification.
    
    Responsibilities:
    1. Receive intent + constraints from context manager
    2. Generate candidate motion plans using primitives
    3. Verify plans using SMT solver
    4. Generate safety certificates for valid plans
    5. Publish planning metrics for monitoring
    """
    
    def __init__(self):
        super().__init__('motion_planner')
        
        # ROS2 Action Server: Long-running planning with feedback
        self._action_server = ActionServer(
            self,
            GenerateMotionPlan,
            '/ai/generate_motion_plan',
            self._execute_planning_callback
        )
        
        # ROS2 Service: Quick planning for simple motions
        self._quick_plan_service = self.create_service(
            QuickMotionPlan,
            '/ai/quick_motion_plan',
            self._quick_plan_callback
        )
        
        # Publishers for monitoring
        self._planning_metrics_pub = self.create_publisher(
            PlanningMetrics,
            '/ai/planning_metrics',
            10
        )
        
        # Subscribers for context
        self._world_state_sub = self.create_subscription(
            WorldState,
            '/world_state',
            self._world_state_callback,
            10
        )
        
        # Motion primitive library (pre-verified)
        self._primitive_library = PreVerifiedPrimitiveLibrary()
        
        # SMT verification engine
        self._verifier = SMTVerificationEngine()
        
        # Current world state cache
        self._current_world_state = None
        
        # Parameters
        self.declare_parameter('planning_timeout_sec', 5.0)
        self.declare_parameter('max_planning_attempts', 3)
        self.declare_parameter('require_safety_certificate', True)
        self.declare_parameter('verification_engine', 'SMT')
        
    async def _execute_planning_callback(self, goal_handle):
        """
        Execute motion planning with formal verification.
        
        Action phases:
        1. INTENT_PARSING (10%)
        2. PRIMITIVE_SELECTION (30%)
        3. PLAN_GENERATION (50%)
        4. FORMAL_VERIFICATION (80%)
        5. CERTIFICATE_GENERATION (95%)
        6. COMPLETION (100%)
        """
        request = goal_handle.request
        
        try:
            # Phase 1: Parse intent
            goal_handle.publish_feedback(GenerateMotionPlan.Feedback(
                stage='INTENT_PARSING',
                progress_percent=10,
                message='Parsing intent and constraints'
            ))
            
            intent = self._parse_intent(request.intent_msg)
            constraints = self._parse_constraints(request.constraints)
            
            # Phase 2: Select primitives
            goal_handle.publish_feedback(GenerateMotionPlan.Feedback(
                stage='PRIMITIVE_SELECTION',
                progress_percent=30,
                message='Selecting motion primitives'
            ))
            
            primitives = self._select_primitives(intent, constraints)
            
            # Phase 3: Generate candidate plan
            goal_handle.publish_feedback(GenerateMotionPlan.Feedback(
                stage='PLAN_GENERATION',
                progress_percent=50,
                message='Generating candidate motion plan'
            ))
            
            candidate_plan = self._generate_plan(primitives, constraints)
            
            # Phase 4: Formal verification
            goal_handle.publish_feedback(GenerateMotionPlan.Feedback(
                stage='FORMAL_VERIFICATION',
                progress_percent=80,
                message='Verifying plan with SMT solver'
            ))
            
            verification = self._verifier.verify(
                candidate_plan,
                world_state=self._current_world_state,
                constraints=constraints
            )
            
            if not verification.is_safe:
                # Plan rejected - provide feedback
                goal_handle.abort()
                return GenerateMotionPlan.Result(
                    success=False,
                    rejection_reason=verification.counterexample,
                    suggested_fixes=verification.suggestions,
                    violated_constraints=verification.violated_constraints
                )
            
            # Phase 5: Generate safety certificate
            goal_handle.publish_feedback(GenerateMotionPlan.Feedback(
                stage='CERTIFICATE_GENERATION',
                progress_percent=95,
                message='Generating safety certificate'
            ))
            
            certificate = self._generate_safety_certificate(
                candidate_plan,
                verification
            )
            
            # Phase 6: Success
            goal_handle.succeed()
            
            result = GenerateMotionPlan.Result(
                success=True,
                plan=candidate_plan.to_msg(),
                safety_certificate=certificate,
                planning_time_ms=self._get_planning_time(),
                verification_proof=verification.proof
            )
            
            # Publish metrics
            self._publish_planning_metrics(result)
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Planning failed: {str(e)}')
            goal_handle.abort()
            return GenerateMotionPlan.Result(
                success=False,
                rejection_reason=str(e)
            )
```

### 4.2 SMT-Based Verification Engine

```python
class SMTVerificationEngine:
    """
    Formal verification of motion plans using SMT solving.
    
    Uses Z3 or CVC5 to verify that motion plans satisfy
    all safety constraints.
    """
    
    def __init__(self):
        self._solver = z3.Solver()
        self._timeout_ms = 1000  # 1 second timeout
        
    def verify(self, 
               plan: MotionPlan,
               world_state: WorldState,
               constraints: PlanningConstraints) -> VerificationResult:
        """
        Verify a motion plan against safety constraints.
        
        Args:
            plan: The motion plan to verify
            world_state: Current world state (obstacles, etc.)
            constraints: Planning constraints from intent
            
        Returns:
            VerificationResult with proof or counterexample
        """
        self._solver.reset()
        
        # Encode plan trajectory
        trajectory_vars = self._encode_trajectory(plan)
        
        # Encode safety constraints
        safety_constraints = self._encode_safety_constraints(
            trajectory_vars, 
            world_state
        )
        
        # Encode dynamic constraints
        dynamic_constraints = self._encode_dynamic_constraints(
            trajectory_vars,
            constraints
        )
        
        # Add all constraints to solver
        self._solver.add(safety_constraints)
        self._solver.add(dynamic_constraints)
        
        # Set timeout
        self._solver.set(timeout=self._timeout_ms)
        
        # Check satisfiability
        result = self._solver.check()
        
        if result == z3.sat:
            model = self._solver.model()
            return VerificationResult(
                is_safe=True,
                proof=self._extract_proof(model),
                model=model
            )
        else:
            return VerificationResult(
                is_safe=False,
                counterexample=self._extract_counterexample(),
                violated_constraints=self._extract_unsat_core(),
                suggestions=self._generate_suggestions(plan, constraints)
            )
    
    def _encode_safety_constraints(self, 
                                   trajectory_vars: TrajectoryVars,
                                   world_state: WorldState) -> z3.BoolRef:
        """
        Encode safety constraints as SMT formulas.
        
        Constraints:
        - Collision avoidance: distance(obstacle, robot) > clearance
        - Joint limits: joint_min <= joint_pos <= joint_max
        - Workspace bounds: robot_pose in workspace
        - Velocity limits: |velocity| <= max_velocity
        """
        constraints = []
        
        # Collision avoidance for each timestep
        for t, pose_var in enumerate(trajectory_vars.poses):
            for obstacle in world_state.obstacles:
                # distance(robot_pose, obstacle) > clearance
                dist = self._euclidean_distance(pose_var, obstacle.position)
                constraints.append(dist > obstacle.clearance)
        
        # Joint limits (for manipulation)
        for t, joint_var in enumerate(trajectory_vars.joint_positions):
            for i, joint in enumerate(joint_var):
                constraints.append(joint >= self._joint_limits[i].min)
                constraints.append(joint <= self._joint_limits[i].max)
        
        return z3.And(constraints)
    
    def _encode_dynamic_constraints(self,
                                   trajectory_vars: TrajectoryVars,
                                   constraints: PlanningConstraints) -> z3.BoolRef:
        """
        Encode dynamic feasibility constraints.
        
        Constraints:
        - Velocity bounds: |v| <= v_max
        - Acceleration bounds: |a| <= a_max
        - Jerk bounds: |jerk| <= j_max (for smoothness)
        """
        constraints = []
        
        # Velocity constraints
        for vel_var in trajectory_vars.velocities:
            constraints.append(z3.Abs(vel_var.linear.x) <= constraints.max_linear_vel)
            constraints.append(z3.Abs(vel_var.angular.z) <= constraints.max_angular_vel)
        
        # Acceleration constraints (finite difference)
        for i in range(1, len(trajectory_vars.velocities)):
            dt = trajectory_vars.timestamps[i] - trajectory_vars.timestamps[i-1]
            acc = (trajectory_vars.velocities[i] - trajectory_vars.velocities[i-1]) / dt
            constraints.append(z3.Abs(acc.linear.x) <= constraints.max_linear_acc)
        
        return z3.And(constraints)
```

### 4.3 Safety Certificate Generation

```python
class SafetyCertificateGenerator:
    """
    Generate safety certificates for verified motion plans.
    
    A safety certificate is a cryptographically signed proof
    that a motion plan satisfies all safety constraints.
    """
    
    def generate(self,
                 plan: MotionPlan,
                 verification: VerificationResult) -> SafetyCertificate:
        """
        Generate a safety certificate for a verified plan.
        
        The certificate includes:
        1. Plan hash (for integrity verification)
        2. Verification proof (from SMT solver)
        3. Valid time window (certificate expiration)
        4. Unique validation ID (for traceability)
        5. Constraints checked (for audit trail)
        6. Digital signature (for authenticity)
        """
        # Compute plan hash
        plan_hash = self._compute_plan_hash(plan)
        
        # Create certificate
        certificate = SafetyCertificate(
            plan_hash=plan_hash,
            verification_proof=verification.proof,
            valid_from=self._get_current_time(),
            valid_until=self._get_current_time() + Duration(seconds=30),
            validation_id=self._generate_validation_id(),
            constraints_checked=self._list_constraints_checked(verification),
            planner_version=self._get_planner_version(),
            verification_engine='SMT',
            world_state_hash=self._compute_world_state_hash()
        )
        
        # Sign certificate
        certificate.signature = self._sign_certificate(certificate)
        
        return certificate
    
    def verify_certificate(self, 
                          certificate: SafetyCertificate,
                          plan: MotionPlan) -> bool:
        """
        Verify that a certificate is valid for a given plan.
        
        Checks:
        1. Certificate signature is valid
        2. Plan hash matches certificate
        3. Certificate has not expired
        4. World state is still valid
        """
        # Check signature
        if not self._verify_signature(certificate):
            return False
        
        # Check plan hash
        if certificate.plan_hash != self._compute_plan_hash(plan):
            return False
        
        # Check expiration
        if self._get_current_time() > certificate.valid_until:
            return False
        
        return True
```

---

## 5. Execution Monitor Node (/ai/execution_monitor)

### 5.1 Node Architecture

```python
# agent_ros_bridge/nodes/execution_monitor.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from agent_ros_bridge_msgs.msg import ExecutionStatus, AnomalyReport

class ExecutionMonitorNode(Node):
    """
    ROS-native execution monitor for progress tracking and anomaly detection.
    
    Responsibilities:
    1. Monitor execution progress of motion plans
    2. Detect anomalies (deviation from plan, unexpected obstacles)
    3. Track execution metrics (time, energy, accuracy)
    4. Trigger recovery behaviors when needed
    5. Report execution status to higher-level systems
    """
    
    def __init__(self):
        super().__init__('execution_monitor')
        
        # Publishers
        self._status_pub = self.create_publisher(
            ExecutionStatus,
            '/ai/execution_status',
            10
        )
        
        self._anomaly_pub = self.create_publisher(
            AnomalyReport,
            '/ai/anomaly_report',
            10
        )
        
        # Subscribers
        self._plan_sub = self.create_subscription(
            MotionPlan,
            '/ai/approved_motion_plan',
            self._on_new_plan,
            10
        )
        
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._on_odometry,
            10
        )
        
        self._joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10
        )
        
        # Active plan tracking
        self._active_plan = None
        self._plan_start_time = None
        self._expected_trajectory = None
        self._executed_trajectory = []
        
        # Monitoring state
        self._monitoring_timer = self.create_timer(0.1, self._monitoring_loop)
        
        # Parameters
        self.declare_parameter('position_tolerance', 0.1)  # meters
        self.declare_parameter('orientation_tolerance', 0.1)  # radians
        self.declare_parameter('stuck_timeout_sec', 10.0)
        self.declare_parameter('progress_check_interval_sec', 1.0)
        
    def _monitoring_loop(self):
        """
        Main monitoring loop - runs at 10Hz.
        
        Performs:
        1. Progress tracking
        2. Deviation detection
        3. Anomaly detection
        4. Recovery triggering
        """
        if self._active_plan is None:
            return
        
        # Get current state
        current_state = self._get_current_state()
        
        # Update executed trajectory
        self._executed_trajectory.append(current_state)
        
        # Check progress
        progress = self._calculate_progress(current_state)
        
        # Check for deviation from expected trajectory
        deviation = self._calculate_deviation(current_state)
        
        # Detect anomalies
        anomalies = self._detect_anomalies(current_state, progress, deviation)
        
        # Publish status
        status = ExecutionStatus(
            plan_id=self._active_plan.plan_id,
            progress_percent=progress,
            deviation=deviation,
            estimated_completion=self._estimate_completion(progress),
            anomalies=[a.to_msg() for a in anomalies],
            timestamp=self.get_clock().now()
        )
        self._status_pub.publish(status)
        
        # Handle anomalies
        for anomaly in anomalies:
            self._handle_anomaly(anomaly)
    
    def _detect_anomalies(self, 
                         current_state: RobotState,
                         progress: float,
                         deviation: float) -> List[Anomaly]:
        """
        Detect anomalies during execution.
        
        Anomaly types:
        1. STUCK: No progress for extended period
        2. DEVIATION: Significant deviation from planned trajectory
        3. OBSTACLE: Unexpected obstacle detected
        4. TIMEOUT: Execution taking longer than expected
        5. SENSOR_FAILURE: Critical sensor data missing
        6. COLLISION_RISK: Imminent collision detected
        """
        anomalies = []
        
        # Check for stuck condition
        if self._is_stuck():
            anomalies.append(Anomaly(
                type=AnomalyType.STUCK,
                severity=Severity.HIGH,
                description='Robot has not made progress for {} seconds'.format(
                    self._get_stuck_duration()
                ),
                suggested_recovery='REPLAN'
            ))
        
        # Check for deviation
        if deviation > self.get_parameter('position_tolerance').value:
            anomalies.append(Anomaly(
                type=AnomalyType.DEVIATION,
                severity=Severity.MEDIUM,
                description=f'Deviation from planned trajectory: {deviation:.2f}m',
                suggested_recovery='REPLAN' if deviation > 0.5 else 'CONTINUE'
            ))
        
        # Check for unexpected obstacles
        if self._detect_unexpected_obstacle():
            anomalies.append(Anomaly(
                type=AnomalyType.OBSTACLE,
                severity=Severity.HIGH,
                description='Unexpected obstacle detected on planned path',
                suggested_recovery='REPLAN'
            ))
        
        # Check for timeout
        if self._is_timeout():
            anomalies.append(Anomaly(
                type=AnomalyType.TIMEOUT,
                severity=Severity.MEDIUM,
                description='Execution exceeded expected duration',
                suggested_recovery='ESCALATE'
            ))
        
        return anomalies
    
    def _handle_anomaly(self, anomaly: Anomaly):
        """
        Handle detected anomalies with appropriate recovery strategies.
        """
        # Publish anomaly report
        self._anomaly_pub.publish(anomaly.to_msg())
        
        # Execute recovery strategy
        if anomaly.suggested_recovery == 'REPLAN':
            self._trigger_replanning()
        elif anomaly.suggested_recovery == 'RETRY':
            self._trigger_retry()
        elif anomaly.suggested_recovery == 'ESCALATE':
            self._escalate_to_human()
        elif anomaly.suggested_recovery == 'EMERGENCY_STOP':
            self._trigger_emergency_stop()
    
    def _trigger_replanning(self):
        """
        Trigger replanning from current position to goal.
        """
        self.get_logger().info('Triggering replanning')
        
        # Create new planning request from current position
        replan_request = GenerateMotionPlan.Goal(
            intent_msg=self._active_plan.original_intent,
            constraints=self._active_plan.constraints,
            start_state=self._get_current_state()
        )
        
        # Call motion planner
        # ... (action client call)
    
    def _calculate_progress(self, current_state: RobotState) -> float:
        """
        Calculate execution progress as percentage.
        
        For navigation: distance traveled / total distance
        For manipulation: waypoints completed / total waypoints
        """
        if self._active_plan.plan_type == PlanType.NAVIGATION:
            total_distance = self._calculate_path_length(self._expected_trajectory)
            remaining_distance = self._calculate_distance_to_goal(current_state)
            return 100 * (1 - remaining_distance / total_distance)
        
        elif self._active_plan.plan_type == PlanType.MANIPULATION:
            waypoints = self._active_plan.waypoints
            completed = self._count_completed_waypoints(current_state, waypoints)
            return 100 * completed / len(waypoints)
        
        return 0.0
```

### 5.2 Recovery Strategies

```python
class RecoveryStrategyLibrary:
    """
    Library of recovery strategies for different anomaly types.
    """
    
    def __init__(self):
        self._strategies = {
            AnomalyType.STUCK: [
                ClearCostmapRecovery(),
                RotateRecovery(),
                BackUpRecovery(),
                EscalateRecovery()
            ],
            AnomalyType.DEVIATION: [
                ReplanRecovery(),
                ReturnToPathRecovery(),
                ContinueRecovery()
            ],
            AnomalyType.OBSTACLE: [
                WaitRecovery(),
                ReplanRecovery(),
                AskHumanRecovery()
            ],
            AnomalyType.TIMEOUT: [
                ContinueRecovery(),
                EscalateRecovery()
            ],
            AnomalyType.SENSOR_FAILURE: [
                RetryRecovery(),
                FallbackSensorRecovery(),
                EscalateRecovery()
            ],
            AnomalyType.COLLISION_RISK: [
                EmergencyStopRecovery()
            ]
        }
    
    def get_strategy(self, 
                    anomaly: Anomaly,
                    context: ExecutionContext) -> RecoveryStrategy:
        """
        Select appropriate recovery strategy based on anomaly and context.
        """
        strategies = self._strategies.get(anomaly.type, [EscalateRecovery()])
        
        # Select first applicable strategy
        for strategy in strategies:
            if strategy.is_applicable(context):
                return strategy
        
        # Default to escalation
        return EscalateRecovery()
```

---

## 6. Integration with Nav2 and MoveIt2

### 6.1 Nav2 Integration (Navigation)

```python
class Nav2Integration:
    """
    Integration with Nav2 navigation stack.
    
    Maps motion primitives to Nav2 actions and monitors execution.
    """
    
    def __init__(self, node: Node):
        self._node = node
        
        # Nav2 action clients
        self._navigate_to_pose_client = ActionClient(
            node, NavigateToPose, '/navigate_to_pose'
        )
        self._follow_path_client = ActionClient(
            node, FollowPath, '/follow_path'
        )
        
        # Nav2 service clients
        self._clear_costmap_client = node.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        
    async def execute_navigate_to_pose(self, 
                                       plan: MotionPlan) -> ExecutionResult:
        """
        Execute a navigation plan using Nav2.
        
        Steps:
        1. Convert motion plan to Nav2 NavigateToPose goal
        2. Send goal to Nav2
        3. Monitor execution via feedback
        4. Return result
        """
        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = plan.target_pose
        goal_msg.behavior_tree = self._select_behavior_tree(plan.constraints)
        
        # Wait for server
        if not await self._navigate_to_pose_client.wait_for_server():
            return ExecutionResult(
                success=False,
                error='Nav2 server not available'
            )
        
        # Send goal
        goal_handle = await self._navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav2_feedback_callback
        )
        
        if not goal_handle.accepted:
            return ExecutionResult(
                success=False,
                error='Goal rejected by Nav2'
            )
        
        # Wait for result
        result = await goal_handle.get_result_async()
        
        return ExecutionResult(
            success=result.result.success,
            error=result.result.error_msg if not result.result.success else None
        )
    
    def _select_behavior_tree(self, constraints: PlanningConstraints) -> str:
        """
        Select appropriate Nav2 behavior tree based on constraints.
        
        Available BTs:
        - navigate_to_pose_w_replanning_and_recovery.xml (default)
        - navigate_to_pose_w_replanning_and_recovery_safety.xml (with extra safety checks)
        - follow_path.xml (for path following)
        """
        if constraints.safety_level == SafetyLevel.HIGH:
            return 'navigate_to_pose_w_replanning_and_recovery_safety.xml'
        return 'navigate_to_pose_w_replanning_and_recovery.xml'
    
    def _nav2_feedback_callback(self, feedback_msg):
        """
        Handle Nav2 feedback for monitoring.
        """
        feedback = feedback_msg.feedback
        # Publish to execution monitor
        # ...


class MoveIt2Integration:
    """
    Integration with MoveIt2 manipulation stack.
    
    Maps motion primitives to MoveIt2 actions and monitors execution.
    """
    
    def __init__(self, node: Node):
        self._node = node
        
        # MoveIt2 action clients
        self._move_group_client = ActionClient(
            node, MoveGroup, '/move_action'
        )
        self._execute_trajectory_client = ActionClient(
            node, ExecuteTrajectory, '/execute_trajectory'
        )
        
        # MoveIt2 service clients
        self._plan_kinematic_path_client = node.create_client(
            GetMotionPlan, '/plan_kinematic_path'
        )
        self._get_planning_scene_client = node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        
    async def execute_manipulation(self,
                                   plan: MotionPlan) -> ExecutionResult:
        """
        Execute a manipulation plan using MoveIt2.
        
        Steps:
        1. Update planning scene with current world state
        2. Plan motion using MoveIt2 planner
        3. Verify plan with safety validator
        4. Execute trajectory
        5. Monitor execution
        """
        # Get current planning scene
        scene = await self._get_planning_scene()
        
        # Create motion plan request
        plan_request = MotionPlanRequest()
        plan_request.workspace_parameters.header.frame_id = 'base_link'
        plan_request.goal_constraints = self._convert_to_moveit_constraints(plan)
        plan_request.group_name = plan.manipulation_group
        plan_request.planner_id = 'RRTConnect'
        plan_request.allowed_planning_time = 5.0
        
        # Call MoveIt2 planner
        plan_response = await self._plan_kinematic_path_client.call_async(
            GetMotionPlan.Request(motion_plan_request=plan_request)
        )
        
        if plan_response.motion_plan_response.error_code.val != 1:
            return ExecutionResult(
                success=False,
                error=f'MoveIt2 planning failed: {plan_response.motion_plan_response.error_code}'
            )
        
        # Execute trajectory
        trajectory = plan_response.motion_plan_response.trajectory
        
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory
        
        goal_handle = await self._execute_trajectory_client.send_goal_async(
            execute_goal,
            feedback_callback=self._moveit_feedback_callback
        )
        
        result = await goal_handle.get_result_async()
        
        return ExecutionResult(
            success=result.result.error_code.val == 1,
            error=str(result.result.error_code) if result.result.error_code.val != 1 else None
        )


# 6.3 Unified Planning Interface

class UnifiedPlanningInterface:
    """
    Unified interface for both navigation and manipulation planning.
    
    This class provides a single interface that automatically selects
    the appropriate planner (Nav2 or MoveIt2) based on the intent type.
    """
    
    def __init__(self, node: Node):
        self._node = node
        self._nav2 = Nav2Integration(node)
        self._moveit2 = MoveIt2Integration(node)
        
    async def execute_plan(self, plan: MotionPlan) -> ExecutionResult:
        """
        Execute a motion plan using the appropriate execution backend.
        
        Args:
            plan: The verified motion plan to execute
            
        Returns:
            ExecutionResult with success status and metrics
        """
        if plan.plan_type == PlanType.NAVIGATION:
            return await self._nav2.execute_navigate_to_pose(plan)
        
        elif plan.plan_type == PlanType.MANIPULATION:
            return await self._moveit2.execute_manipulation(plan)
        
        elif plan.plan_type == PlanType.COMPOSITE:
            # Execute composite plans sequentially
            for sub_plan in plan.sub_plans:
                result = await self.execute_plan(sub_plan)
                if not result.success:
                    return result
            return ExecutionResult(success=True)
        
        return ExecutionResult(
            success=False,
            error=f'Unknown plan type: {plan.plan_type}'
        )
