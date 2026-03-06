# NL2ROS: Natural Language to ROS Code Translation System

## Overview

This system translates natural language commands into production-ready, industrial-grade ROS code through a structured pipeline with validation, safety checks, and verification.

## Translation Pipeline

```
Natural Language
       ↓
[1] Intent Classification
       ↓
[2] Entity Extraction
       ↓
[3] Context Resolution
       ↓
[4] ROS Primitive Mapping
       ↓
[5] Code Generation
       ↓
[6] Safety Validation
       ↓
[7] Output Formatting
       ↓
Industrial ROS Code
```

---

## Stage 1: Intent Classification

### Intent Categories

| Intent | Description | Example |
|--------|-------------|---------|
| `NAVIGATE` | Move to location | "Go to the kitchen" |
| `MANIPULATE` | Arm/gripper control | "Pick up the box" |
| `SENSE` | Sensor operation | "Scan the area" |
| `CONFIGURE` | Parameter change | "Set speed to 0.5" |
| `QUERY` | State inquiry | "What is your battery level?" |
| `MISSION` | Multi-step task | "Patrol the perimeter" |
| `SAFETY` | Emergency action | "Stop immediately" |

### Classification Prompt

```
You are an intent classifier for ROS robot commands.

Input: "{natural_language_command}"

Classify into exactly one category:
- NAVIGATE: Movement, positioning, path following
- MANIPULATE: Arm motion, gripper, manipulation
- SENSE: Camera, lidar, sensor activation
- CONFIGURE: Parameter setting, mode changes
- QUERY: Status, telemetry requests
- MISSION: Multi-step, scheduled, or complex tasks
- SAFETY: Emergency stop, protective actions

Output format:
{{
  "intent": "CATEGORY",
  "confidence": 0.0-1.0,
  "sub_intent": "specific_action",
  "requires_confirmation": true/false
}}
```

---

## Stage 2: Entity Extraction

### Entity Types

| Entity | Pattern | Normalization |
|--------|---------|---------------|
| `LOCATION` | "kitchen", "point A", "home" | Resolve to coordinates/frame |
| `OBJECT` | "box", "red ball", "obstacle" | Object class + properties |
| `QUANTITY` | "2 meters", "90 degrees", "5 seconds" | SI units + tolerance |
| `SPEED` | "slowly", "fast", "0.5 m/s" | m/s with safety bounds |
| `DIRECTION` | "forward", "left", "clockwise" | Vector/quaternion |
| `TIME` | "in 5 minutes", "at 3pm" | ROS Time/Duration |
| `ROBOT_ID` | "turtlebot1", "arm_01" | Namespace + ID |

### Extraction Prompt

```
You are an entity extractor for ROS commands.

Input: "{natural_language_command}"
Known Context: {context_json}

Extract and normalize entities:

1. LOCATIONS: Map to known locations or relative positions
2. OBJECTS: Identify with properties (color, size, type)
3. QUANTITIES: Convert to SI units with tolerances
4. SPEEDS: Normalize to m/s or rad/s with safety limits
5. DIRECTIONS: Convert to rotation matrices or quaternions
6. TIMES: Parse to ROS Time or Duration messages
7. ROBOT_IDS: Resolve to full namespaces

Output format:
{{
  "entities": [
    {{
      "type": "LOCATION|OBJECT|QUANTITY|SPEED|DIRECTION|TIME|ROBOT_ID",
      "text": "original text",
      "value": normalized_value,
      "unit": "si_unit",
      "tolerance": ±value,
      "frame_id": "map|base_link|etc",
      "confidence": 0.0-1.0
    }}
  ],
  "missing_entities": ["required_but_not_found"],
  "ambiguous_entities": ["could_be_multiple_things"]
}}
```

---

## Stage 3: Context Resolution

### Context Sources

1. **Session Context** — Previous commands, current location
2. **Robot State** — Battery, pose, operational mode
3. **Environment** — Map, obstacles, known objects
4. **Safety Constraints** — Speed limits, boundaries, no-go zones
5. **Task History** — What was just attempted

### Resolution Prompt

```
You are a context resolver for ROS commands.

Input Command: "{natural_language_command}"
Extracted Entities: {entities_json}
Current Context: {context_json}

Resolve ambiguities and fill gaps:

1. PRONOUN RESOLUTION: "it", "there", "that" → specific entity
2. DEFAULT VALUES: Apply safe defaults for missing parameters
3. CONSTRAINT CHECKING: Validate against safety limits
4. FRAME TRANSFORMATION: Convert to appropriate TF frames
5. TEMPORAL RESOLUTION: "now", "then" → specific timestamps

Apply safety rules:
- Max linear speed: {max_linear_speed} m/s
- Max angular speed: {max_angular_speed} rad/s
- Workspace boundaries: {workspace_bounds}
- Required clearances: {clearance_distances}

Output format:
{{
  "resolved_command": "fully_specified_command",
  "parameters": {{
    "target_frame": "frame_id",
    "target_pose": {{"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}},
    "speed": {{"linear": 0.5, "angular": 1.0}},
    "tolerances": {{"position": 0.1, "orientation": 0.05}},
    "constraints": ["safety_constraint_1", "safety_constraint_2"]
  }},
  "required_transforms": ["source_frame->target_frame"],
  "safety_clearance": true/false,
  "estimated_duration": seconds,
  "resource_requirements": ["topic_1", "service_1", "action_1"]
}}
```

---

## Stage 4: ROS Primitive Mapping

### Primitive Library

| Primitive | ROS API | Use Case |
|-----------|---------|----------|
| `PUBLISH` | rospy.Publisher / rclpy.Publisher | Direct command injection |
| `SUBSCRIBE` | rospy.Subscriber / rclpy.Subscriber | State monitoring |
| `SERVICE_CALL` | rospy.ServiceProxy / rclpy.Client | Synchronous operations |
| `ACTION_CLIENT` | actionlib.SimpleActionClient / rclpy.ActionClient | Long-running tasks |
| `PARAM_SET` | rospy.set_param / rclpy.Node.set_parameters | Configuration |
| `PARAM_GET` | rospy.get_param / rclpy.Node.get_parameters | State queries |
| `TF_LOOKUP` | tf.TransformListener / tf2_ros.Buffer | Coordinate transforms |
| `DYNAMIC_RECONFIGURE` | dynamic_reconfigure.Client | Runtime tuning |

### Mapping Prompt

```
You are a ROS primitive mapper for industrial robotics.

Resolved Command: "{resolved_command}"
Intent: {intent}
Parameters: {parameters_json}
ROS Version: {ros_version} (ros1|ros2)

Map to ROS primitives following industrial best practices:

PRIMITIVE SELECTION CRITERIA:
- Use ACTIONS for: Navigation, manipulation, long-running tasks
- Use SERVICES for: Configuration, queries, immediate operations
- Use TOPICS for: Continuous control, telemetry, monitoring
- Use PARAMETERS for: Persistent configuration changes
- Use TF for: All coordinate transformations

CODE PATTERNS:
- Always use timeouts on blocking operations
- Implement proper error handling with retry logic
- Include preemption handling for actions
- Validate all inputs before execution
- Log all operations at appropriate levels

Select primitives:

Output format:
{{
  "primitives": [
    {{
      "type": "PUBLISH|SUBSCRIBE|SERVICE_CALL|ACTION_CLIENT|PARAM_SET|PARAM_GET|TF_LOOKUP",
      "topic|service|action|param": "/full/topic/name",
      "msg_type": "geometry_msgs/Twist|nav_msgs/Odometry|etc",
      "purpose": "what this primitive accomplishes",
      "execution_order": 1,
      "dependencies": ["primitives_that_must_complete_first"],
      "timeout_sec": 5.0,
      "retry_count": 3,
      "is_critical": true/false
    }}
  ],
  "execution_strategy": "sequential|parallel|state_machine",
  "fallback_behavior": "what_to_do_on_failure",
  "monitoring_requirements": ["topics_to_monitor"]
}}
```

---

## Stage 5: Code Generation

### Generation Templates

#### Template: Navigation (Nav2/Nav1)

```python
# AUTO-GENERATED ROS2 NAVIGATION CODE
# Generated from: "{original_command}"
# Timestamp: {timestamp}
# Safety Level: {safety_level}

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class GeneratedNavigation(Node):
    def __init__(self):
        super().__init__('generated_navigation')
        
        # Action client for navigation
        self._nav_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose'
        )
        
        # TF buffer for coordinate transforms
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, 
            self
        )
        
        # Safety parameters
        self.max_linear_speed = {max_linear_speed}  # m/s
        self.max_angular_speed = {max_angular_speed}  # rad/s
        self.position_tolerance = {position_tolerance}  # meters
        
    def navigate_to_pose(self, target_pose: PoseStamped) -> bool:
        '''Execute navigation to target pose with safety checks.'''
        
        # SAFETY CHECK: Validate target is in workspace
        if not self._is_in_workspace(target_pose):
            self.get_logger().error(
                f'Target pose outside workspace: {target_pose}'
            )
            return False
        
        # TRANSFORM: Convert to map frame if needed
        if target_pose.header.frame_id != 'map':
            try:
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    target_pose.header.frame_id,
                    rclpy.time.Time()
                )
                target_pose = do_transform_pose(target_pose, transform)
            except tf2_ros.TransformException as ex:
                self.get_logger().error(
                    f'Could not transform pose: {ex}'
                )
                return False
        
        # BUILD GOAL
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        # Configure behavior tree (optional)
        goal_msg.behavior_tree = '{behavior_tree_path}'
        
        # SEND GOAL with timeout
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        self.get_logger().info(
            f'Navigating to: x={target_pose.pose.position.x:.2f}, '
            f'y={target_pose.pose.position.y:.2f}'
        )
        
        # Send with feedback callback
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        # Wait for acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation completed successfully')
            return True
        else:
            self.get_logger().error(f'Navigation failed with status: {result.status}')
            return False
    
    def _feedback_callback(self, feedback_msg):
        '''Monitor navigation progress.'''
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )
    
    def _is_in_workspace(self, pose: PoseStamped) -> bool:
        '''Check if pose is within allowed workspace.'''
        # Workspace bounds from safety configuration
        bounds = {workspace_bounds}
        return (
            bounds['x_min'] <= pose.pose.position.x <= bounds['x_max'] and
            bounds['y_min'] <= pose.pose.position.y <= bounds['y_max']
        )

def main(args=None):
    rclpy.init(args=args)
    node = GeneratedNavigation()
    
    # Build target pose from resolved parameters
    target = PoseStamped()
    target.header.frame_id = '{target_frame}'
    target.header.stamp = node.get_clock().now().to_msg()
    target.pose.position.x = {target_x}
    target.pose.position.y = {target_y}
    target.pose.position.z = {target_z}
    target.pose.orientation.x = {target_qx}
    target.pose.orientation.y = {target_qy}
    target.pose.orientation.z = {target_qz}
    target.pose.orientation.w = {target_qw}
    
    try:
        success = node.navigate_to_pose(target)
        exit_code = 0 if success else 1
    except Exception as e:
        node.get_logger().error(f'Navigation exception: {e}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
```

#### Template: Manipulation (MoveIt2)

```python
# AUTO-GENERATED MOVEIT2 MANIPULATION CODE
# Generated from: "{original_command}"

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class GeneratedManipulation(Node):
    def __init__(self):
        super().__init__('generated_manipulation')
        
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # Planning parameters
        self.planning_time = 5.0  # seconds
        self.max_velocity_scaling = 0.5  # 50% of max
        self.max_acceleration_scaling = 0.5
        
    def plan_and_execute(self, target_pose: PoseStamped, 
                         planning_group: str = 'panda_arm') -> bool:
        '''Plan and execute motion to target pose.'''
        
        # Build motion plan request
        request = MotionPlanRequest()
        request.group_name = planning_group
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        
        # Goal constraints
        goal_constraints = Constraints()
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = 'panda_link8'
        position_constraint.target_point_offset.x = target_pose.pose.position.x
        position_constraint.target_point_offset.y = target_pose.pose.position.y
        position_constraint.target_point_offset.z = target_pose.pose.position.z
        
        # Add tolerance sphere
        tolerance_shape = SolidPrimitive()
        tolerance_shape.type = SolidPrimitive.SPHERE
        tolerance_shape.dimensions = [0.01]  # 1cm tolerance
        position_constraint.constraint_region.primitives.append(tolerance_shape)
        
        goal_constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = 'panda_link8'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        
        goal_constraints.orientation_constraints.append(orientation_constraint)
        request.goal_constraints.append(goal_constraints)
        
        # Planning options
        options = PlanningOptions()
        options.plan_only = False
        options.look_around = False
        options.replan = True
        options.replan_attempts = 5
        
        # Build goal
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        
        # Execute
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup server not available')
            return False
        
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Motion planning failed')
            return False
        
        # Wait for execution
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        success = result.result.error_code.val == 1  # SUCCESS
        if success:
            self.get_logger().info('Motion executed successfully')
        else:
            self.get_logger().error(f'Motion failed: {result.result.error_code.val}')
        
        return success
```

#### Template: Direct Control (Topic Publishing)

```python
# AUTO-GENERATED DIRECT CONTROL CODE
# Generated from: "{original_command}"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header
import time

class GeneratedDirectControl(Node):
    def __init__(self):
        super().__init__('generated_direct_control')
        
        # QoS profile for reliable command delivery
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos
        )
        
        # Safety limits
        self.max_linear = {max_linear_speed}
        self.max_angular = {max_angular_speed}
        
        # Command rate
        self._timer_period = 0.1  # 10 Hz
        self._active_command = None
        self._timer = self.create_timer(
            self._timer_period, 
            self._command_callback
        )
        
    def _command_callback(self):
        '''Publish active command at fixed rate.'''
        if self._active_command is not None:
            # Apply safety limits
            limited = self._apply_limits(self._active_command)
            self._cmd_vel_pub.publish(limited)
    
    def _apply_limits(self, cmd: Twist) -> Twist:
        '''Apply safety limits to command.'''
        limited = Twist()
        limited.linear.x = max(
            -self.max_linear,
            min(self.max_linear, cmd.linear.x)
        )
        limited.linear.y = max(
            -self.max_linear,
            min(self.max_linear, cmd.linear.y)
        )
        limited.angular.z = max(
            -self.max_angular,
            min(self.max_angular, cmd.angular.z)
        )
        return limited
    
    def move_for_duration(self, linear_x: float, angular_z: float, 
                          duration_sec: float):
        '''Execute motion for specified duration.'''
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        self._active_command = cmd
        self.get_logger().info(
            f'Moving: linear={linear_x:.2f}, angular={angular_z:.2f} '
            f'for {duration_sec:.1f}s'
        )
        
        time.sleep(duration_sec)
        
        # Stop
        self._active_command = Twist()
        self.get_logger().info('Motion complete')

def main(args=None):
    rclpy.init(args=args)
    node = GeneratedDirectControl()
    
    try:
        node.move_for_duration(
            linear_x={linear_velocity},
            angular_z={angular_velocity},
            duration_sec={duration}
        )
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure stop command is sent
        node._active_command = Twist()
        node._command_callback()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Stage 6: Safety Validation

### Validation Rules

| Rule | Check | Action on Failure |
|------|-------|-------------------|
| `SPEED_LIMIT` | Linear/angular within bounds | Clamp to max |
| `WORKSPACE_BOUNDARY` | Target in allowed area | Reject + alert |
| `COLLISION_CHECK` | Path collision-free | Replan or abort |
| `EMERGENCY_STOP` | E-stop not active | Block all motion |
| `BATTERY_LEVEL` | Sufficient for task | Warn + limit range |
| `JOINT_LIMITS` | Within mechanical limits | Clamp or reject |
| `FORCE_TORQUE` | Within sensor limits | Reduce speed |

### Validation Prompt

```
You are a safety validator for industrial ROS code.

Generated Code:
```python
{generated_code}
```

Safety Configuration:
{ safety_config_json}

Validate against industrial safety standards:

1. ISO 10218-1/2 (Robot safety)
2. ISO/TS 15066 (Collaborative robots)
3. IEC 61508 (Functional safety)
4. Company-specific safety rules

Perform static analysis:
- [ ] No hardcoded credentials or secrets
- [ ] All timeouts specified
- [ ] Error handling for all external calls
- [ ] Emergency stop integration present
- [ ] Velocity/acceleration limits enforced
- [ ] Workspace boundaries checked
- [ ] Resource cleanup in finally blocks

Perform dynamic checks (if possible):
- [ ] Simulate execution
- [ ] Check for infinite loops
- [ ] Verify resource limits

Output format:
{{
  "validation_passed": true/false,
  "critical_issues": [
    {{
      "rule": "RULE_NAME",
      "severity": "CRITICAL|HIGH|MEDIUM|LOW",
      "description": "issue description",
      "location": "line_number_or_function",
      "fix_suggestion": "how to fix"
    }}
  ],
  "warnings": [
    {{
      "rule": "RULE_NAME",
      "severity": "MEDIUM|LOW",
      "description": "warning description",
      "recommendation": "suggested improvement"
    }}
  ],
  "modified_code": "code_with_fixes_applied",
  "safety_certification": "PASS|CONDITIONAL|FAIL"
}}
```

---

## Stage 7: Output Formatting

### Final Output Structure

```json
{
  "translation_id": "uuid",
  "timestamp": "ISO8601",
  "original_command": "natural language input",
  "ros_version": "ros1|ros2",
  "intent": {
    "category": "NAVIGATE|MANIPULATE|SENSE|...",
    "confidence": 0.95,
    "sub_intent": "navigate_to_pose"
  },
  "entities": [
    {
      "type": "LOCATION",
      "value": "kitchen",
      "resolved": {"x": 5.2, "y": 3.1, "frame_id": "map"}
    }
  ],
  "generated_code": {
    "language": "python",
    "ros_version": "ros2",
    "code": "full source code",
    "filename": "generated_navigate_001.py",
    "line_count": 150
  },
  "safety_validation": {
    "status": "PASS",
    "critical_issues": 0,
    "warnings": 2
  },
  "execution_metadata": {
    "estimated_duration": 45.5,
    "required_topics": ["/cmd_vel", "/odom"],
    "required_services": [],
    "required_actions": ["/navigate_to_pose"],
    "estimated_resource_usage": {
      "cpu_percent": 15,
      "memory_mb": 128
    }
  },
  "fallback_behavior": {
    "on_timeout": "cancel_goal_and_stop",
    "on_failure": "retry_twice_then_abort",
    "on_preemption": "save_state_and_yield"
  },
  "human_readable_summary": "The robot will navigate to the kitchen at position (5.2, 3.1) in the map frame, maintaining a maximum speed of 0.5 m/s. The operation is expected to take approximately 45 seconds."
}
```

---

## Usage Examples

### Example 1: Simple Navigation

**Input:** "Go to the kitchen slowly"

**Pipeline Output:**
```json
{
  "intent": {"category": "NAVIGATE", "confidence": 0.98},
  "entities": [
    {"type": "LOCATION", "text": "kitchen", "resolved": {"x": 5.2, "y": 3.1}},
    {"type": "SPEED", "text": "slowly", "resolved": 0.3}
  ],
  "generated_code": "...NavigateToPose action client code...",
  "safety_validation": {"status": "PASS"},
  "human_readable_summary": "Navigate to kitchen at reduced speed (0.3 m/s)"
}
```

### Example 2: Manipulation Task

**Input:** "Pick up the red box from the table and place it on the shelf"

**Pipeline Output:**
```json
{
  "intent": {"category": "MANIPULATE", "confidence": 0.95},
  "entities": [
    {"type": "OBJECT", "text": "red box", "properties": {"color": "red", "type": "box"}},
    {"type": "LOCATION", "text": "table", "resolved": "table_location"},
    {"type": "LOCATION", "text": "shelf", "resolved": "shelf_location"}
  ],
  "generated_code": "...MoveIt2 pick-and-place sequence...",
  "safety_validation": {
    "status": "CONDITIONAL",
    "warnings": [
      {
        "rule": "VISION_VERIFICATION",
        "description": "Object detection not integrated - verify object position before execution"
      }
    ]
  }
}
```

### Example 3: Emergency Stop

**Input:** "Stop immediately!"

**Pipeline Output:**
```json
{
  "intent": {"category": "SAFETY", "confidence": 1.0, "sub_intent": "emergency_stop"},
  "entities": [{"type": "URGENCY", "text": "immediately", "priority": "CRITICAL"}],
  "generated_code": "...Direct e-stop publisher with highest priority...",
  "safety_validation": {"status": "PASS", "notes": "E-stop pattern verified"},
  "execution_metadata": {"estimated_duration": 0.1}
}
```

---

## Integration with Agent ROS Bridge

### API Endpoint

```python
# Agent ROS Bridge integration
from agent_ros_bridge.integrations.nl2ros import NL2Rostranslator

translator = NL2Rostranslator(
    ros_version='ros2',
    safety_config='config/safety.yaml',
    workspace_bounds='config/workspace.yaml'
)

result = translator.translate(
    command="Go to the kitchen slowly",
    context={
        'current_pose': robot.get_pose(),
        'known_locations': location_db,
        'safety_limits': safety_limits
    }
)

# Execute generated code
if result.safety_validation.status == 'PASS':
    execution_id = await bridge.execute_generated_code(
        result.generated_code,
        monitoring=True
    )
```

### WebSocket Protocol

```json
// Request
{
  "type": "nl2ros.translate",
  "command": "Navigate to charging station",
  "context": {...},
  "request_id": "uuid"
}

// Response
{
  "type": "nl2ros.result",
  "request_id": "uuid",
  "result": {...},
  "timestamp": "..."
}
```

---

## Quality Assurance

### Test Coverage Requirements

| Component | Coverage | Test Types |
|-----------|----------|------------|
| Intent Classifier | 95% | Unit, integration, adversarial |
| Entity Extractor | 90% | Unit, edge cases, ambiguity |
| Code Generator | 90% | Unit, compilation, execution |
| Safety Validator | 100% | All rules, boundary conditions |
| End-to-End | 80% | Real robot, simulation |

### Performance Targets

| Metric | Target | Maximum |
|--------|--------|---------|
| Translation latency | <100ms | 500ms |
| Code generation | <200ms | 1s |
| Safety validation | <100ms | 500ms |
| Total pipeline | <500ms | 2s |
| Concurrent translations | 100 | - |

---

## Future Enhancements

1. **Multi-language Support** — Spanish, Chinese, German NL input
2. **Learning from Feedback** — Improve based on execution success/failure
3. **Skill Composition** — Combine multiple primitives into reusable skills
4. **Natural Language Programming** — "Whenever X happens, do Y"
5. **Explanation Generation** — "I generated this code because..."

---

**Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Design Complete, Implementation Ready
