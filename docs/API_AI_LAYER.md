# v0.6.1 AI Layer API Documentation

**Version:** 0.6.1  
**Date:** March 9, 2026  
**Status:** Week 3 - Integration Complete

---

## Overview

The v0.6.1 AI Layer provides natural language understanding and motion planning capabilities for Agent ROS Bridge. It consists of four main components:

1. **Intent Parser** - Natural language to structured intent
2. **Context Manager** - Spatial and contextual reference resolution
3. **Motion Planner** - Motion planning with safety validation
4. **Safety Validator** - Trajectory validation and safety certification

---

## Services

### /ai/parse_intent

Parse natural language utterances into structured intents.

**Type:** ROS2 Service (`ParseIntent`)

**Request:**
```yaml
string utterance          # Natural language input
string robot_id           # Target robot identifier
string session_id         # Optional session context
```

**Response:**
```yaml
Intent intent             # Parsed intent
bool success              # Whether parsing succeeded
float64 latency_ms        # Processing time in milliseconds
string error_message      # Error details if failed
```

**Intent Message:**
```yaml
string type               # Intent type (NAVIGATE, MANIPULATE, SENSE, QUERY, SAFETY, CONFIGURE, UNKNOWN)
float64 confidence        # Confidence score (0.0-1.0)
Entity[] entities         # Extracted entities
string raw_utterance      # Original input
string source             # Parsing source (RULE_BASED, LLM_ASSISTED, UNKNOWN)
float64 latency_ms        # Processing time
builtin_interfaces/Time timestamp  # Timestamp
string robot_id           # Target robot
```

**Entity Message:**
```yaml
string type               # Entity type (LOCATION, OBJECT, SPEED, MODE, etc.)
string value              # Entity value
float64 confidence        # Entity extraction confidence
string resolved_from      # Original text that resolved to this entity
```

**Example:**
```bash
# Request
rosservice call /ai/parse_intent "utterance: 'go to kitchen' robot_id: 'robot_01'"

# Response
intent:
  type: "NAVIGATE"
  confidence: 0.95
  entities:
    - type: "LOCATION"
      value: "kitchen"
      confidence: 0.95
  raw_utterance: "go to kitchen"
  source: "RULE_BASED"
  latency_ms: 2.34
success: true
latency_ms: 2.34
```

**Performance:**
- Target: <10ms (p95)
- Rule-based: ~2-5ms
- LLM fallback: ~50-100ms (not yet implemented)

---

### /safety/validate_trajectory

Validate trajectories against safety constraints.

**Type:** ROS2 Service (`ValidateTrajectory`)

**Request:**
```yaml
Trajectory trajectory     # Trajectory to validate
SafetyLimits limits       # Safety limits to check against
```

**Response:**
```yaml
bool approved             # Whether trajectory is approved
SafetyCertificate certificate  # Safety certificate (if approved)
float64 validation_time_ms     # Validation time
string rejection_reason   # Reason for rejection (if not approved)
```

**SafetyCertificate Message:**
```yaml
string certificate_id     # Unique certificate ID
string trajectory_hash    # Hash of validated trajectory
float64 issued_at         # Issue timestamp
float64 expires_at        # Expiration timestamp
string[] constraints_checked  # List of checked constraints
```

**Example:**
```bash
# Request
rosservice call /safety/validate_trajectory "trajectory: {...} limits: {...}"

# Response (approved)
approved: true
certificate:
  certificate_id: "cert_12345"
  trajectory_hash: "abc123..."
  issued_at: 1710000000.0
  expires_at: 1710000030.0
  constraints_checked: ["velocity", "workspace", "joint_limits"]
validation_time_ms: 3.45
```

**Performance:**
- Target: <10ms (p95)
- Typical: ~3-5ms

---

### /safety/get_limits

Get current safety limits.

**Type:** ROS2 Service (`GetSafetyLimits`)

**Request:** (empty)

**Response:**
```yaml
SafetyLimits limits       # Current safety limits
```

**SafetyLimits Message:**
```yaml
float64 max_velocity      # Maximum velocity (m/s)
float64 max_acceleration  # Maximum acceleration (m/s²)
float64 max_force         # Maximum force (N)
float64 workspace_x_min   # Workspace boundaries
float64 workspace_x_max
float64 workspace_y_min
float64 workspace_y_max
float64 workspace_z_min
float64 workspace_z_max
```

---

### /safety/get_status

Get safety validator status and statistics.

**Type:** ROS2 Service (`GetSafetyStatus`)

**Request:** (empty)

**Response:**
```yaml
int64 validation_count    # Total validations performed
int64 rejection_count     # Total rejections
float64 average_validation_time_ms  # Average validation time
float64 uptime_seconds    # Node uptime
```

---

## Actions

### /ai/plan_motion

Plan motion from current state to goal.

**Type:** ROS2 Action (`PlanMotion`)

**Goal:**
```yaml
string intent_type        # Type of motion (NAVIGATE, MANIPULATE, etc.)
PoseStamped target_pose   # Target pose
string target_location    # Named location (alternative to pose)
string object_name        # Object to manipulate (for MANIPULATE)
Context context           # Context (robot pose, environment)
```

**Feedback:**
```yaml
string status             # Current status (planning, validating, complete)
float64 progress          # Progress (0.0-1.0)
string current_step       # Current planning step
```

**Result:**
```yaml
bool success              # Whether planning succeeded
MotionPlan plan           # Generated motion plan
float64 planning_time_ms  # Planning time
string error_message      # Error details if failed
```

**MotionPlan Message:**
```yaml
string[] primitives       # List of motion primitives
float64 expected_duration # Expected execution time (seconds)
float64 planning_time     # Time spent planning
SafetyCertificate safety_certificate  # Safety certificate
```

**Performance:**
- Target: <100ms (p95)
- Typical: ~50-80ms

---

### /ai/execute_motion

Execute a planned motion.

**Type:** ROS2 Action (`ExecuteMotion`)

**Goal:**
```yaml
MotionPlan plan           # Motion plan to execute
float64 start_time        # Execution start time
```

**Feedback:**
```yaml
float64 progress          # Execution progress (0.0-1.0)
string current_primitive  # Currently executing primitive
string status             # Execution status
```

**Result:**
```yaml
bool success              # Whether execution succeeded
float64 execution_time_ms # Execution time
string error_message      # Error details if failed
```

---

## Intent Types

| Type | Description | Example Utterances |
|------|-------------|-------------------|
| NAVIGATE | Move to a location | "go to kitchen", "navigate to the living room" |
| MANIPULATE | Pick/place objects | "pick up the cup", "place the book on the table" |
| SENSE | Perception tasks | "what do you see", "scan the area" |
| QUERY | Status queries | "where are you", "what is your status" |
| SAFETY | Emergency commands | "stop", "emergency stop", "halt" |
| CONFIGURE | System configuration | "set speed to slow", "enable autonomous mode" |
| UNKNOWN | Unrecognized intent | - |

---

## Performance Targets

| Component | Target | p95 Latency | Status |
|-----------|--------|-------------|--------|
| Intent Parser | <10ms | ~5ms | ✅ Met |
| Safety Validator | <10ms | ~4ms | ✅ Met |
| Motion Planner | <100ms | ~70ms | ✅ Met |
| Context Manager | <5ms | ~3ms | ✅ Met |

---

## Error Handling

### Intent Parser Errors

| Error | Cause | Resolution |
|-------|-------|------------|
| Empty utterance | No input provided | Check input before sending |
| UNKNOWN intent | No pattern matched | Use clearer utterance or LLM fallback |
| Low confidence | Ambiguous input | Request clarification |

### Safety Validator Errors

| Error | Cause | Resolution |
|-------|-------|------------|
| Velocity limit exceeded | Trajectory too fast | Reduce velocity |
| Workspace violation | Trajectory out of bounds | Adjust goal position |
| Joint limit exceeded | Joint angles invalid | Check joint constraints |
| Force limit exceeded | Excessive force predicted | Reduce speed/acceleration |

### Motion Planner Errors

| Error | Cause | Resolution |
|-------|-------|------------|
| Planning failed | No valid path found | Adjust goal or check obstacles |
| Safety validation failed | Unsafe trajectory | Modify constraints |
| Invalid goal | Goal pose unreachable | Check workspace limits |

---

## Integration Examples

### Example 1: Navigate to Location

```python
import rclpy
from agent_ros_bridge_msgs.srv import ParseIntent
from agent_ros_bridge_msgs.action import PlanMotion
from rclpy.action import ActionClient

# Initialize
rclpy.init()
node = rclpy.create_node('example')

# Parse intent
parse_client = node.create_client(ParseIntent, '/ai/parse_intent')
request = ParseIntent.Request(utterance='go to kitchen', robot_id='robot_01')
future = parse_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
intent = future.result().intent

# Plan motion
plan_client = ActionClient(node, PlanMotion, '/ai/plan_motion')
plan_client.wait_for_server()
goal = PlanMotion.Goal(
    intent_type=intent.type,
    target_location='kitchen'
)
future = plan_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
result = future.result().get_result()

print(f"Plan generated: {result.plan}")
```

### Example 2: Safety Validation

```python
from agent_ros_bridge_msgs.srv import ValidateTrajectory
from agent_ros_bridge_msgs.msg import Trajectory, SafetyLimits

# Create trajectory
trajectory = Trajectory()
trajectory.waypoints = [...]

# Set limits
limits = SafetyLimits(
    max_velocity=1.0,
    max_acceleration=2.0,
    workspace_x_min=-5.0,
    workspace_x_max=5.0,
    workspace_y_min=-5.0,
    workspace_y_max=5.0,
    workspace_z_min=0.0,
    workspace_z_max=2.0
)

# Validate
client = node.create_client(ValidateTrajectory, '/safety/validate_trajectory')
request = ValidateTrajectory.Request(trajectory=trajectory, limits=limits)
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()

if result.approved:
    print(f"Safe to execute! Certificate: {result.certificate.certificate_id}")
else:
    print(f"Unsafe: {result.rejection_reason}")
```

---

## Testing

### Unit Tests

```bash
# Run AI layer tests
pytest tests/unit/ai/ -v

# Run safety tests
pytest tests/unit/safety/ -v
```

### Integration Tests

```bash
# Run end-to-end pipeline tests
pytest tests/integration/test_ai_layer_integration.py -v
```

### Performance Benchmarks

```bash
# Run performance benchmarks
python scripts/benchmark_ai_layer.py --iterations 1000

# Save results to file
python scripts/benchmark_ai_layer.py --output results/benchmark_$(date +%s).json
```

---

## Troubleshooting

### High Latency

If intent parsing or safety validation is slow:

1. Check CPU usage: `htop` or `top`
2. Profile the code: `python -m cProfile scripts/benchmark_ai_layer.py`
3. Check for memory leaks: `valgrind --tool=massif`

### Low Confidence Scores

If intent parser returns low confidence:

1. Check utterance clarity
2. Verify pattern coverage in `intent_parser.py`
3. Consider enabling LLM fallback (future release)

### Safety Validation Failures

If trajectories are frequently rejected:

1. Check safety limits configuration
2. Verify trajectory generation
3. Review rejection reasons in logs

---

## Future Enhancements

### v0.6.2 (Planned)

- LLM fallback for complex utterances
- Context-aware intent parsing
- Multi-language support

### v0.7.0 (Planned)

- Learning from demonstration
- Adaptive motion planning
- Predictive safety validation

---

*Last updated: March 9, 2026*  
*For issues or questions, see the troubleshooting guide or file an issue on GitHub.*
