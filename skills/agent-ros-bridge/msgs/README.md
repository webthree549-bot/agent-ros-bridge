# Agent ROS Bridge Safety Messages

This package contains ROS2 message and service definitions for the Agent ROS Bridge safety system.

## Messages

### SafetyCertificate.msg
A cryptographic certificate validating that a trajectory has passed safety checks.
- `validation_id`: Unique UUID for this validation
- `issued_at`: When the certificate was issued
- `expires_at`: When the certificate expires (30s validity window)
- `plan_hash`: SHA-256 hash of the validated trajectory
- `constraints_checked`: Boolean array of verified constraints
- `constraint_names`: Names of constraints checked

### SafetyLimits.msg
Defines operational safety limits for a robot.
- `max_linear_velocity`: Maximum linear velocity (m/s)
- `max_angular_velocity`: Maximum angular velocity (rad/s)
- `max_joint_velocity`: Maximum joint velocity for manipulators (rad/s)
- `max_force`: Maximum force/torque (N or N·m)
- `workspace_bounds`: Polygon defining allowed workspace
- `restricted_zones`: Names of zones to avoid

### EmergencyStop.msg
Emergency stop status message.
- `triggered`: True if e-stop is active
- `source`: Source of e-stop (HUMAN, WATCHDOG, VALIDATOR, AI)
- `trigger_time`: When the e-stop was triggered
- `reason`: Human-readable reason

### TrajectoryValidationRequest.msg
Request to validate a trajectory.
- `trajectory`: The path to validate
- `current_state`: Current robot state
- `limits`: Safety limits to enforce

### TrajectoryValidationResponse.msg
Response from trajectory validation.
- `approved`: True if trajectory passes all checks
- `certificate`: Safety certificate (if approved)
- `rejection_reason`: Reason for rejection (if not approved)
- `validation_time_ms`: Time taken to validate

### RobotState.msg
Current robot state for safety validation.
- `pose`: Current position and orientation
- `twist`: Current velocity
- `joint_positions`: Joint positions (for manipulators)
- `joint_velocities`: Joint velocities (for manipulators)
- `timestamp`: When state was recorded

## Services

### ValidateTrajectory.srv
Validates a trajectory for safety compliance.

### GetSafetyLimits.srv
Retrieves safety limits for a specific robot.

### TriggerEmergencyStop.srv
Triggers an emergency stop.

### ClearEmergencyStop.srv
Clears an active emergency stop (requires authorization).

### GetSafetyStatus.srv
Gets current safety system status.

## Usage

```python
from agent_ros_bridge_safety_msgs.srv import ValidateTrajectory
from agent_ros_bridge_safety_msgs.msg import TrajectoryValidationRequest

# Create validation request
request = TrajectoryValidationRequest()
request.trajectory = my_trajectory
request.current_state = robot_state
request.limits = safety_limits

# Call validation service
response = await validation_client.call(request)
if response.approved:
    execute_trajectory(response.certificate)
else:
    print(f"Rejected: {response.rejection_reason}")
```

## Version

Compatible with Agent ROS Bridge v0.6.1
