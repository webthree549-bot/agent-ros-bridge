# TrajectoryValidationRequest.msg
# Request to validate a trajectory for safety compliance

nav_msgs/Path trajectory       # The trajectory to validate
RobotState current_state       # Current robot state (position, velocity, etc.)
SafetyLimits limits            # Safety limits to enforce during validation
