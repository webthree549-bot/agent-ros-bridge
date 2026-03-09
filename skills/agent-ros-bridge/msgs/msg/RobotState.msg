# RobotState.msg
# Current state of the robot for safety validation

geometry_msgs/Pose pose        # Current position and orientation
geometry_msgs/Twist twist      # Current velocity
float64[] joint_positions      # Joint positions for manipulators (optional)
float64[] joint_velocities     # Joint velocities for manipulators (optional)
builtin_interfaces/Time timestamp    # When the state was recorded
