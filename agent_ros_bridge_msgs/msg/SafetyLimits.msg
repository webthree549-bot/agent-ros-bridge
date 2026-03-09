# Safety limits for motion planning
float64 max_linear_velocity      # m/s
float64 max_angular_velocity     # rad/s
float64 max_linear_acceleration  # m/s²
float64 max_angular_acceleration # rad/s²
float64 max_joint_velocity       # rad/s
float64 max_joint_acceleration   # rad/s²

# Workspace bounds
geometry_msgs/Vector3 workspace_min
geometry_msgs/Vector3 workspace_max

# Force/torque limits
float64 max_force
float64 max_torque
