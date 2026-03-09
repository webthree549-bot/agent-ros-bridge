# SafetyLimits.msg
# Defines the operational safety limits for a robot
# Used by the safety validator to check trajectory compliance

float64 max_linear_velocity    # Maximum allowed linear velocity (m/s)
float64 max_angular_velocity   # Maximum allowed angular velocity (rad/s)
float64 max_joint_velocity     # Maximum allowed joint velocity for manipulators (rad/s)
float64 max_force              # Maximum allowed force/torque (N or N·m)
geometry_msgs/Polygon workspace_bounds  # Polygon defining the allowed workspace boundary
string[] restricted_zones      # Names of restricted zones to avoid (references config)
