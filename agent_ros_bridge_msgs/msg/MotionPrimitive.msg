# MotionPrimitive.msg
# Pre-verified motion primitive for robot execution

# Primitive identification
string type           # "NAVIGATE", "MANIPULATE", "GRIPPER"
string primitive_id   # "navigate_to_pose", "pick_object", etc.
string version        # Semantic version of primitive

# Target specification
geometry_msgs/PoseStamped target_pose

# Parameters (key-value pairs for flexibility)
string[] parameter_names
string[] parameter_values

# Expected duration for planning
float64 expected_duration

# Safety bounds (enforced during execution)
float64 max_linear_velocity    # m/s
float64 max_angular_velocity   # rad/s
float64 min_obstacle_clearance # m

# Pre/post condition flags
bool requires_gripper_empty    # For pick operations
bool requires_gripper_occupied # For place operations
bool requires_localization     # For navigation
