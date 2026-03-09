# RobotState.msg
# Complete robot state for planning and monitoring

string robot_id                    # Unique robot identifier
geometry_msgs/PoseStamped pose     # Current robot pose
sensor_msgs/JointState joint_state # Arm joint states (if manipulator)
geometry_msgs/Twist velocity       # Current velocity

# Motion state
bool is_moving                     # Whether robot is currently moving
string current_action              # Current action being executed

# Localization quality
float64 localization_confidence    # 0.0 to 1.0

# Safety status
bool emergency_stop_active         # E-stop status
bool safety_zone_violated          # Whether in safety violation

# Battery/Power
float64 battery_level              # 0.0 to 1.0
bool low_battery                   # Below threshold warning
