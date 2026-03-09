# Anomaly.msg
# Detected anomaly during execution

string type           # "STUCK", "DEVIATION", "OBSTACLE", "TIMEOUT", "SENSOR_FAILURE", "COLLISION_RISK"
string description    # Human-readable description
time detected_at      # When anomaly was detected
geometry_msgs/Pose location  # Where anomaly occurred (robot pose)
float64 severity      # 0.0 to 1.0 (higher = more severe)

# Recovery information
string suggested_recovery   # "REPLAN", "RETRY", "ESCALATE", "EMERGENCY_STOP", "CONTINUE"
bool auto_recoverable       # Whether system can auto-recover
