# ExecutionStatus.msg
# Real-time execution status for monitoring

string plan_id                    # Reference to motion plan
string state                      # "PENDING", "EXECUTING", "PAUSED", "COMPLETED", "FAILED"
float64 progress_percent          # 0.0 to 100.0

# Current execution details
string current_primitive          # ID of currently executing primitive
int32 current_primitive_index     # Index in primitives array
time start_time                   # When execution started
time estimated_completion         # Predicted completion time

# Anomaly tracking
Anomaly[] detected_anomalies
int32 recovery_attempts           # Number of recovery attempts made

# Performance metrics
float64 actual_duration           # Time elapsed so far
float64 deviation_from_plan       # Position deviation in meters
