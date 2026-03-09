# MotionPlan.msg
# Complete motion plan with safety certificate

string plan_id                    # Unique plan identifier (UUID)
MotionPrimitive[] primitives      # Sequence of motion primitives
nav_msgs/Path[] trajectories      # One trajectory per primitive
float64 total_duration            # Estimated total execution time

# Safety certificate (required for execution)
SafetyCertificate safety_certificate

# Validation flags
bool requires_validation          # Whether plan needs safety validation
bool is_validated                 # Whether plan has been validated

# Metadata
string planner_version            # Version of motion planner
time created_at                   # Plan creation timestamp
string intent_source              # Original intent that generated plan
