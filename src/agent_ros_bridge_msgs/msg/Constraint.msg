# Constraint.msg
# Represents a constraint on robot behavior extracted from natural language

# Constraint type
# SPEED_LIMIT: Maximum velocity (e.g., "slowly" → 0.3 m/s)
# TIMEOUT: Maximum execution time (e.g., "quickly" → 30s timeout)
# AVOID_ZONE: Area to avoid (e.g., "carefully" → avoid obstacles)
# FORCE_LIMIT: Maximum force/torque (e.g., "gently" → 5N max)
# SAFETY_MODE: Safety level (e.g., "carefully" → high safety)
# APPROACH_ANGLE: Preferred approach direction
# WAIT_CONDITION: Condition to wait for (e.g., "when ready")
# PRECISION: Accuracy requirement (e.g., "exactly" → ±1mm)
# COLLISION_AVOIDANCE: Collision avoidance level
# HUMAN_PROXIMITY: Minimum distance to humans
# NOISE_LEVEL: Maximum noise level
# ENERGY_LIMIT: Battery/energy constraint
string type

# Constraint value (format depends on type)
# SPEED_LIMIT: "0.5" (m/s)
# TIMEOUT: "30.0" (seconds)
# AVOID_ZONE: "obstacles" or "humans" or specific zone_id
# FORCE_LIMIT: "10.0" (Newtons)
# SAFETY_MODE: "low", "medium", "high", "critical"
# APPROACH_ANGLE: "front", "back", "any" or degrees
# WAIT_CONDITION: "human_approval", "sensor_trigger"
# PRECISION: "0.01" (meters)
# COLLISION_AVOIDANCE: "aggressive", "normal", "conservative"
# HUMAN_PROXIMITY: "1.0" (meters)
# NOISE_LEVEL: "quiet", "normal", "loud"
# ENERGY_LIMIT: "50" (percent battery)
string value

# Optional: Unit of the value (if not implied by type)
string unit

# Optional: Priority of this constraint (higher = more important)
# Default: 1
int32 priority 1

# Optional: Whether this constraint is hard (must be satisfied) or soft (preferred)
# true = hard constraint, execution fails if violated
# false = soft constraint, best effort to satisfy
bool is_hard false
