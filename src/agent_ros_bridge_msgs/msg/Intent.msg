# Intent.msg
# Represents a parsed natural language intent with associated metadata

# Intent type classification
# NAVIGATE: Move to a location (e.g., "go to the kitchen")
# MANIPULATE: Interact with objects (e.g., "pick up the cup")
# SENSE: Gather sensor data (e.g., "scan the room")
# QUERY: Request information (e.g., "where are you?")
# CONFIGURE: Change settings (e.g., "set speed to slow")
# MISSION: Complex multi-step tasks (e.g., "deliver this package")
# SAFETY: Safety-related commands (e.g., "emergency stop")
# UNKNOWN: Could not determine intent
string type

# Confidence score for the intent classification (0.0 to 1.0)
# 0.0 = completely uncertain, 1.0 = completely certain
# Thresholds:
#   >= 0.95: High confidence, auto-execute
#   0.70-0.94: Medium confidence, may need confirmation
#   < 0.70: Low confidence, request clarification
float64 confidence

# Extracted entities from the utterance
# e.g., for "go to the kitchen slowly":
#   - LOCATION: "kitchen"
#   - SPEED: "slowly"
Entity[] entities

# Constraints applied to the intent
# e.g., "slowly" → SPEED_LIMIT constraint
#      "carefully" → SAFETY_MODE constraint
Constraint[] constraints

# Original raw utterance from the user
string raw_utterance

# Source of the parsing decision
# "RULE_BASED": Fast deterministic parser (< 5ms)
# "LLM_ASSISTED": LLM fallback parser (< 100ms timeout)
# "EMBEDDING": Semantic similarity matching
# "UNKNOWN": Could not parse
string source

# Latency of the parsing operation in milliseconds
# Target: < 10ms for rule-based, < 100ms for LLM fallback
float64 latency_ms

# Timestamp when the intent was parsed
builtin_interfaces/Time timestamp

# Optional: Robot ID this intent is targeting
string robot_id
