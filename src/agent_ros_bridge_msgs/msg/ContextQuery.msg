# ContextQuery.msg
# Query for resolving contextual references in natural language

# Type of reference being queried
# "LOCATION": Spatial location (e.g., "kitchen", "there")
# "OBJECT": Physical object (e.g., "it", "the cup")
# "POSE": Specific pose/position (e.g., "here", "current position")
# "ROBOT": Robot reference (e.g., "you", "the robot")
# "PERSON": Human reference (e.g., "me", "the operator")
# "TIME": Temporal reference (e.g., "then", "that time")
# "ACTION": Previous action (e.g., "that", "what I asked before")
string reference_type

# The text that needs to be resolved
# e.g., "kitchen", "it", "there", "here"
string reference_text

# Robot ID for context-specific resolution
# Different robots may have different contexts
string robot_id

# Optional: Current timestamp for temporal resolution
builtin_interfaces/Time query_time

# Optional: Conversation/session ID for discourse context
string session_id

# Optional: Additional context hints
# e.g., for "it": hint="object" to prefer object resolution
string hint
