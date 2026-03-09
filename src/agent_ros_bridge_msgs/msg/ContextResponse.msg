# ContextResponse.msg
# Response to a context query with resolved reference

# Whether the reference was successfully resolved
bool found

# Type of the resolved entity
# "POSE": Resolved to a specific pose
# "LOCATION": Resolved to a named location
# "OBJECT": Resolved to an object ID
# "ROBOT": Resolved to a robot ID
# "PERSON": Resolved to a person ID
# "TIME": Resolved to a timestamp
# "ACTION": Resolved to a previous action
string resolved_type

# Resolved pose (if resolved_type is "POSE")
# Includes frame_id for coordinate frame reference
geometry_msgs/PoseStamped pose

# Resolved entity ID (if resolved to a named entity)
# e.g., location_id="kitchen_01", object_id="cup_123"
string entity_id

# Human-readable description of the resolution
# e.g., "kitchen at (5.2, 3.1, 0.0) in map frame"
string description

# Confidence in the resolution (0.0 to 1.0)
float64 confidence

# Alternative resolutions (if ambiguous)
# e.g., "it" could refer to multiple objects
string[] alternatives

# Timestamp when the resolution was performed
builtin_interfaces/Time timestamp
