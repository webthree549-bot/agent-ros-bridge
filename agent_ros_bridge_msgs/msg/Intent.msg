# Intent message for agent_ros_bridge
# Represents a parsed user intent

# Intent type (NAVIGATE, MANIPULATE, SAFETY, PERCEIVE, CONFIGURE, etc.)
string type

# Confidence score (0.0 to 1.0)
float32 confidence

# Original utterance
string utterance

# Extracted entities
Entity[] entities

# Additional parameters as key-value pairs
string[] param_keys
string[] param_values
