# Entity message for agent_ros_bridge
# Represents an extracted entity from user utterance

# Entity type (LOCATION, OBJECT, ACTION, NUMBER, etc.)
string type

# Entity value
string value

# Confidence score (0.0 to 1.0)
float32 confidence

# Start position in original utterance
int32 start_pos

# End position in original utterance
int32 end_pos
