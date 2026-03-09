# Entity.msg
# Represents an extracted entity from natural language

# Entity type classification
# LOCATION: Places, rooms, positions (e.g., "kitchen", "door", "here")
# OBJECT: Physical objects (e.g., "cup", "box", "it")
# QUANTITY: Numeric values (e.g., "5 meters", "3 cups")
# SPEED: Velocity descriptors (e.g., "fast", "slowly", "0.5 m/s")
# DIRECTION: Spatial directions (e.g., "left", "forward", "north")
# TIME: Temporal references (e.g., "now", "in 5 minutes", "ASAP")
# ROBOT_ID: Robot identifiers (e.g., "turtlebot1", "all robots")
# POSE: Specific positions with orientation (e.g., "at (5, 3, 0)")
# PERSON: Human references (e.g., "me", "John", "the operator")
string type

# Extracted value of the entity
# Format depends on entity type:
#   LOCATION: "kitchen", "room_101"
#   OBJECT: "red_cup", "package"
#   QUANTITY: "5.0", "3"
#   SPEED: "0.5", "fast"
#   DIRECTION: "forward", "left"
#   TIME: ISO 8601 timestamp or relative duration
#   ROBOT_ID: "turtlebot_01"
#   POSE: "map:5.2,3.1,0.0,0.0,0.0,0.707,0.707"
string value

# Confidence score for entity extraction (0.0 to 1.0)
float64 confidence

# Original text that was resolved to this entity
# e.g., resolved_from="the red cup on the table" → value="red_cup"
string resolved_from

# Optional: Normalized/formatted value for machine use
# e.g., for "five meters": normalized_value="5.0"
string normalized_value

# Optional: Unit of measurement (if applicable)
# e.g., "meters", "seconds", "radians"
string unit
