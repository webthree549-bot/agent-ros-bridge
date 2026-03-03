# ADR-0004: Dynamic ROS Message Type System

## Status

**Accepted**

## Context

ROS uses strongly-typed messages defined in `.msg` files. Traditional ROS applications require:
1. Message definitions at compile time
2. Generated Python bindings
3. Recompilation when message types change

For a universal bridge that needs to:
- Support arbitrary robot message types
- Work with custom robot-specific messages
- Avoid recompilation for each new robot
- Provide runtime tool discovery

Static message typing was too restrictive.

## Decision

We will implement a **dynamic message type system** that loads and uses ROS message types at runtime without requiring pre-compiled bindings.

### Design

```python
# Message type registry
class MessageTypeRegistry:
    def __init__(self):
        self._types: Dict[str, Type] = {}
    
    def register(self, msg_type: str, python_class: Type):
        self._types[msg_type] = python_class
    
    def get(self, msg_type: str) -> Optional[Type]:
        return self._types.get(msg_type)
    
    def load_from_module(self, module_name: str):
        # Dynamically import ROS message module
        # Extract all message classes
        # Register them
```

### Built-in Message Types

Support 50+ common ROS message types out of the box:

```python
BUILTIN_TYPES = {
    # Geometry
    "geometry_msgs/Twist": geometry_msgs.msg.Twist,
    "geometry_msgs/Pose": geometry_msgs.msg.Pose,
    "geometry_msgs/Point": geometry_msgs.msg.Point,
    
    # Sensor
    "sensor_msgs/LaserScan": sensor_msgs.msg.LaserScan,
    "sensor_msgs/Image": sensor_msgs.msg.Image,
    "sensor_msgs/PointCloud2": sensor_msgs.msg.PointCloud2,
    
    # Navigation
    "nav_msgs/Odometry": nav_msgs.msg.Odometry,
    "nav_msgs/Path": nav_msgs.msg.Path,
    
    # Standard
    "std_msgs/String": std_msgs.msg.String,
    "std_msgs/Float64": std_msgs.msg.Float64,
    # ... etc
}
```

### Dynamic Loading

```python
async def publish_dynamic(self, topic: str, msg_type: str, data: dict):
    """Publish message with runtime type resolution."""
    # 1. Look up message class
    msg_class = self.registry.get(msg_type)
    if not msg_class:
        # 2. Try to auto-detect from topic
        msg_class = await self._detect_msg_type(topic)
    
    # 3. Create message instance
    msg = msg_class()
    
    # 4. Populate from dict
    self._dict_to_ros_msg(data, msg)
    
    # 5. Publish
    publisher.publish(msg)
```

### Message Conversion

```python
def ros_msg_to_dict(self, msg) -> dict:
    """Convert ROS message to dictionary."""
    result = {"_type": type(msg).__name__}
    
    for slot in msg.__slots__:
        value = getattr(msg, slot)
        if hasattr(value, '__slots__'):
            # Nested message
            result[slot] = self.ros_msg_to_dict(value)
        elif isinstance(value, list):
            # Array of messages
            result[slot] = [
                self.ros_msg_to_dict(v) if hasattr(v, '__slots__') else v
                for v in value
            ]
        else:
            result[slot] = value
    
    return result
```

## Consequences

### Positive

- **Flexibility** — Works with any ROS message type without recompilation
- **Runtime Discovery** — Can introspect topics and auto-detect types
- **Custom Messages** — Supports robot-specific message types
- **No Code Generation** — Simpler deployment, no build step
- **Universal** — Same code works across different robots

### Negative

- **Performance** — Runtime reflection slower than compiled bindings (~10-20%)
- **Type Safety** — Less compile-time checking, more runtime errors
- **Documentation** — Harder to document "any message type"
- **IDE Support** — Less autocomplete for dynamic types

### Neutral

- **Caching** — Message type lookups cached after first use
- **Validation** — Runtime validation of message structure

## Alternatives Considered

### Static Code Generation

**Rejected:** Would require build step for each robot configuration. Defeats "universal bridge" goal.

### ROS Bridge Suite Approach

**Rejected:** External rosbridge_suite adds latency and complexity. Native integration provides better performance.

### JSON-Only Messages

**Rejected:** Would lose type safety and ROS ecosystem compatibility. Native ROS messages required for many robots.

## Implementation History

- **v0.3.x** — Static message types only (std_msgs/String)
- **v0.5.0** — Dynamic message type system implemented
- **v0.5.0** — 50+ built-in message types
- **v0.5.0** — Auto-detection from topic introspection

## References

- [Message Registry Implementation](../../agent_ros_bridge/gateway_v2/connectors/ros2_connector.py)
- [Multi-ROS Guide](../MULTI_ROS.md)
- Related: [ADR-0003: Multi-ROS Support](0003-multi-ros-support.md)
