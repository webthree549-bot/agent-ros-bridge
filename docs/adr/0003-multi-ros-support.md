# ADR-0003: Support Both ROS1 and ROS2

## Status

**Accepted**

## Context

The ROS (Robot Operating System) ecosystem is split between two major versions:

- **ROS1** (Noetic, Melodic) — Mature, widely used, Python 2/3
- **ROS2** (Jazzy, Humble, Iron) — Next generation, real-time, DDS-based

Many organizations have significant ROS1 codebases but are transitioning to ROS2. Robot vendors support different versions. A bridge that only supported one version would exclude significant portions of the market.

Key challenges:
- ROS1 uses Python 2 (older distros) or 3 (Noetic)
- ROS2 uses Python 3 exclusively
- Different package structures (catkin vs colcon)
- Different message serialization
- Different node/topic discovery mechanisms

## Decision

We will support **both ROS1 and ROS2** through a unified connector interface with version-specific implementations.

### Architecture

```
┌─────────────────────────────────────┐
│        Connector Interface          │
│  connect() · publish() · subscribe()│
│  discover() · call_service()        │
└──────────────┬──────────────────────┘
               │
       ┌───────┴───────┐
       ▼               ▼
┌─────────────┐  ┌─────────────┐
│ ROS1 Connector│  │ ROS2 Connector│
│  (rospy)     │  │  (rclpy)     │
└─────────────┘  └─────────────┘
```

### Implementation Strategy

1. **Abstract Base Class** — `BaseROSConnector` defines common interface
2. **Dynamic Imports** — ROS libraries imported only when needed
3. **Message Registry** — Dynamic message type loading for both versions
4. **URI Scheme** — `ros1://` and `ros2://` distinguish connectors

### Code Structure

```python
# Abstract interface
class BaseROSConnector(ABC):
    @abstractmethod
    async def connect(self, uri: str) -> bool: ...
    
    @abstractmethod
    async def publish(self, topic: str, message: dict) -> bool: ...

# ROS1 implementation
class ROS1Connector(BaseROSConnector):
    def __init__(self):
        import rospy  # Dynamic import
        
    async def publish(self, topic, message):
        # rospy-specific implementation

# ROS2 implementation  
class ROS2Connector(BaseROSConnector):
    def __init__(self):
        import rclpy  # Dynamic import
        
    async def publish(self, topic, message):
        # rclpy-specific implementation
```

### Docker Strategy

Separate Docker images for each ROS version:
- `agent-ros-bridge:latest` — Base (no ROS)
- `agent-ros-bridge:ros1-noetic` — ROS1 Noetic
- `agent-ros-bridge:ros2-jazzy` — ROS2 Jazzy
- `agent-ros-bridge:ros2-humble` — ROS2 Humble

## Consequences

### Positive

- **Market Coverage** — Supports entire ROS ecosystem
- **Migration Path** — Helps users transition from ROS1 to ROS2
- **Flexibility** — Users can choose appropriate version
- **Future-Proof** — Positioned for ROS2 adoption growth

### Negative

- **Maintenance Burden** — Two codebases to maintain
- **Testing Complexity** — Must test with both ROS versions
- **Documentation** — More complex setup instructions
- **Docker Images** — Multiple images to build and maintain

### Neutral

- **Code Duplication** — ~60% code shared, extracted to base class
- **CI Complexity** — Separate test jobs for ROS1/ROS2

## Alternatives Considered

### ROS2 Only

**Rejected:** Would exclude large existing ROS1 user base. Many industrial robots still use ROS1.

### ROS1 Only

**Rejected:** ROS2 is the future. New projects choose ROS2. Would limit long-term viability.

### ros1_bridge Approach

**Rejected:** External ros1_bridge adds deployment complexity. Native support provides better integration.

## Implementation History

- **v0.3.x** — ROS2 support only
- **v0.5.0** — ROS1 connector added with full feature parity
- **v0.5.0** — Dynamic message types for both versions

## References

- [Multi-ROS Guide](../MULTI_ROS.md)
- [ROS1 Connector](../../agent_ros_bridge/gateway_v2/connectors/ros1_connector.py)
- [ROS2 Connector](../../agent_ros_bridge/gateway_v2/connectors/ros2_connector.py)
- Related: [ADR-0004: Dynamic Message Types](0004-dynamic-message-types.md)
