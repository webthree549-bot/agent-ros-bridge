# Agent ROS Bridge - API Documentation

## Overview

Agent ROS Bridge provides a unified interface for AI agents to control ROS-based robots through multiple transport protocols.

## Core API

### Bridge Class

```python
from agent_ros_bridge import Bridge

# Create bridge instance
bridge = Bridge(ros_version=2)

# Register transports
bridge.transport_manager.register(
    WebSocketTransport({"port": 8765})
)

# Start bridge
await bridge.start()
```

### Transport Management

```python
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer

# WebSocket transport
ws_transport = WebSocketTransport({
    "host": "0.0.0.0",
    "port": 8765,
    "auth": {"enabled": True, "jwt_secret": "..."}
})

# gRPC transport
grpc_transport = GRPCServer({
    "host": "0.0.0.0",
    "port": 50051,
    "auth": {"enabled": True}
})
```

### Safety Validation

```python
from agent_ros_bridge.safety.validator import SafetyValidatorNode

validator = SafetyValidatorNode()

# Validate trajectory
trajectory = {
    "waypoints": [{"x": 0, "y": 0}, {"x": 1, "y": 1}],
    "velocities": [0.1, 0.2]
}
limits = {
    "max_linear_velocity": 1.0,
    "workspace_bounds": {"x_min": -10, "x_max": 10, "y_min": -10, "y_max": 10}
}

result = validator.validate_trajectory(trajectory, limits)
# Returns: {"valid": True, "certificate": {...}} or {"valid": False, "reason": "..."}
```

### Error Handling

```python
from agent_ros_bridge.utils.error_handling import (
    AgentError, ErrorCode, InputValidator, with_retry
)

# Validate input
result = InputValidator.validate_utterance("go to kitchen")
if not result.valid:
    raise AgentError(
        code=result.error_code,
        message=result.error_message
    )

# Retry decorator
@with_retry(max_retries=3, delay=1.0)
def call_external_service():
    # May fail temporarily
    pass
```

## Integration APIs

### LangChain

```python
# Get LangChain tool
tool = bridge.get_langchain_tool(["navigate", "move_arm"])

# Use with LangChain agent
from langchain.agents import initialize_agent
agent = initialize_agent([tool], llm, agent="zero-shot-react-description")
```

### MCP (Claude Desktop)

```python
# Get MCP server
mcp_server = bridge.get_mcp_server()

# Use with Claude Desktop
# Configure in Claude Desktop settings:
# {
#   "mcpServers": {
#     "ros": {
#       "command": "python",
#       "args": ["-m", "agent_ros_bridge.mcp"]
#     }
#   }
# }
```

## ROS2 Interface

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Publish | Velocity commands |
| `/odom` | nav_msgs/Odometry | Subscribe | Robot odometry |
| `/scan` | sensor_msgs/LaserScan | Subscribe | Lidar data |
| `/map` | nav_msgs/OccupancyGrid | Subscribe | SLAM map |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | Navigate to goal |
| `/navigate_through_poses` | nav2_msgs/NavigateThroughPoses | Waypoint navigation |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/ai/parse_intent` | agent_ros_bridge_msgs/ParseIntent | Parse natural language |
| `/safety/validate_motion` | agent_ros_bridge_msgs/ValidateMotion | Validate trajectory |

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_VERSION` | 2 | ROS version (1 or 2) |
| `JWT_SECRET` | - | JWT signing secret |
| `REDIS_URL` | - | Redis connection URL |
| `LOG_LEVEL` | INFO | Logging level |

### Config File

```yaml
# config/bridge.yaml
bridge:
  ros_version: 2
  transports:
    websocket:
      port: 8765
      auth:
        enabled: true
        jwt_secret: ${JWT_SECRET}
    grpc:
      port: 50051
  safety:
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
    workspace_bounds:
      x_min: -10
      x_max: 10
      y_min: -10
      y_max: 10
```

## Error Codes

| Code | Description |
|------|-------------|
| AI001 | Intent parsing failed |
| AI002 | Context resolution failed |
| AI003 | LLM unavailable |
| SAF001 | Validation failed |
| SAF002 | Workspace violation |
| MP001 | Planning failed |
| SYS001 | ROS2 unavailable |
| SYS999 | Unknown error |

## WebSocket Protocol

### Connection

```javascript
const ws = new WebSocket('ws://localhost:8765', 'foxglove.websocket.v1');
```

### Message Format

```json
{
  "header": {
    "msg_type": "command",
    "timestamp": "2024-01-01T00:00:00Z"
  },
  "payload": {
    "command": "navigate",
    "args": {"x": 1.0, "y": 2.0}
  }
}
```

## Docker Deployment

```bash
# Run with Docker
docker run -it \
  --name ros2_humble \
  -p 8765:8765 \
  -p 11311:11311 \
  -e TURTLEBOT3_MODEL=burger \
  ros2_humble:latest

# Start navigation stack
docker exec ros2_humble bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 launch nav2_bringup navigation_launch.py
"
```

## Performance Targets

| Operation | Target | Status |
|-----------|--------|--------|
| Intent parsing | <10ms | ✅ |
| Safety validation | <10ms | ✅ |
| Motion planning | <100ms | ✅ |
| End-to-end | <100ms | ✅ |
