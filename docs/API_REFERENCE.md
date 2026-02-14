# Agent ROS Bridge - API Reference

Complete API reference for developers.

## Table of Contents

1. [Core Classes](#core-classes)
2. [Transports](#transports)
3. [Connectors](#connectors)
4. [Plugins](#plugins)
5. [Fleet Management](#fleet-management)
6. [Actions](#actions)
7. [Metrics](#metrics)
8. [Authentication](#authentication)

---

## Core Classes

### Bridge

Main gateway class that manages transports, connectors, and plugins.

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
```

#### Methods

##### `register_transport(transport: Transport)`
Register a transport endpoint.

```python
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

ws_transport = WebSocketTransport({'port': 8765})
bridge.transport_manager.register(ws_transport)
```

##### `register_connector(connector: Connector)`
Register a robot connector.

```python
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

ros2_connector = ROS2Connector({'auto_discover': True})
bridge.connector_registry.register(ros2_connector)
```

##### `async start()`
Start the bridge and all registered components.

```python
await bridge.start()
```

##### `async stop()`
Stop the bridge gracefully.

```python
await bridge.stop()
```

### Message

Unified message format for all communications.

```python
from agent_ros_bridge import Message, Header, Command, Telemetry

msg = Message(
    header=Header(),
    command=Command(action="move", parameters={"x": 1.0}),
    telemetry=Telemetry(topic="/odom", data={"x": 0.5})
)
```

#### Fields

| Field | Type | Description |
|-------|------|-------------|
| `header` | Header | Message metadata |
| `command` | Optional[Command] | Robot command |
| `telemetry` | Optional[Telemetry] | Sensor data |
| `event` | Optional[Event] | System event |
| `metadata` | Dict[str, Any] | Additional data |

### Identity

Authenticated identity for connections.

```python
from agent_ros_bridge import Identity

identity = Identity(
    id="user_123",
    name="Operator",
    roles=["operator"],
    metadata={"department": "warehouse"}
)
```

---

## Transports

### WebSocketTransport

WebSocket server for real-time bidirectional communication.

```python
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

transport = WebSocketTransport({
    'host': '0.0.0.0',
    'port': 8765,
    'auth': {
        'enabled': True,
        'jwt_secret': 'secret-key'
    }
})
```

#### Configuration

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `host` | str | '0.0.0.0' | Bind address |
| `port` | int | 8765 | Listen port |
| `tls_cert` | Optional[str] | None | TLS certificate path |
| `tls_key` | Optional[str] | None | TLS key path |
| `auth.enabled` | bool | False | Enable JWT auth |
| `auth.jwt_secret` | Optional[str] | None | JWT secret key |

### MQTTTransport

MQTT client for IoT sensor integration.

```python
from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport

transport = MQTTTransport({
    'host': 'localhost',
    'port': 1883,
    'username': 'user',
    'password': 'pass',
    'subscriptions': ['robots/#', 'sensors/#']
})
```

#### Configuration

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `host` | str | 'localhost' | MQTT broker host |
| `port` | int | 1883 | MQTT broker port |
| `username` | Optional[str] | None | Auth username |
| `password` | Optional[str] | None | Auth password |
| `tls` | bool | False | Enable TLS |
| `subscriptions` | List[str] | ['#'] | Topics to subscribe |

---

## Connectors

### ROS2Connector

Connects to ROS2 robots via rclpy.

```python
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

connector = ROS2Connector({
    'auto_discover': True,
    'domain_id': 0
})
```

#### Configuration

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `auto_discover` | bool | True | Auto-discover robots |
| `domain_id` | int | 0 | ROS_DOMAIN_ID |
| `namespace` | str | '' | Robot namespace |

#### Methods

##### `async discover() -> List[RobotEndpoint]`
Discover available ROS2 robots.

##### `async connect(endpoint: RobotEndpoint) -> Robot`
Connect to a specific robot.

### ROS1Connector

Connects to ROS1 robots via rospy.

```python
from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector

connector = ROS1Connector({
    'auto_discover': True,
    'master_uri': 'http://localhost:11311'
})
```

---

## Plugins

### Creating Custom Plugins

```python
from agent_ros_bridge import Plugin, Message, Identity

class MyPlugin(Plugin):
    name = "my_plugin"
    version = "1.0.0"
    
    async def initialize(self, gateway: Bridge) -> bool:
        """Called when plugin is loaded"""
        self.gateway = gateway
        return True
    
    async def shutdown(self) -> None:
        """Called when plugin is unloaded"""
        pass
    
    async def handle_message(self, message: Message, 
                            identity: Identity) -> Optional[Message]:
        """Handle incoming messages"""
        if message.command and message.command.action == "my_action":
            # Process command
            return Message(telemetry=...)
        return None
```

### Loading Plugins

```python
from agent_ros_bridge.plugins.my_plugin import MyPlugin

plugin = MyPlugin()
await bridge.plugin_manager.load_plugin(plugin)
```

### ArmRobotPlugin

Plugin for controlling robot arms.

```python
from agent_ros_bridge.plugins.arm_robot import ArmRobotPlugin

arm = ArmRobotPlugin(
    arm_type="ur",  # or "xarm", "franka"
    ros_version="ros2",
    namespace=""
)

await arm.initialize(bridge)

# Control commands
result = await arm.handle_command("arm.move_joints", {
    "joints": [0, -1.57, 0, -1.57, 0, 0]
})
```

#### Available Commands

| Command | Parameters | Description |
|---------|------------|-------------|
| `arm.move_joints` | `joints: List[float]` | Move to joint positions |
| `arm.move_cartesian` | `x, y, z, qx, qy, qz, qw` | Move to cartesian pose |
| `arm.get_state` | None | Get current state |
| `arm.gripper` | `position: float` | Control gripper (0-1) |
| `arm.stop` | None | Emergency stop |

---

## Fleet Management

### FleetOrchestrator

Manages multi-robot fleets with task allocation.

```python
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, Task

orchestrator = FleetOrchestrator()
await orchestrator.start()

# Add robot
robot = FleetRobot(
    robot_id="tb4_001",
    name="TurtleBot4-Alpha",
    capabilities=RobotCapability(
        can_navigate=True,
        max_payload_kg=5.0
    )
)
await orchestrator.add_robot(robot)

# Submit task
task = Task(
    type="navigate",
    target_location="zone_a",
    priority=5
)
task_id = await orchestrator.submit_task(task)
```

#### Task Types

| Type | Description | Required Capabilities |
|------|-------------|---------------------|
| `navigate` | Move to location | `can_navigate` |
| `transport` | Carry payload | `can_navigate`, payload capacity |
| `manipulate` | Arm operation | `can_manipulate` |
| `charge` | Go to charger | `can_navigate` |

#### Methods

##### `async add_robot(robot: FleetRobot) -> bool`
Add a robot to the fleet.

##### `async remove_robot(robot_id: str) -> bool`
Remove a robot from the fleet.

##### `async submit_task(task: Task) -> str`
Submit a task to the queue. Returns task ID.

##### `async cancel_task(task_id: str) -> bool`
Cancel a pending or executing task.

##### `get_metrics() -> FleetMetrics`
Get fleet performance metrics.

##### `get_fleet_status() -> Dict`
Get complete fleet status for dashboards.

---

## Actions

### ROS Actions

For long-running tasks with feedback.

```python
from agent_ros_bridge.actions import create_action_client, ActionFeedback

# Create client
client = create_action_client(
    action_name="navigate_to_pose",
    action_type="nav2_msgs/action/NavigateToPose",
    ros_version="ros2"
)

await client.connect()

# Register callbacks
def on_feedback(feedback: ActionFeedback):
    print(f"Distance remaining: {feedback.feedback_data['distance_remaining']}")

def on_result(result):
    print(f"Navigation {'succeeded' if result.success else 'failed'}")

client.register_feedback_callback(on_feedback)
client.register_result_callback(on_result)

# Send goal
result = await client.send_goal(
    goal_data={"pose": {"x": 5.0, "y": 3.0, "theta": 1.57}},
    timeout_sec=60.0
)
```

#### ActionClient Methods

##### `async connect() -> bool`
Connect to action server.

##### `async send_goal(goal_data, timeout_sec=30.0) -> ActionResult`
Send goal and wait for result.

##### `async cancel_goal() -> bool`
Cancel current goal.

##### `register_feedback_callback(callback)`
Register callback for feedback updates.

##### `register_result_callback(callback)`
Register callback for results.

---

## Metrics

### MetricsCollector

Collects and exposes Prometheus metrics.

```python
from agent_ros_bridge.metrics import get_metrics

metrics = get_metrics()

# Record metrics
metrics.record_message_sent("websocket", size_bytes=1024)
metrics.record_task_completed(task_type="navigate", duration_sec=5.2)
metrics.set_robots_online(4)

# Get snapshot
snapshot = metrics.get_snapshot()
print(f"Robots: {snapshot.robots_online}/{snapshot.robots_total}")
```

#### Recording Methods

##### `record_message_sent(transport, size_bytes=0)`
Record a sent message.

##### `record_message_received(transport, size_bytes=0)`
Record a received message.

##### `record_task_completed(task_type, duration_sec)`
Record task completion with duration.

##### `record_task_failed(task_type)`
Record task failure.

##### `set_robots_online(count)`
Set number of online robots.

##### `set_active_connections(count, transport)`
Set active connection count.

### MetricsServer

HTTP server for Prometheus scraping.

```python
from agent_ros_bridge.metrics import MetricsServer

server = MetricsServer(port=9090)
await server.start()

# Metrics available at http://localhost:9090/metrics
```

---

## Authentication

### Authenticator

JWT and API key authentication.

```python
from agent_ros_bridge.gateway_v2.auth import Authenticator, AuthConfig

config = AuthConfig(
    enabled=True,
    jwt_secret="your-secret-key",
    jwt_expiry_hours=24,
    api_keys={
        "ak_123456": {
            "user_id": "operator",
            "roles": ["operator"]
        }
    }
)

auth = Authenticator(config)

# Create token
token = auth.create_token("user_123", roles=["operator"])

# Verify token
payload = auth.verify_token(token)
if payload:
    print(f"User: {payload['sub']}, Roles: {payload['roles']}")
```

### Role-Based Access Control

```python
from agent_ros_bridge.gateway_v2.auth import RoleBasedAccessControl

rbac = RoleBasedAccessControl()

# Check permission
if rbac.can_execute(roles=["operator"], action="move"):
    # Allow action
    pass
```

#### Default Roles

| Role | Permissions |
|------|-------------|
| `admin` | All actions |
| `operator` | list_robots, get_state, move, publish |
| `viewer` | list_robots, get_state, subscribe |
| `anonymous` | list_robots only |

---

## WebSocket Protocol

### Connection

```
ws://host:port?token=JWT_TOKEN
```

### Message Format

**Request:**
```json
{
  "header": {
    "message_id": "uuid",
    "timestamp": "2025-01-01T00:00:00Z"
  },
  "command": {
    "action": "action_name",
    "parameters": {
      "key": "value"
    }
  }
}
```

**Response:**
```json
{
  "header": {
    "message_id": "response_uuid",
    "correlation_id": "request_uuid",
    "timestamp": "2025-01-01T00:00:01Z"
  },
  "telemetry": {
    "topic": "response",
    "data": {
      "result": "..."
    },
    "quality": 1.0
  }
}
```

### Error Response

```json
{
  "header": {...},
  "event": {
    "event_type": "error",
    "severity": "error",
    "data": {
      "error": "Error message",
      "code": "ERROR_CODE"
    }
  }
}
```

---

## Constants

### QoS Levels

```python
from agent_ros_bridge import QoS

QoS.BEST_EFFORT     # Fire and forget
QoS.AT_LEAST_ONCE   # Retry until ack
QoS.EXACTLY_ONCE    # Deduplication guaranteed
```

### Default Ports

| Service | Port |
|---------|------|
| WebSocket | 8765 |
| MQTT | 1883 |
| gRPC | 50051 |
| Metrics | 9090 |
| Dashboard | 8080 |

---

## Type Definitions

```python
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

# Message types
CommandData = Dict[str, Any]
TelemetryData = Dict[str, Any]
EventData = Dict[str, Any]

# Robot types
RobotID = str
TaskID = str

# Configuration
ConfigDict = Dict[str, Any]
```

---

## Exceptions

```python
class BridgeError(Exception):
    """Base bridge exception"""
    pass

class ConnectionError(BridgeError):
    """Connection failed"""
    pass

class AuthenticationError(BridgeError):
    """Authentication failed"""
    pass

class TaskError(BridgeError):
    """Task execution failed"""
    pass
```

---

## See Also

- [User Manual](USER_MANUAL.md) - Complete user guide
- [Architecture](ARCHITECTURE.md) - System design
- [Native ROS](NATIVE_ROS.md) - ROS installation
- [Multi-ROS](MULTI_ROS.md) - Fleet management
