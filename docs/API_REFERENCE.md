# API Reference

Complete API reference for OpenClaw ROS Bridge v2.

## Python API

### Gateway

```python
class OpenClawGateway:
    """Main gateway class for robot management"""
    
    def __init__(self, config: Optional[Dict] = None)
    """Initialize gateway with optional configuration"""
    
    async def start(self) -> None
    """Start the gateway and all transports"""
    
    async def stop(self) -> None
    """Stop the gateway and cleanup"""
    
    def create_fleet(self, name: str) -> RobotFleet
    """Create a new robot fleet"""
    
    async def connect_robot(self, uri: str, fleet_name: Optional[str] = None) -> Robot
    """Connect to a robot at the given URI"""
```

### Robot

```python
class Robot:
    """Represents a connected robot"""
    
    async def execute(self, command: Command) -> Any
    """Execute a command on the robot"""
    
    async def subscribe(self, topic: str) -> AsyncIterator[Telemetry]
    """Subscribe to telemetry from the robot"""
```

### Fleet

```python
class RobotFleet:
    """Manages a group of robots"""
    
    async def broadcast(self, command: Command, selector: Optional[Callable] = None) -> Dict
    """Broadcast command to selected robots"""
```

### Plugin

```python
class Plugin(ABC):
    """Base class for plugins"""
    
    name: str
    version: str
    
    async def initialize(self, gateway: OpenClawGateway) -> None
    """Called when plugin is loaded"""
    
    async def shutdown(self) -> None
    """Called when plugin is unloaded"""
    
    async def handle_message(self, message: Message, identity: Identity) -> Optional[Message]
    """Handle incoming messages"""
```

## Message Format

### Request

```json
{
  "header": {
    "message_id": "uuid",
    "timestamp": "2026-02-13T08:30:00Z",
    "source": "client_id",
    "target": "gateway",
    "correlation_id": "optional_uuid"
  },
  "command": {
    "action": "action_name",
    "parameters": {},
    "timeout_ms": 5000,
    "priority": 5
  },
  "metadata": {}
}
```

### Response

```json
{
  "header": {
    "message_id": "uuid",
    "timestamp": "2026-02-13T08:30:01Z",
    "correlation_id": "original_uuid"
  },
  "telemetry": {
    "topic": "/response",
    "data": {},
    "quality": 1.0
  },
  "status": "ok"
}
```

## Commands

### Core Commands

| Command | Parameters | Response |
|---------|------------|----------|
| `ping` | None | `{"pong": true}` |
| `get_status` | None | Gateway status |
| `list_handlers` | None | List of handlers |
| `discover` | None | List of robots |

### Greenhouse Commands

| Command | Parameters |
|---------|------------|
| `greenhouse.status` | `{"id": "demo"}` |
| `greenhouse.read_sensors` | `{"id": "demo"}` |
| `greenhouse.fan` | `{"id": "demo", "on": true}` |
| `greenhouse.valve` | `{"id": "demo", "open": true}` |

## Configuration Schema

```yaml
gateway:
  name: string
  log_level: enum [DEBUG, INFO, WARNING, ERROR]

transports:
  websocket:
    enabled: boolean
    host: string
    port: integer
    tls_cert: string (optional)
    tls_key: string (optional)
  
  grpc:
    enabled: boolean
    host: string
    port: integer

plugins:
  - name: string
    enabled: boolean
    options: object
```

---

For complete examples, see [User Manual](USER_MANUAL.md).
