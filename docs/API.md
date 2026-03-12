# Agent ROS Bridge - API Documentation

## Overview

Agent ROS Bridge provides a universal interface for AI agents to control ROS-based robots through multiple transport protocols.

## Core Concepts

### Bridge
The central orchestrator that manages transports, connectors, and message routing.

```python
from agent_ros_bridge import Bridge

bridge = Bridge(ros_version=2)
await bridge.start()
```

### Transports
Communication channels for different protocols:
- **WebSocket**: Browser-based clients
- **gRPC**: High-performance microservices
- **MQTT**: IoT device integration
- **LCM**: Low-latency internal communication

### Modules (New in v0.6.1)
Self-contained components with typed input/output streams.

```python
from agent_ros_bridge.gateway_v2 import Module, In, Out

class CameraModule(Module):
    image: Out[Image]
    config: In[dict]
    
    async def run(self):
        while self._running:
            img = await capture_image()
            await self.image.publish(img)
```

### Blueprints (New in v0.6.1)
Declarative composition of modules and their connections.

```python
from agent_ros_bridge.gateway_v2 import Blueprint, autoconnect

blueprint = Blueprint()
    .add_module("camera", camera_bp)
    .add_module("detector", detector_bp)
    .connect("camera", "image", "detector", "input")

await blueprint.start()
```

## API Reference

### Bridge

#### `Bridge(ros_version: int = 2)`
Create a new bridge instance.

**Parameters:**
- `ros_version`: ROS version (1 or 2)

**Methods:**
- `start() -> bool`: Start the bridge
- `stop() -> None`: Stop the bridge
- `register_transport(transport)`: Add a transport
- `register_connector(connector)`: Add a connector

### Transports

#### `WebSocketTransport(config: dict)`
WebSocket transport for browser clients.

**Config Options:**
- `host`: Bind address (default: "0.0.0.0")
- `port`: Port number (default: 8765)
- `auth`: Authentication config
- `tls_cert`: TLS certificate path
- `tls_key`: TLS key path

#### `LCMTransport(config: dict)` (New in v0.6.1)
High-performance LCM transport.

**Config Options:**
- `udp_url`: Multicast URL (default: "udpm://239.255.76.67:7667")
- `shared_memory`: Enable shared memory (default: True)
- `queue_size`: Message queue size (default: 1000)

**Example:**
```python
from agent_ros_bridge.gateway_v2.transports import LCMTransport

transport = LCMTransport({
    "udp_url": "udpm://239.255.76.67:7667",
    "shared_memory": True
})

# Create publisher
pub = transport.publisher("robot/commands")
pub.publish({"cmd": "move", "speed": 0.5})

# Create subscriber
def on_message(data):
    print(f"Received: {data}")

sub = transport.subscriber("robot/commands", on_message)
sub.subscribe()
```

### Modules (New in v0.6.1)

#### `Module`
Base class for all modules.

**Type Annotations:**
- `In[T]`: Input stream of type T
- `Out[T]`: Output stream of type T

**Methods:**
- `start()`: Start the module
- `stop()`: Stop the module
- `run()`: Main loop (override in subclass)

**Decorators:**
- `@rpc`: Mark method as RPC callable
- `@skill`: Mark method as AI-callable skill

**Example:**
```python
class NavigationModule(Module):
    target: In[Pose]
    cmd_vel: Out[Twist]
    
    @rpc
    def set_max_speed(self, speed: float):
        self.max_speed = speed
    
    @skill
    def navigate_to(self, x: float, y: float) -> bool:
        """AI-callable skill to navigate to position."""
        return True
    
    async def run(self):
        while self._running:
            target = await self.target.get()
            cmd = self.compute_velocity(target)
            await self.cmd_vel.publish(cmd)
```

### Blueprints (New in v0.6.1)

#### `Blueprint`
Compose modules into systems.

**Methods:**
- `add_module(name, blueprint)`: Add a module
- `connect(src_mod, src_stream, dst_mod, dst_stream)`: Connect streams
- `autoconnect()`: Auto-connect by matching names/types
- `build()`: Instantiate all modules
- `start()`: Start the system
- `stop()`: Stop the system

#### `autoconnect(*blueprints)`
Automatically connect modules by matching stream names and types.

**Example:**
```python
from agent_ros_bridge.gateway_v2 import autoconnect

blueprint = autoconnect(
    CameraModule.blueprint(),
    DetectorModule.blueprint(),
    NavigationModule.blueprint()
)

await blueprint.start()
```

### Safety

#### `SafetyValidator`
Validate trajectories and commands.

```python
from agent_ros_bridge.safety import SafetyValidator

validator = SafetyValidator()
result = validator.validate_trajectory(
    trajectory={"waypoints": [...]},
    limits={"max_velocity": 1.0}
)
```

### Error Handling (New in v0.6.1)

#### `AgentError`
Standardized error with code and context.

```python
from agent_ros_bridge.utils.error_handling import AgentError, ErrorCode

raise AgentError(
    code=ErrorCode.VALIDATION_FAILED,
    message="Trajectory exceeds velocity limits",
    context={"max_vel": 1.0, "requested": 1.5}
)
```

#### `InputValidator`
Validate inputs across AI services.

```python
from agent_ros_bridge.utils.error_handling import InputValidator

result = InputValidator.validate_utterance("go to kitchen")
if not result.valid:
    raise AgentError(result.error_code, result.error_message)
```

#### `CircuitBreaker`
Prevent cascade failures.

```python
from agent_ros_bridge.utils.error_handling import CircuitBreaker

cb = CircuitBreaker(failure_threshold=5, recovery_timeout=30)

@with_circuit_breaker(cb)
def call_llm(prompt):
    # May fail
    pass
```

### Security (Enhanced in v0.6.1)

#### Password Hashing
```python
from agent_ros_bridge.security_utils import hash_password, verify_password

hashed = hash_password("my_password")
is_valid = verify_password("my_password", hashed)
```

#### Token Generation
```python
from agent_ros_bridge.security_utils import generate_token

token = generate_token(length=32)
```

#### API Key Management
```python
from agent_ros_bridge.security_utils import generate_api_key, validate_api_key

key_data = generate_api_key("robot_api")
stored_hash = hash_api_key(key_data["key"])
is_valid = validate_api_key(key_data["key"], stored_hash)
```

#### Encryption
```python
from agent_ros_bridge.security_utils import encrypt, decrypt

key = secrets.token_bytes(32)
encrypted = encrypt("sensitive data", key)
decrypted = decrypt(encrypted, key)
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_VERSION` | 2 | ROS version (1 or 2) |
| `JWT_SECRET` | - | JWT signing secret |
| `REDIS_URL` | - | Redis connection URL |
| `LOG_LEVEL` | INFO | Logging level |
| `LCM_URL` | udpm://239.255.76.67:7667 | LCM multicast URL |

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
    lcm:
      udp_url: "udpm://239.255.76.67:7667"
      shared_memory: true
  safety:
    max_linear_velocity: 1.0
    max_angular_velocity: 1.0
```

## Examples

### Basic Robot Control

```python
import asyncio
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

async def main():
    bridge = Bridge(ros_version=2)
    
    # Add WebSocket transport
    bridge.transport_manager.register(
        WebSocketTransport({"port": 8765})
    )
    
    await bridge.start()
    print("Bridge running on ws://localhost:8765")
    
    # Keep running
    while True:
        await asyncio.sleep(1)

asyncio.run(main())
```

### Module-Based Architecture

```python
from agent_ros_bridge.gateway_v2 import Module, In, Out, rpc, skill

class RobotModule(Module):
    cmd_vel: Out[Twist]
    odometry: In[Odometry]
    
    @skill
    def move_forward(self, distance: float) -> bool:
        """AI-callable: Move forward by distance."""
        # Implementation
        return True
    
    async def run(self):
        while self._running:
            odom = await self.odometry.get()
            # Process odometry
            await asyncio.sleep(0.1)

# Use in blueprint
from agent_ros_bridge.gateway_v2 import Blueprint

blueprint = Blueprint()
blueprint.add_module("robot", RobotModule.blueprint())
await blueprint.start()
```

### Mixed Transport System

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports import (
    WebSocketTransport,
    LCMTransport
)

bridge = Bridge()

# WebSocket for external AI agents
bridge.transport_manager.register(
    WebSocketTransport({"port": 8765})
)

# LCM for internal high-performance communication
bridge.transport_manager.register(
    LCMTransport({"udp_url": "udpm://239.255.76.67:7667"})
)

await bridge.start()
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

## Performance Targets

| Operation | Target | Status |
|-----------|--------|--------|
| Intent parsing | <10ms | ✅ |
| Safety validation | <10ms | ✅ |
| Motion planning | <100ms | ✅ |
| LCM latency | <1ms | ✅ |
| End-to-end | <100ms | ✅ |

## Version History

### v0.6.1 (Current)
- LCM transport for high-performance messaging
- Blueprint pattern for module composition
- Enhanced security utilities
- Comprehensive error handling
- 587+ tests, 32% coverage

### v0.6.0
- Initial ROS2 support
- WebSocket/gRPC/MQTT transports
- Safety validation
- Fleet management

## Support

- GitHub Issues: https://github.com/agent-ros-bridge/issues
- Documentation: https://docs.agent-ros-bridge.ai
- Discord: https://discord.gg/agent-ros-bridge
