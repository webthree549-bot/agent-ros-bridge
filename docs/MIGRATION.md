# Migration Guide: v1 to v2

This guide helps you migrate from OpenClaw ROS Bridge v1 to v2.

## Overview

OpenClaw ROS Bridge v2 introduces a new **Gateway Architecture** that is:
- **Multi-Protocol**: Support for WebSocket, gRPC, MQTT, not just TCP
- **Multi-Robot**: Manage fleets of robots, not just one
- **Plugin-Based**: Dynamic plugin loading system
- **Cloud-Native**: Designed for Docker and Kubernetes

## Quick Migration Checklist

- [ ] Update imports from `openclaw_ros_bridge` to `openclaw_ros_bridge.gateway_v2`
- [ ] Replace `openclaw_server` with `OpenClawGateway`
- [ ] Update plugin registration
- [ ] Test with new command format
- [ ] Update configuration files

## Key Changes

### 1. Import Changes

**v1:**
```python
from openclaw_ros_bridge import openclaw_server
from openclaw_ros_bridge.hal import sensor_hal, actuator_hal
```

**v2:**
```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
```

### 2. Server Initialization

**v1:**
```python
from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server

openclaw_server.start()
```

**v2:**
```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

async def main():
    gateway = OpenClawGateway()
    
    # Add transports
    gateway.transport_manager.register(
        WebSocketTransport({"port": 8765})
    )
    
    # Start
    await gateway.start()

asyncio.run(main())
```

### 3. Plugin Registration

**v1:**
```python
class MyPlugin:
    def register(self, server):
        server.register_handler("my_command", self.handle)

plugin = MyPlugin()
plugin.register(openclaw_server)
```

**v2:**
```python
from openclaw_ros_bridge.gateway_v2.core import Plugin

class MyPlugin(Plugin):
    name = "my_plugin"
    version = "1.0.0"
    
    async def initialize(self, gateway):
        # Plugin initialization
        pass
    
    async def handle_message(self, message, identity):
        # Handle messages
        if message.command and message.command.action == "my_command":
            return Message(...)
        return None

# Load plugin
gateway = OpenClawGateway()
await gateway.plugin_manager.load_plugin(MyPlugin())
```

### 4. Command Format

**v1:**
```json
{
  "action": "read_sensor",
  "sensor": "env"
}
```

**v2:**
```json
{
  "command": {
    "action": "read_sensor",
    "parameters": {
      "sensor": "env"
    }
  }
}
```

### 5. Configuration

**v1:**
```python
# Environment variables
export ROS_DISTRO=jazzy
export MOCK_MODE=true
```

**v2:**
```yaml
# config/gateway.yaml
name: "my_gateway"
log_level: INFO

transports:
  websocket:
    enabled: true
    port: 8765
  
  grpc:
    enabled: true
    port: 50051
```

## Feature Mapping

| v1 Feature | v2 Equivalent | Notes |
|------------|---------------|-------|
| TCP Server | WebSocket Transport | More protocols available |
| `register_handler()` | Plugin System | More flexible |
| Direct ROS integration | Connector Layer | Supports multiple platforms |
| Single robot | Fleet Management | Multiple robots |
| Config files | YAML Config | More structured |

## Common Migration Scenarios

### Scenario 1: Simple TCP Server

**v1 Code:**
```python
from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server

openclaw_server.start()
```

**v2 Code:**
```python
import asyncio
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

async def main():
    gateway = OpenClawGateway()
    gateway.transport_manager.register(
        WebSocketTransport({"port": 9999})  # Same port as v1
    )
    await gateway.start()
    
    # Keep running
    while True:
        await asyncio.sleep(1)

asyncio.run(main())
```

### Scenario 2: Custom Plugin

**v1 Code:**
```python
class MyPlugin:
    def register(self, server):
        server.register_handler("custom_cmd", self.handle_custom)
    
    def handle_custom(self, cmd):
        return {"status": "ok", "data": cmd}

plugin = MyPlugin()
plugin.register(openclaw_server)
```

**v2 Code:**
```python
from openclaw_ros_bridge.gateway_v2.core import Plugin, Message, Header, Telemetry

class MyPlugin(Plugin):
    name = "my_plugin"
    
    async def initialize(self, gateway):
        self.gateway = gateway
    
    async def handle_message(self, message, identity):
        if not message.command:
            return None
        
        if message.command.action == "custom_cmd":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="/result",
                    data={"status": "ok"}
                )
            )
        return None

# Usage
gateway = OpenClawGateway()
await gateway.plugin_manager.load_plugin(MyPlugin())
```

### Scenario 3: ROS Integration

**v1 Code:**
```python
from openclaw_ros_bridge import get_ros_communicator

ros = get_ros_communicator()
ros.publish("/topic", data)
```

**v2 Code:**
```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

async def main():
    gateway = OpenClawGateway()
    gateway.connector_registry.register(ROS2Connector())
    
    # Connect to robot
    robot = await gateway.connect_robot("ros2://192.168.1.100")
    
    # Execute command
    result = await robot.execute({
        "action": "publish",
        "parameters": {"topic": "/topic", "data": data}
    })

asyncio.run(main())
```

## Backward Compatibility

v1 code is preserved in `openclaw_ros_bridge/legacy/` for backward compatibility.

To continue using v1:
```python
from openclaw_ros_bridge.legacy import openclaw_server
```

**Note:** v1 is deprecated and will be removed in a future version.

## Testing Your Migration

1. **Start with mock mode:**
   ```bash
   export MOCK_MODE=true
   openclaw-gateway --demo
   ```

2. **Test basic connectivity:**
   ```bash
   curl http://localhost:8765/health
   ```

3. **Test commands:**
   ```bash
   wscat -c ws://localhost:8765
   > {"command": {"action": "ping"}}
   ```

4. **Verify plugin loading:**
   ```bash
   > {"command": {"action": "list_handlers"}}
   ```

## Getting Help

- **Documentation:** https://openclaw-ros-bridge.readthedocs.io
- **Issues:** https://github.com/webthree549-bot/openclaw-ros-bridge/issues
- **Discussions:** https://github.com/webthree549-bot/openclaw-ros-bridge/discussions

## Timeline

| Version | Status | Support Until |
|---------|--------|---------------|
| v1.x | Deprecated | December 2026 |
| v2.x | Current | Active |

---

**Last Updated:** February 2026  
**Maintainer:** OpenClaw ROS Team
