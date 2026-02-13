# Greenhouse Demo - Application Example

This is a **demo application** built on top of the generic OpenClaw ROS Bridge. It demonstrates how to create application-specific functionality using the plugin architecture.

## Overview

The greenhouse demo shows:
- How to register custom command handlers with the generic TCP server
- How to use the HAL (Hardware Abstraction Layer) for sensors and actuators
- How to build a complete application on top of the bridge infrastructure

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                 OpenClaw AI Agent                           │
└──────────────────┬──────────────────────────────────────────┘
                   │ TCP/JSON
                   ▼
┌─────────────────────────────────────────────────────────────┐
│           Greenhouse Server (Application)                   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  OpenClaw TCP Server (Generic - Core Bridge)       │   │
│  │  • Core commands: ping, get_status, list_handlers  │   │
│  └─────────────────────────────────────────────────────┘   │
│                           ▲                                 │
│  ┌────────────────────────┴────────────────────────────┐   │
│  │      Greenhouse Plugin (Application-Specific)      │   │
│  │  • read_sensor (temperature, humidity)             │   │
│  │  • write_actuator (fan, valve)                     │   │
│  │  • get_greenhouse_status                           │   │
│  └─────────────────────────────────────────────────────┘   │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              HAL (Hardware Abstraction Layer)               │
│     • sensor_hal - Temperature, humidity, camera           │
│     • actuator_hal - Fan, valve, motors, grippers          │
└─────────────────────────────────────────────────────────────┘
```

## Files

| File | Purpose |
|------|---------|
| `greenhouse_plugin.py` | Plugin that registers greenhouse-specific commands |
| `greenhouse_server.py` | Demo server launcher (combines generic server + plugin) |
| `scripts/run_demo.sh` | Run greenhouse demo standalone |
| `scripts/demo_openclaw.sh` | Full OpenClaw + greenhouse integration demo |
| `scripts/gh_control.sh` | Command-line control utility |

## Quick Start

```bash
# From project root

# 1. Start Docker container
./scripts/docker_start.sh --jazzy

# 2. Run greenhouse demo
./demo/greenhouse/scripts/run_demo.sh

# Or in mock mode (no Docker/ROS required):
export MOCK_MODE=true
./demo/greenhouse/scripts/run_demo.sh
```

## Creating Your Own Application

To build a custom application on the bridge:

1. **Create a plugin** (see `greenhouse_plugin.py`):
```python
from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server

class MyRobotPlugin:
    def register(self, server):
        server.register_handler("move_arm", self.handle_move_arm)
        server.register_handler("gripper", self.handle_gripper)
    
    def handle_move_arm(self, cmd):
        # Your logic here
        return {"status": "ok", "position": [x, y, z]}
```

2. **Launch with plugin**:
```python
from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server
from my_app.my_plugin import MyRobotPlugin

plugin = MyRobotPlugin()
plugin.register(openclaw_server)
openclaw_server.start()
```

3. **Use generic commands + your custom commands**:
```python
# Core commands (always available)
{"action": "ping"}
{"action": "get_status"}
{"action": "list_handlers"}

# Your custom commands
{"action": "move_arm", "x": 0.5, "y": 0.2, "z": 0.1}
{"action": "gripper", "state": "close"}
```

## Commands

### Core Commands (Generic Bridge)

| Command | Description |
|---------|-------------|
| `ping` | Health check |
| `get_status` | Get bridge status and registered handlers |
| `list_handlers` | List available command handlers |

### Greenhouse Commands (Application-Specific)

| Command | Parameters | Description |
|---------|------------|-------------|
| `read_sensor` | `sensor`: "env" | Read temperature/humidity |
| `write_actuator` | `actuator`: "fan"\|"valve", `value`: bool | Control actuators |
| `get_greenhouse_status` | - | Get full greenhouse status |

## Notes

- This is a **demo application**, not the core bridge functionality
- The core bridge (`openclaw_tcp_server.py`) is application-agnostic
- Applications register their own handlers to extend functionality
- Multiple applications can share the same bridge infrastructure
