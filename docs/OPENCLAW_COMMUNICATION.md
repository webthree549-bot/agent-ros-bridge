# OpenClaw ↔ Agent ROS Bridge Communication

**How OpenClaw AI agents talk to robots through Agent ROS Bridge**

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  OpenClaw AI Agent (macOS/Host)                                  │
│  - Natural language understanding                               │
│  - Decision making                                              │
│  - Command generation                                           │
└────────────────────┬────────────────────────────────────────────┘
                     │ TCP Socket (JSON)
                     │ Port 9999 (default)
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  Agent ROS Bridge (Docker Container)                             │
│  - TCP Server (receives OpenClaw commands)                      │
│  - ROS2 Bridge (translates to ROS)                              │
│  - Plugin System (handles specific robots)                      │
└────────────────────┬────────────────────────────────────────────┘
                     │ ROS2 Protocol
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  ROS2 Robots / IoT Sensors                                       │
│  - Physical robots                                              │
│  - Simulated robots (mock mode)                                 │
│  - Greenhouse sensors/actuators                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Communication Protocol

### Connection

**Transport:** TCP Socket  
**Default Port:** 9999 (configurable in `config/openclaw_config.yaml`)  
**Format:** JSON messages  
**Direction:** Bidirectional

### Message Flow

#### 1. OpenClaw → Bridge (Command)

```json
{
  "action": "read_sensor",
  "sensor": "temperature",
  "request_id": "uuid-123"
}
```

Or actuator command:
```json
{
  "action": "write_actuator",
  "actuator": "fan",
  "value": true,
  "request_id": "uuid-124"
}
```

#### 2. Bridge → OpenClaw (Response)

```json
{
  "status": "ok",
  "data": {
    "temperature": 25.5,
    "humidity": 60.0
  },
  "request_id": "uuid-123"
}
```

Or error:
```json
{
  "status": "error",
  "message": "Sensor not found",
  "request_id": "uuid-123"
}
```

---

## Configuration

### OpenClaw Config (`config/openclaw_config.yaml`)

```yaml
default_version: "v2"
openclaw_versions:
  v2:
    tcp_host: "127.0.0.1"      # Bridge host
    tcp_port: 9999              # Bridge port
    recv_buffer_size: 8192
    send_timeout: 10
    recv_timeout: 10
    reconnect_attempts: 15
global:
  tcp_reconnect: true
  heartbeat_interval: 3.0       # Keep-alive seconds
```

### Environment Setup

```bash
# Required: JWT secret for bridge auth
export JWT_SECRET=$(openssl rand -base64 32)

# Optional: Bridge host/port override
export BRIDGE_HOST=127.0.0.1
export BRIDGE_PORT=9999
```

---

## Example Communication Session

### Greenhouse Control Demo

```python
# OpenClaw Agent (Python pseudocode)
import socket
import json

# Connect to bridge
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 9999))

# 1. Read sensors
request = {
    "action": "read_sensor",
    "sensor": "environment"
}
sock.send(json.dumps(request).encode())

# Receive response
response = json.loads(sock.recv(8192).decode())
# {"status": "ok", "data": {"temperature": 32.0, "humidity": 35.0}}

# 2. AI Decision
if response["data"]["temperature"] > 28:
    # Turn on fan
    command = {
        "action": "write_actuator",
        "actuator": "fan",
        "value": True
    }
    sock.send(json.dumps(command).encode())

# 3. Get confirmation
result = json.loads(sock.recv(8192).decode())
# {"status": "ok", "message": "Fan turned ON"}
```

---

## Available Commands

### System Commands

| Command | Description | Response |
|---------|-------------|----------|
| `ping` | Check bridge health | `{"status": "ok", "message": "pong"}` |
| `get_status` | Get bridge status | Robot list, connection info |
| `discover` | Find robots on network | List of available robots |

### Sensor Commands

| Command | Parameters | Description |
|---------|------------|-------------|
| `read_sensor` | `sensor`: name | Read sensor value |
| `read_all_sensors` | - | Read all sensors |
| `subscribe_sensor` | `sensor`, `interval` | Stream sensor data |

### Actuator Commands

| Command | Parameters | Description |
|---------|------------|-------------|
| `write_actuator` | `actuator`, `value` | Control actuator |
| `get_actuators` | - | List available actuators |

### Robot Commands

| Command | Parameters | Description |
|---------|------------|-------------|
| `move` | `direction`, `distance` | Move robot |
| `navigate` | `x`, `y`, `theta` | Navigate to pose |
| `stop` | - | Emergency stop |
| `get_pose` | - | Get current position |

### Greenhouse Commands

| Command | Description |
|---------|-------------|
| `greenhouse.status` | Get all sensor readings |
| `greenhouse.fan` | Control fan on/off |
| `greenhouse.valve` | Control water valve |
| `greenhouse.light` | Control grow lights |
| `greenhouse.water` | Trigger irrigation |

---

## Docker Setup

### Bridge Container

```yaml
# docker-compose.yml
services:
  bridge:
    image: agent-ros-bridge:latest
    ports:
      - "9999:9999"  # TCP for OpenClaw
      - "8765:8765"  # WebSocket for web
    environment:
      - JWT_SECRET=${JWT_SECRET}
      - OPENCLAW_TCP_PORT=9999
    command: >
      bash -c "python3 scripts/openclaw_tcp_server.py"
```

### Starting the Bridge

```bash
# Start container with OpenClaw TCP server
docker-compose up

# Server listens on 0.0.0.0:9999
# Ready for OpenClaw connections
```

---

## Security

### Authentication

- **JWT Required:** All connections need valid JWT token
- **Token Generation:**
```bash
python3 scripts/generate_token.py --user openclaw --roles admin
```

### Connection Security

- **Localhost Only:** Default binds to 127.0.0.1
- **No Public Exposure:** Never expose port 9999 without firewall
- **TLS Recommended:** For production, use TLS tunnel

---

## Error Handling

### Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `Connection refused` | Bridge not running | Start Docker container |
| `Authentication failed` | Invalid JWT | Generate new token |
| `Timeout` | Robot not responding | Check ROS connection |
| `Unknown action` | Wrong command | Check command spelling |

### Retry Logic

```python
# OpenClaw handles reconnection automatically
reconnect_attempts: 15      # Configurable
heartbeat_interval: 3.0     # Keep connection alive
```

---

## Integration Points

### From OpenClaw Side

```python
# OpenClaw agent code
from openclaw import Agent

agent = Agent()

@agent.command()
def check_greenhouse():
    """Read greenhouse sensors"""
    response = agent.bridge.send({
        "action": "greenhouse.status"
    })
    return f"Temp: {response['temperature']}°C"
```

### From Bridge Side

```python
# Agent ROS Bridge plugin
from agent_ros_bridge import Plugin

class GreenhousePlugin(Plugin):
    def handle_command(self, command):
        if command["action"] == "greenhouse.status":
            return {
                "temperature": self.read_temp(),
                "humidity": self.read_humidity()
            }
```

---

## Summary

**OpenClaw talks to Agent ROS Bridge via:**

1. **TCP Socket** (port 9999) - Primary connection
2. **JSON Protocol** - Message format
3. **Request/Response** - Synchronous commands
4. **Bidirectional** - Bridge can push updates
5. **Authenticated** - JWT tokens required

**This enables:**
- Natural language → Robot control
- Chat interface → Physical robots
- AI decisions → Real-world actions
