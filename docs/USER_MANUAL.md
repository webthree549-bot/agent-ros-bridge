# OpenClaw ROS Bridge - User Manual

**Version 2.0.0** | **Last Updated: February 2026**

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Installation](#2-installation)
3. [Quick Start Guide](#3-quick-start-guide)
4. [Architecture Overview](#4-architecture-overview)
5. [Configuration](#5-configuration)
6. [Using the Gateway](#6-using-the-gateway)
7. [Working with Robots](#7-working-with-robots)
8. [Plugin Development](#8-plugin-development)
9. [Troubleshooting](#9-troubleshooting)
10. [API Reference](#10-api-reference)
11. [Advanced Topics](#11-advanced-topics)
12. [Migration from v1](#12-migration-from-v1)

---

## 1. Introduction

### 1.1 What is OpenClaw ROS Bridge?

OpenClaw ROS Bridge is a **Universal Robot Gateway** that enables OpenClaw AI agents to control ROS-based robots and embodied intelligence systems. It provides:

- **Multi-Protocol Support**: WebSocket, gRPC, MQTT, TCP
- **Multi-Robot Management**: Control fleets of robots
- **Plugin Architecture**: Build custom robot applications
- **Version Agnostic**: Works with ROS1 and ROS2
- **Simulation Ready**: Test without physical hardware

### 1.2 Who Is This For?

- **AI Developers**: Connect OpenClaw agents to physical robots
- **Robotics Engineers**: Universal interface for diverse robot platforms
- **Researchers**: Rapid prototyping and simulation
- **Industrial Users**: Fleet management and automation

### 1.3 Key Features

| Feature | Description |
|---------|-------------|
| Multi-Protocol | WebSocket, gRPC, MQTT, TCP, HTTP |
| Multi-Robot | Manage fleets of robots |
| Plugin System | Build custom applications |
| Cloud-Native | Docker, Kubernetes ready |
| Production-Ready | CI/CD, security, observability |

---

## 2. Installation

### 2.1 Requirements

**System Requirements:**
- Python 3.8 or higher
- Docker (optional, for containerized deployment)
- ROS1 Noetic or ROS2 Humble/Jazzy (optional, for physical robots)

**Supported Platforms:**
- Ubuntu 20.04/22.04/24.04
- macOS 12+
- Windows (WSL2)
- ARM64 (Raspberry Pi, Jetson)

### 2.2 Installation Methods

#### Method 1: Via ClawHub (Recommended)

```bash
openclaw skills add openclaw-ros-bridge
```

#### Method 2: Via PyPI

```bash
pip install openclaw-ros-bridge

# With all extras
pip install "openclaw-ros-bridge[all]"

# Development install
pip install "openclaw-ros-bridge[dev,test]"
```

#### Method 3: From Source

```bash
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git
cd openclaw-ros-bridge
pip install -e ".[dev]"
```

#### Method 4: Docker

```bash
# Pull and run
docker run -p 8765:8765 -p 50051:50051 \
  ghcr.io/webthree549-bot/openclaw-ros-bridge:latest

# Or with docker-compose
docker-compose up -d
```

### 2.3 Verify Installation

```bash
# Check version
openclaw-gateway --version

# Check installation
python -c "from openclaw_ros_bridge.gateway_v2 import OpenClawGateway; print('OK')"
```

---

## 3. Quick Start Guide

### 3.1 Your First Robot Connection

#### Step 1: Start the Gateway

```bash
# Start with demo mode (includes greenhouse simulation)
openclaw-gateway --demo
```

You should see:
```
🤖 OpenClaw Gateway v2.0.0
Listening on:
  - WebSocket: ws://0.0.0.0:8765
  - gRPC: grpc://0.0.0.0:50051
Press Ctrl+C to stop
```

#### Step 2: Test Connection

**Using WebSocket:**
```bash
# Install wscat if needed
npm install -g wscat

# Connect
wscat -c ws://localhost:8765

# Send ping
> {"command": {"action": "ping"}}
< {"status": "ok", "pong": true}
```

**Using Python:**
```python
import asyncio
import websockets
import json

async def test():
    async with websockets.connect("ws://localhost:8765") as ws:
        await ws.send(json.dumps({
            "command": {"action": "ping"}
        }))
        response = await ws.recv()
        print(response)

asyncio.run(test())
```

#### Step 3: Control the Greenhouse Demo

```python
import asyncio
import websockets
import json

async def control_greenhouse():
    uri = "ws://localhost:8765"
    
    async with websockets.connect(uri) as ws:
        # Get status
        await ws.send(json.dumps({
            "command": {"action": "greenhouse.status"}
        }))
        status = await ws.recv()
        print(f"Status: {status}")
        
        # Turn on fan
        await ws.send(json.dumps({
            "command": {
                "action": "greenhouse.fan",
                "parameters": {"on": True}
            }
        }))
        result = await ws.recv()
        print(f"Fan result: {result}")

asyncio.run(control_greenhouse())
```

### 3.2 Quick Commands Reference

| Command | Description | Example |
|---------|-------------|---------|
| `ping` | Health check | `{"command": {"action": "ping"}}` |
| `discover` | Find robots | `{"command": {"action": "discover"}}` |
| `get_status` | Gateway status | `{"command": {"action": "get_status"}}` |
| `greenhouse.status` | Demo status | `{"command": {"action": "greenhouse.status"}}` |

---

## 4. Architecture Overview

### 4.1 Three-Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    AI Agent (OpenClaw)                      │
└──────────────────────────┬──────────────────────────────────┘
                           │ WebSocket / gRPC / MQTT / TCP
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    OpenClaw Gateway                         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Transport Layer                                     │   │
│  │  • WebSocket (8765)                                 │   │
│  │  • gRPC (50051)                                     │   │
│  │  • MQTT (1883)                                      │   │
│  │  • TCP (9999)                                       │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Orchestration Layer                                 │   │
│  │  • Fleet Management                                 │   │
│  │  • Service Discovery                                │   │
│  │  • Plugin Manager                                   │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Connector Layer                                     │   │
│  │  • ROS1 / ROS2                                      │   │
│  │  • Modbus / MQTT                                    │   │
│  │  • MAVLink (coming)                                 │   │
│  └─────────────────────────────────────────────────────┘   │
└──────────────────────────┬──────────────────────────────────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
        ┌─────────┐  ┌─────────┐  ┌─────────┐
        │ Robot 1 │  │ Robot 2 │  │ Robot 3 │
        │ (ROS2)  │  │ (ROS1)  │  │ (MQTT)  │
        └─────────┘  └─────────┘  └─────────┘
```

### 4.2 Transport Layer

The transport layer handles communication between AI agents and the gateway.

**Supported Transports:**

| Transport | Port | Use Case | Protocol |
|-----------|------|----------|----------|
| WebSocket | 8765 | Web dashboards, browsers | ws:// wss:// |
| gRPC | 50051 | Microservices, cloud | grpc:// |
| TCP | 9999 | Simple integrations | tcp:// |
| MQTT | 1883 | IoT, mobile robots | mqtt:// |

**Choosing a Transport:**

- **WebSocket**: Use for web-based UIs, real-time dashboards
- **gRPC**: Use for microservices, high-performance applications
- **TCP**: Use for simple, lightweight integrations
- **MQTT**: Use for IoT devices, low-bandwidth scenarios

### 4.3 Connector Layer

Connectors interface with different robot platforms.

**Available Connectors:**

| Connector | Status | Description |
|-----------|--------|-------------|
| ROS2 | ✅ Available | ROS2 Humble, Jazzy |
| ROS1 | ✅ Available | ROS1 Noetic |
| Modbus | 🚧 Planned | Industrial PLCs |
| MAVLink | 🚧 Planned | Drones, UAVs |

---

## 5. Configuration

### 5.1 Configuration Files

Configuration files are located in `config/`:

```bash
config/
├── gateway.yaml          # Main gateway configuration
├── ros2_config.yaml      # ROS2 settings
├── openclaw_config.yaml  # OpenClaw integration
├── hal_config.yaml       # Hardware abstraction
└── fault_config.yaml     # Recovery policies
```

### 5.2 Gateway Configuration (gateway.yaml)

```yaml
name: "my_robot_gateway"
log_level: INFO

transports:
  websocket:
    enabled: true
    host: 0.0.0.0
    port: 8765
    tls_cert: null        # Path to TLS certificate
    tls_key: null         # Path to TLS key
  
  grpc:
    enabled: true
    host: 0.0.0.0
    port: 50051
  
  mqtt:
    enabled: false
    broker: "localhost"
    port: 1883

connectors:
  ros2:
    enabled: true
    domain_id: 0
  
  ros1:
    enabled: false

plugins:
  - name: greenhouse
    enabled: true
    options:
      control_interval: 5

discovery:
  enabled: true
  methods:
    - mdns
    - ros2
  interval: 30

telemetry:
  enabled: true
  metrics_port: 9090
```

### 5.3 Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `OPENCLAW_LOG_LEVEL` | Logging level | INFO |
| `OPENCLAW_CONFIG` | Config file path | ./config/gateway.yaml |
| `ROS_DISTRO` | ROS distribution | auto-detect |
| `ROS_DOMAIN_ID` | ROS2 domain ID | 0 |
| `MOCK_MODE` | Enable mock mode | false |

### 5.4 Command-Line Options

```bash
openclaw-gateway [OPTIONS]

Options:
  --config PATH           Config file path
  --demo                  Run in demo mode
  --log-level LEVEL       Logging level (DEBUG/INFO/WARNING/ERROR)
  --websocket-port PORT   WebSocket port
  --grpc-port PORT        gRPC port
  --version               Show version
  --help                  Show help
```

---

## 6. Using the Gateway

### 6.1 Starting the Gateway

**Basic Start:**
```bash
openclaw-gateway
```

**With Custom Config:**
```bash
openclaw-gateway --config /path/to/config.yaml
```

**Demo Mode (with greenhouse):**
```bash
openclaw-gateway --demo
```

**Docker:**
```bash
docker run -p 8765:8765 openclaw/ros-bridge:latest
```

### 6.2 Connecting Clients

**WebSocket (JavaScript):**
```javascript
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = () => {
    ws.send(JSON.stringify({
        command: {action: 'ping'}
    }));
};

ws.onmessage = (event) => {
    console.log('Received:', event.data);
};
```

**WebSocket (Python):**
```python
import asyncio
import websockets
import json

async def client():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps({
            "command": {"action": "ping"}
        }))
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(client())
```

**gRPC (Python):**
```python
import grpc
# See API Reference for gRPC specifics
```

### 6.3 Core Commands

**Health Check:**
```json
{"command": {"action": "ping"}}
```
Response:
```json
{"status": "ok", "pong": true}
```

**Get Status:**
```json
{"command": {"action": "get_status"}}
```
Response:
```json
{
  "status": "ok",
  "ros": "jazzy",
  "mock": false,
  "handlers": ["ping", "get_status", "greenhouse.status"]
}
```

**List Handlers:**
```json
{"command": {"action": "list_handlers"}}
```
Response:
```json
{
  "status": "ok",
  "handlers": ["ping", "get_status", "discover", "greenhouse.status"]
}
```

**Discover Robots:**
```json
{"command": {"action": "discover"}}
```
Response:
```json
{
  "status": "ok",
  "robots": [
    {"uri": "ros2://192.168.1.100", "name": "warehouse_bot_1"}
  ]
}
```

---

## 7. Working with Robots

### 7.1 Connecting to a Robot

```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from openclaw_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

async def main():
    # Create gateway
    gateway = OpenClawGateway()
    
    # Register connector
    gateway.connector_registry.register(ROS2Connector())
    
    # Connect to robot
    robot = await gateway.connect_robot("ros2://192.168.1.100")
    
    # Execute command
    result = await robot.execute({
        "action": "move_to",
        "parameters": {"x": 1.0, "y": 2.0, "theta": 0.0}
    })
    
    print(f"Result: {result}")
```

### 7.2 Fleet Management

**Create a Fleet:**
```python
# Create fleet
warehouse = gateway.create_fleet("warehouse")

# Add robots
robot1 = await gateway.connect_robot(
    "ros2://192.168.1.101",
    fleet_name="warehouse"
)
robot2 = await gateway.connect_robot(
    "ros2://192.168.1.102",
    fleet_name="warehouse"
)
```

**Broadcast Commands:**
```python
# Send to all robots in fleet
await warehouse.broadcast({
    "action": "return_to_dock"
})

# Send to selected robots
await warehouse.broadcast(
    {"action": "emergency_stop"},
    selector=lambda r: r.battery < 20
)
```

### 7.3 Subscribing to Telemetry

```python
# Subscribe to sensor data
async for telemetry in robot.subscribe("/sensors"):
    print(f"Topic: {telemetry.topic}")
    print(f"Data: {telemetry.data}")
```

---

## 8. Plugin Development

### 8.1 Creating a Plugin

```python
# my_robot_plugin.py
from openclaw_ros_bridge.gateway_v2.core import Plugin, Message, Identity

class MyRobotPlugin(Plugin):
    """Custom robot control plugin"""
    
    name = "my_robot"
    version = "1.0.0"
    
    async def initialize(self, gateway):
        """Called when plugin is loaded"""
        self.gateway = gateway
        print(f"Plugin {self.name} v{self.version} initialized")
    
    async def shutdown(self):
        """Called when plugin is unloaded"""
        print(f"Plugin {self.name} shutting down")
    
    async def handle_message(self, message: Message, identity: Identity):
        """Handle incoming messages"""
        if not message.command:
            return None
        
        action = message.command.action
        
        if action == "my_robot.move":
            return await self.handle_move(message.command)
        elif action == "my_robot.status":
            return await self.handle_status()
        
        return None  # Not handled by this plugin
    
    async def handle_move(self, command):
        """Handle move command"""
        x = command.parameters.get("x", 0)
        y = command.parameters.get("y", 0)
        
        # Your robot control logic here
        
        return Message(
            header=Header(correlation_id=command.id),
            telemetry=Telemetry(
                topic="/my_robot/result",
                data={"status": "moved", "x": x, "y": y}
            )
        )
```

### 8.2 Loading the Plugin

```python
# server.py
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
from my_robot_plugin import MyRobotPlugin

async def main():
    gateway = OpenClawGateway()
    
    # Load plugin
    plugin = MyRobotPlugin()
    await gateway.plugin_manager.load_plugin(plugin)
    
    # Start gateway
    await gateway.start()

if __name__ == "__main__":
    asyncio.run(main())
```

### 8.3 Using the Plugin

```json
// From client
{
  "command": {
    "action": "my_robot.move",
    "parameters": {
      "x": 1.0,
      "y": 2.0
    }
  }
}
```

---

## 9. Troubleshooting

### 9.1 Common Issues

#### Gateway Won't Start

**Symptom:**
```
Error: Port 8765 already in use
```

**Solution:**
```bash
# Find process using port
lsof -i :8765

# Kill process
kill -9 <PID>

# Or use different port
openclaw-gateway --websocket-port 8766
```

#### Connection Refused

**Symptom:**
```
ConnectionRefusedError: [Errno 61] Connection refused
```

**Solution:**
```bash
# Check if gateway is running
curl http://localhost:8765/health

# Check logs
tail -f logs/gateway.log

# Restart gateway
openclaw-skill-ros-bridge restart
```

#### Docker Container Won't Start

**Symptom:**
```
Error response from daemon: driver failed programming external connectivity
```

**Solution:**
```bash
# Check Docker is running
docker info

# Restart Docker
docker restart

# Check port conflicts
lsof -i :8765

# Use different ports in docker-compose.yml
```

### 9.2 ROS-Specific Issues

#### ROS Topics Not Showing

**Solution:**
```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Check ROS is running
ros2 node list

# Check topics
ros2 topic list

# Verify domain ID
export ROS_DOMAIN_ID=0
```

#### Message Type Errors

**Symptom:**
```
AttributeError: 'NoneType' object has no attribute 'data'
```

**Solution:**
Ensure proper message initialization:
```python
from std_msgs.msg import String
msg = String()  # Initialize properly
msg.data = "hello"
```

### 9.3 Debug Mode

Enable debug logging:
```bash
export OPENCLAW_LOG_LEVEL=DEBUG
openclaw-gateway
```

View detailed logs:
```bash
tail -f logs/gateway.log | grep DEBUG
```

### 9.4 Getting Help

1. **Check Documentation:** https://openclaw-ros-bridge.readthedocs.io
2. **GitHub Issues:** https://github.com/webthree549-bot/openclaw-ros-bridge/issues
3. **Discussions:** https://github.com/webthree549-bot/openclaw-ros-bridge/discussions
4. **Email:** dev@openclaw-ros.org

---

## 10. API Reference

### 10.1 Core Message Format

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

### 10.2 Core Commands

| Command | Parameters | Response |
|---------|------------|----------|
| `ping` | None | `{"pong": true}` |
| `get_status` | None | Gateway status |
| `list_handlers` | None | List of handlers |
| `discover` | None | List of robots |
| `fleet.list` | None | List of fleets |
| `robot.execute` | `robot_id`, `action` | Execution result |

### 10.3 Python API

**Gateway:**
```python
class OpenClawGateway:
    async def start() -> None
    async def stop() -> None
    def create_fleet(name: str) -> RobotFleet
    async def connect_robot(uri: str, fleet: str = None) -> Robot
```

**Robot:**
```python
class Robot:
    async def execute(command: Command) -> Any
    async def subscribe(topic: str) -> AsyncIterator[Telemetry]
```

**Fleet:**
```python
class RobotFleet:
    async def broadcast(command: Command, selector: Callable = None)
    def get_robot(robot_id: str) -> Optional[Robot]
```

---

## 11. Advanced Topics

### 11.1 Security

**Enable TLS:**
```yaml
transports:
  websocket:
    enabled: true
    port: 8765
    tls_cert: /path/to/cert.pem
    tls_key: /path/to/key.pem
```

**Enable Authentication:**
```yaml
security:
  enabled: true
  authentication:
    - jwt
  jwt_secret: your-secret-key
```

### 11.2 Performance Tuning

**For High Throughput:**
```yaml
# Use gRPC instead of WebSocket
transports:
  grpc:
    enabled: true
    port: 50051
```

**For Low Latency:**
```yaml
# Use Unix sockets for local communication
transports:
  unix:
    enabled: true
    path: /tmp/openclaw.sock
```

### 11.3 Monitoring

**Enable Prometheus Metrics:**
```yaml
telemetry:
  enabled: true
  metrics_port: 9090
```

**View Metrics:**
```bash
curl http://localhost:9090/metrics
```

---

## 12. Migration from v1

### 12.1 Key Changes

| v1 | v2 | Notes |
|----|----|----|
| Single TCP server | Multi-transport | WebSocket, gRPC, MQTT |
| Single robot | Fleet management | Multiple robots |
| Static handlers | Plugin system | Dynamic loading |
| ROS-only | Universal | Multiple connectors |

### 12.2 Migration Steps

1. **Update Imports:**
   ```python
   # v1
   from openclaw_ros_bridge import openclaw_server
   
   # v2
   from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
   ```

2. **Update Server Start:**
   ```python
   # v1
   openclaw_server.start()
   
   # v2
   gateway = OpenClawGateway()
   await gateway.start()
   ```

3. **Update Plugin Registration:**
   ```python
   # v1
   openclaw_server.register_handler("cmd", handler)
   
   # v2
   await gateway.plugin_manager.load_plugin(MyPlugin())
   ```

### 12.3 Compatibility Mode

v1 code is preserved in `openclaw_ros_bridge/legacy/` for backward compatibility.

---

## Appendix A: Glossary

| Term | Definition |
|------|------------|
| **Gateway** | Central component that manages robot connections |
| **Transport** | Communication protocol (WebSocket, gRPC, etc.) |
| **Connector** | Interface to specific robot platforms |
| **Plugin** | Extension that adds custom functionality |
| **Fleet** | Group of robots managed together |
| **Handler** | Function that processes specific commands |

## Appendix B: Environment Variables Reference

| Variable | Description | Example |
|----------|-------------|---------|
| `OPENCLAW_LOG_LEVEL` | Logging level | DEBUG |
| `OPENCLAW_CONFIG` | Config file path | /etc/openclaw/gateway.yaml |
| `OPENCLAW_WEBSOCKET_PORT` | WebSocket port | 8765 |
| `OPENCLAW_GRPC_PORT` | gRPC port | 50051 |
| `ROS_DISTRO` | ROS distribution | jazzy |
| `ROS_DOMAIN_ID` | ROS2 domain | 0 |
| `MOCK_MODE` | Enable mock mode | true |

---

**Document Version:** 2.0.0  
**Last Updated:** February 13, 2026  
**Maintainer:** OpenClaw ROS Team  
**License:** MIT
