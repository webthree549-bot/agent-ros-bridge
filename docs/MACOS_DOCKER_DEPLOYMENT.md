# OpenClaw + ROS Bridge Deployment Architecture

## Overview

When OpenClaw runs on **macOS** and ROS2 + OpenClaw ROS Bridge runs in **Docker**, they communicate over **TCP/IP networking**.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              macOS Host                                       │
│  ┌─────────────────────────────────┐    ┌─────────────────────────────────┐  │
│  │       OpenClaw AI Agent         │    │         Docker Desktop          │  │
│  │         (Native macOS)          │    │                                 │  │
│  │                                 │    │  ┌───────────────────────────┐  │  │
│  │  ┌─────────────────────────┐    │    │  │  ROS2 Jazzy Container     │  │  │
│  │  │   OpenClaw Client       │◄───┼────┼──┼─►┌─────────────────────┐  │  │  │
│  │  │   (TCP Socket)          │    │    │  │  │ OpenClaw ROS Bridge │  │  │  │
│  │  └─────────────────────────┘    │    │  │  │   (TCP Server)      │  │  │  │
│  │                                 │    │  │  └─────────────────────┘  │  │  │
│  └─────────────────────────────────┘    │  │           │                 │  │  │
│                                         │  │           ▼                 │  │  │
│                                         │  │  ┌─────────────────────┐    │  │  │
│                                         │  │  │   ROS2 Nodes        │    │  │  │
│                                         │  │  │   (Topics/Services) │    │  │  │
│                                         │  │  └─────────────────────┘    │  │  │
│                                         │  │           │                 │  │  │
│                                         │  │           ▼                 │  │  │
│                                         │  │  ┌─────────────────────┐    │  │  │
│                                         │  │  │  HAL / Hardware     │    │  │  │
│                                         │  │  │  (Mock or Real)     │    │  │  │
│                                         │  │  └─────────────────────┘    │  │  │
│                                         │  └───────────────────────────┘  │  │
│                                         └─────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘

                        Network: TCP/IP (localhost/bridge)
```

---

## How It Works

### 1. Network Communication

OpenClaw (macOS) connects to OpenClaw ROS Bridge (Docker) via **TCP socket**:

```python
# OpenClaw side (macOS)
import socket

# Connect to bridge in Docker
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("localhost", 9999))  # Default OpenClaw v2 port

# Send command
sock.send(json.dumps({
    "action": "read_sensor",
    "sensor": "temperature"
}).encode())

# Receive response
response = sock.recv(1024)
```

### 2. Docker Networking Modes

#### Option A: Host Network (Linux only, NOT macOS)
```bash
# This works on Linux but NOT macOS
docker run --network host ros2-jazzy-bridge
```

#### Option B: Port Mapping (macOS compatible)
```bash
# Map container port to host port
docker run -p 9999:9999 ros2-jazzy-bridge

# Now macOS can connect to localhost:9999
```

#### Option C: Docker Bridge Network (Recommended)
```bash
# Create custom bridge network
docker network create openclaw-ros

# Run container in network
docker run --network openclaw-ros --name ros-bridge ros2-jazzy-bridge

# Get container IP
docker inspect ros-bridge | grep IPAddress
# Returns: 172.18.0.2
```

### 3. Configuration

**On macOS (OpenClaw):**
```bash
# .env file or environment
export OPENCLAW_BRIDGE_HOST=localhost  # or Docker IP
export OPENCLAW_BRIDGE_PORT=9999
export OPENCLAW_VERSION=v2
```

**In Docker (ROS Bridge):**
```bash
# config/openclaw_config.yaml
openclaw_versions:
  v2:
    tcp_host: "0.0.0.0"  # Listen on all interfaces
    tcp_port: 9999
    recv_buffer_size: 8192
    send_timeout: 10
    recv_timeout: 10
```

---

## Connection Flow

```
1. Start Docker Container
   $ ./scripts/docker_start.sh

2. Docker container starts ROS2 + Bridge
   - ROS2 daemon initializes
   - OpenClaw TCP server starts on port 9999
   - Port 9999 mapped to macOS localhost:9999

3. OpenClaw on macOS connects
   $ openclaw-agent
   - Connects to localhost:9999
   - Handshake with OpenClaw ROS Bridge

4. Communication established
   ┌─────────────┐         JSON/TCP         ┌─────────────┐
   │   OpenClaw  │ ◄──────────────────────► │    Bridge   │
   │    macOS    │    {cmd: "read_sensor"}  │   Docker    │
   └─────────────┘                          └──────┬──────┘
                                                    │
                                                    ▼
                                            ┌─────────────┐
                                            │   ROS2      │
                                            │  Topics     │
                                            └─────────────┘
```

---

## Real-World Example

### Terminal 1: Start Docker Container
```bash
cd openclaw-ros-bridge

# Start container with port mapping
./scripts/docker_start.sh --jazzy

# Inside container, verify bridge is listening
root@ros-bridge:/app# netstat -tlnp | grep 9999
tcp  0  0 0.0.0.0:9999  0.0.0.0:*  LISTEN  1234/python3
```

### Terminal 2: Test from macOS
```bash
# Test TCP connection from macOS
nc -vz localhost 9999
# Connection to localhost port 9999 [tcp/*] succeeded!

# Send test command
echo '{"cmd": "get_status"}' | nc localhost 9999
# {"status": "ok", "ros": "jazzy", "mock": true}
```

### Terminal 3: Run OpenClaw Agent (macOS)
```python
# openclaw_agent.py (runs on macOS)
from openclaw_ros_bridge import OpenClawClient

client = OpenClawClient(host="localhost", port=9999)
client.connect()

# Read sensor data
response = client.send_command({
    "action": "read_sensor",
    "sensor": "temperature"
})
print(response)  # {"temperature": 25.0, "humidity": 50.0}

# Control actuator
client.send_command({
    "action": "write_actuator",
    "actuator": "fan",
    "value": True
})
```

---

## Docker Networking on macOS

### The Challenge
macOS Docker uses a **virtual machine** (not native Linux kernel), so `--network host` doesn't work.

### The Solution
Use **port mapping** (`-p` flag) or **docker-compose networking**.

### docker-compose.yml
```yaml
version: '3.8'
services:
  ros-bridge:
    build: .
    container_name: ros2-jazzy-bridge
    ports:
      - "9999:9999"    # OpenClaw TCP
      - "9090:9090"    # rosbridge websocket (optional)
    environment:
      - ROS_DISTRO=jazzy
      - MOCK_MODE=true
    volumes:
      - .:/app
    networks:
      - openclaw-net

  # Optional: Web-based ROS tools
  ros-tools:
    image: osrf/ros:jazzy-desktop
    ports:
      - "8080:8080"
    depends_on:
      - ros-bridge

networks:
  openclaw-net:
    driver: bridge
```

Run with:
```bash
docker-compose up -d
# Services available:
# - localhost:9999 → OpenClaw Bridge
# - localhost:9090 → rosbridge (for web UIs)
```

---

## Network Diagram

```
┌────────────────────────────────────────────────────────────────┐
│                        macOS Host                               │
│                                                                 │
│  ┌──────────────────┐        ┌──────────────────────────────┐  │
│  │  OpenClaw Agent  │        │      Docker VM (Linux)       │  │
│  │                  │        │                              │  │
│  │  TCP Client      │◄──────►│  ┌────────────────────────┐  │  │
│  │  localhost:9999  │        │  │  ros2-jazzy-bridge     │  │  │
│  └──────────────────┘        │  │                        │  │  │
│                              │  │  TCP Server:9999 ◄─────┼──┼──┤
│  ┌──────────────────┐        │  │       │                │  │  │
│  │  Web Browser     │        │  │       ▼                │  │  │
│  │  localhost:8080  │◄───────┼──┼─► ROS2 Web Bridge     │  │  │
│  └──────────────────┘        │  │       │                │  │  │
│                              │  │       ▼                │  │  │
│  ┌──────────────────┐        │  │  ┌─────────────┐       │  │  │
│  │  RViz/CLI Tools  │        │  │  │ ROS2 Topics │       │  │  │
│  │  (ros2 topic)    │◄───────┼──┼─►│ /sensor/... │       │  │  │
│  └──────────────────┘        │  │  └─────────────┘       │  │  │
│                              │  └────────────────────────┘  │  │
│                              └──────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘

              Network Flow:
              1. macOS app → localhost:9999 → Docker bridge
              2. Bridge processes → ROS2 topics
              3. Sensors/actuators ↔ HAL ↔ ROS2
```

---

## Troubleshooting

### Connection Refused
```bash
# Check if port is mapped
docker ps
# PORTS column should show: 0.0.0.0:9999->9999/tcp

# Check if server is listening inside container
docker exec ros2-jazzy-bridge netstat -tlnp | grep 9999

# Check firewall
sudo lsof -i :9999
```

### Wrong IP Address
```bash
# Get Docker container IP
docker inspect ros2-jazzy-bridge | jq -r '.[0].NetworkSettings.IPAddress'
# Use this IP from other Docker containers
# Use 'localhost' from macOS host
```

### Test Connectivity
```bash
# From macOS, test TCP connection
nc -vz localhost 9999

# Send test message
echo '{"ping": true}' | nc localhost 9999
```

---

## Summary

| Component | Location | Network Role |
|-----------|----------|--------------|
| OpenClaw AI | macOS | TCP Client |
| ROS2 + Bridge | Docker | TCP Server + ROS2 |
| Communication | localhost:9999 | JSON over TCP |
| Hardware | Docker or external | ROS2 topics |

The key insight: **TCP is the bridge** between macOS and Docker. ROS2 runs entirely inside Docker, while OpenClaw on macOS connects via standard TCP sockets.
