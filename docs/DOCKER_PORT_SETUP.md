# Docker Port Setup for macOS + OpenClaw

This guide explains how to expose Docker ports so OpenClaw on macOS can communicate with the ROS Bridge running in Docker.

## The Problem

On **macOS**, Docker runs inside a virtual machine. The `--network host` option (which works on Linux) **does not work** on macOS. Instead, we need to use **port mapping** (`-p` flag).

## Port Mapping Solution

### Ports Used

| Port | Purpose | Protocol |
|------|---------|----------|
| **9999** | OpenClaw TCP (v2) | TCP |
| **8888** | OpenClaw TCP (v1) | TCP |
| **9090** | rosbridge WebSocket | WebSocket |

### Method 1: Using docker_start.sh (Automatic)

The `docker_start.sh` script now automatically detects macOS and uses port mapping:

```bash
# On macOS - ports are automatically mapped
./scripts/docker_start.sh --jazzy

# Output shows:
# [DOCKER] Detected macOS - using port mapping for OpenClaw connection
```

This maps:
- Container port 9999 → macOS localhost:9999
- Container port 9090 → macOS localhost:9090

### Method 2: Using Docker Compose

```bash
# Start with docker-compose (includes port mapping)
docker-compose -f docker/docker-compose.yml up ros2-jazzy-bridge

# Ports are defined in docker-compose.yml:
# - "9999:9999"
# - "8888:8888"  
# - "9090:9090"
```

### Method 3: Manual Docker Run

```bash
# Run with explicit port mapping
docker run -it \
    --name ros2-jazzy-bridge \
    -p 9999:9999 \
    -p 8888:8888 \
    -p 9090:9090 \
    -v "$PWD:/app" \
    ros:jazzy-ros-base \
    bash
```

## Testing the Connection

### Step 1: Start Docker Container with Ports

```bash
./scripts/docker_start.sh --jazzy

# Inside container, start the TCP server
cd /app
./scripts/start_openclaw_server.sh

# Or as ROS2 node:
ros2 run openclaw_ros_bridge openclaw_tcp_server
```

### Step 2: Test from macOS Host

```bash
# On macOS (outside Docker), test TCP connection
nc -vz localhost 9999
# Connection to localhost port 9999 [tcp/*] succeeded!

# Send test command
echo '{"action": "ping"}' | nc localhost 9999
# {"status": "ok", "pong": true}

# Get status
echo '{"action": "get_status"}' | nc localhost 9999
# {"status": "ok", "ros": "jazzy", "mock": true}

# Read sensor
echo '{"action": "read_sensor", "sensor": "env"}' | nc localhost 9999
# {"status": "ok", "data": {"temperature": 25.0, "humidity": 50.0}}
```

### Step 3: Connect OpenClaw Agent

```python
# openclaw_agent.py (runs on macOS)
import socket
import json

class OpenClawClient:
    def __init__(self, host="localhost", port=9999):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def connect(self):
        self.sock.connect((self.host, self.port))
        print(f"Connected to ROS Bridge at {self.host}:{self.port}")
    
    def send(self, cmd):
        self.sock.send(json.dumps(cmd).encode() + b'\n')
        return json.loads(self.sock.recv(4096).decode())
    
    def read_sensor(self):
        return self.send({"action": "read_sensor", "sensor": "env"})
    
    def control_fan(self, on):
        return self.send({"action": "write_actuator", "actuator": "fan", "value": on})

# Use it
client = OpenClawClient("localhost", 9999)  # Docker exposed port
client.connect()
print(client.read_sensor())
client.control_fan(True)
```

## Network Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         macOS Host                               │
│  ┌─────────────────────────┐        ┌─────────────────────────┐ │
│  │    OpenClaw Agent       │        │   Docker Desktop VM     │ │
│  │    (Python/Node)        │        │                         │ │
│  │                         │        │  ┌───────────────────┐  │ │
│  │  TCP Client             │◄──────►│  │ ROS2 Jazzy        │  │ │
│  │  localhost:9999         │        │  │                   │  │ │
│  │                         │        │  │ ┌───────────────┐ │  │ │
│  │  socket.connect()       │        │  │ │ TCP Server    │ │  │ │
│  │  (to Docker port)       │        │  │ │ 0.0.0.0:9999  │ │  │ │
│  │                         │        │  │ └───────────────┘ │  │ │
│  │                         │        │  │       │           │  │ │
│  └─────────────────────────┘        │  │       ▼           │  │ │
│                                     │  │ ┌───────────────┐ │  │ │
│                                     │  │ │ ROS2 Topics   │ │  │ │
│                                     │  │ └───────────────┘ │  │ │
│                                     │  └───────────────────┘  │ │
│                                     └─────────────────────────┘ │
│                                                 │               │
│                                                 │ Docker port   │
│                                                 │ mapping       │
└─────────────────────────────────────────────────────────────────┘
```

## Troubleshooting

### Connection Refused

```bash
# Check if port is mapped
docker ps
# PORTS column should show: 0.0.0.0:9999->9999/tcp

# Check if server is listening inside container
docker exec ros2-jazzy-bridge netstat -tlnp | grep 9999

# Should show: tcp 0 0 0.0.0.0:9999 0.0.0.0:* LISTEN
```

### Server Not Starting

```bash
# Inside container, test server manually
python3 -c "
from openclaw_ros_bridge.communication.openclaw_tcp_server import main
main()
"

# Check for errors
```

### Port Already in Use

```bash
# Find process using port
lsof -i :9999

# Kill it if needed
kill -9 <PID>
```

### Firewall Blocking

```bash
# macOS - check firewall
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --getglobalstate

# Temporarily disable (not recommended for long term)
sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate off
```

## Configuration

### Change Default Port

Edit `config/openclaw_config.yaml`:

```yaml
openclaw_versions:
  v2:
    tcp_host: "0.0.0.0"  # Listen on all interfaces (required for Docker)
    tcp_port: 9999        # Change this if needed
```

Then update Docker port mapping:

```bash
# Use new port
docker run -p 9998:9998 ...
```

## Quick Start (macOS)

```bash
# 1. Start Docker container with port mapping
./scripts/docker_start.sh --jazzy

# 2. Inside container, start TCP server
./scripts/start_openclaw_server.sh

# 3. On macOS, test connection
nc -vz localhost 9999

# 4. Run OpenClaw agent (connects to localhost:9999)
python3 openclaw_agent.py
```

## Summary

| Platform | Network Mode | Command |
|----------|--------------|---------|
| **macOS** | Port mapping (`-p`) | `./scripts/docker_start.sh` |
| **Linux** | Host network (`--network host`) | `./scripts/docker_start.sh` |

On macOS, always use **port mapping** - the script handles this automatically!
