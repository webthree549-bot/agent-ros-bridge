# macOS Setup Guide: OpenClaw + ROS Docker

Complete step-by-step guide for macOS users to set up OpenClaw with ROS2 Docker from scratch.

---

## Prerequisites

- **macOS 12+** (Monterey or later)
- **Apple Silicon (M1/M2/M3)** or Intel Mac
- **Homebrew** installed
- **~10GB free disk space**

---

## Step 1: Install Docker Desktop

### Option A: Official Docker Desktop (Recommended)

```bash
# Install via Homebrew
brew install --cask docker

# Or download from: https://www.docker.com/products/docker-desktop
```

### Option B: OrbStack (Faster, Lighter)

```bash
brew install --cask orbstack
```

### Start Docker

```bash
# Open Docker Desktop from Applications
open -a Docker

# Or if using OrbStack
open -a OrbStack

# Verify installation
docker --version
docker-compose --version
```

---

## Step 2: Install OpenClaw

### Install OpenClaw CLI

```bash
# Clone OpenClaw (if not already)
git clone https://github.com/openclaw/openclaw.git
cd openclaw

# Install
npm install -g openclaw

# Verify
openclaw --version
```

### Configure OpenClaw

```bash
# Create config directory
mkdir -p ~/.openclaw

# Edit config
openclaw config

# Add this to ~/.openclaw/config.yaml:
gateway:
  url: ws://localhost:7777
  token: your-token-here

agents:
  main:
    model: moonshot/kimi-k2.5
    workspace: ~/.openclaw/workspace
```

---

## Step 3: Install OpenClaw ROS Bridge Skill

### Install via OpenClaw CLI

```bash
# Install the ROS bridge skill
openclaw skills install openclaw-ros-bridge

# Or manually clone
git clone https://github.com/webthree549-bot/openclaw-ros-bridge.git ~/.openclaw/skills/openclaw-ros-bridge
```

### Set up Workspace

```bash
# Create workspace directory
mkdir -p ~/.openclaw/workspace/ros-bridge
cd ~/.openclaw/workspace/ros-bridge

# Copy skill files (if manually installed)
cp -r ~/.openclaw/skills/openclaw-ros-bridge/* .

# Or use the skill directly
openclaw skills run openclaw-ros-bridge
```

---

## Step 4: Build ROS Docker Image

### Navigate to Project

```bash
cd ~/.openclaw/workspace/ros-bridge
# or wherever you cloned the project
```

### Build Docker Image

```bash
# Build the ROS2 Jazzy image
./scripts/docker_build.sh

# This will take 5-10 minutes the first time
```

### Alternative: Use Pre-built Image

```bash
# If available, pull pre-built image
docker pull openclaw/ros2-jazzy-bridge:latest
```

---

## Step 5: Start ROS Container

### Start Container

```bash
# Start Docker container with ROS2 Jazzy
./scripts/docker_start.sh

# Or specify ROS version
./scripts/docker_start.sh --jazzy    # ROS2 Jazzy (default)
./scripts/docker_start.sh --humble   # ROS2 Humble
./scripts/docker_start.sh --noetic   # ROS1 Noetic
```

### Verify Container is Running

```bash
# Check container status
docker ps

# Expected output:
# CONTAINER ID   IMAGE                    NAMES
# abc123         ros2-jazzy-bridge       ros2-jazzy-bridge

# View logs
docker logs ros2-jazzy-bridge
```

---

## Step 6: Test OpenClaw + ROS Connection

### Terminal 1: Enter Container

```bash
# Attach to running container
docker exec -it ros2-jazzy-bridge bash

# Inside container, check ROS2
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Expected: Empty or minimal topics
```

### Terminal 2: Run Demo from macOS

```bash
# On macOS host
cd ~/.openclaw/workspace/ros-bridge

# Run greenhouse demo in mock mode
./scripts/run_demo.sh --greenhouse --mock
```

### Terminal 3: Verify Topics

```bash
# In container (Terminal 1)
ros2 topic list

# Expected output:
# /greenhouse/actuator/fan
# /greenhouse/actuator/valve
# /greenhouse/sensor/environment
# /parameter_events
# /rosout

# Echo sensor data
ros2 topic echo /greenhouse/sensor/environment
```

---

## Step 7: OpenClaw Agent Integration

### Create OpenClaw Agent Script

```bash
# Create agent script
cat > ~/.openclaw/workspace/ros-bridge/openclaw_ros_agent.py << 'EOF'
#!/usr/bin/env python3
"""OpenClaw Agent for ROS Bridge"""
import json
import socket
import time

class OpenClawROSAgent:
    def __init__(self, host="localhost", port=9999):
        self.host = host
        self.port = port
        self.socket = None
    
    def connect(self):
        """Connect to OpenClaw ROS Bridge"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"Connected to {self.host}:{self.port}")
    
    def send_command(self, command):
        """Send command to bridge"""
        self.socket.send(json.dumps(command).encode() + b'\n')
        response = self.socket.recv(4096).decode()
        return json.loads(response)
    
    def read_sensor(self, sensor_type="env"):
        """Read sensor data"""
        return self.send_command({
            "action": "read_sensor",
            "sensor": sensor_type
        })
    
    def control_actuator(self, actuator, value):
        """Control actuator"""
        return self.send_command({
            "action": "write_actuator",
            "actuator": actuator,
            "value": value
        })
    
    def run(self):
        """Main agent loop"""
        self.connect()
        
        try:
            while True:
                # Read sensors
                data = self.read_sensor("env")
                print(f"Temperature: {data.get('temperature')}°C")
                print(f"Humidity: {data.get('humidity')}%")
                
                # AI decision logic
                if data.get('temperature', 0) > 28:
                    self.control_actuator("fan", True)
                    print("Fan: ON")
                else:
                    self.control_actuator("fan", False)
                    print("Fan: OFF")
                
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.socket.close()

if __name__ == "__main__":
    agent = OpenClawROSAgent()
    agent.run()
EOF

chmod +x ~/.openclaw/workspace/ros-bridge/openclaw_ros_agent.py
```

### Run Agent

```bash
cd ~/.openclaw/workspace/ros-bridge

# In one terminal: start demo
./scripts/run_demo.sh --greenhouse --mock

# In another terminal: run agent
python3 openclaw_ros_agent.py
```

---

## Step 8: Verify Full Integration

### Check Data Flow

```
┌──────────────┐     TCP/JSON      ┌──────────────┐     ROS2      ┌──────────┐
│   OpenClaw   │ ────────────────► │     ROS      │ ────────────► │   HAL    │
│    Agent     │                   │    Bridge    │               │ (Mock)   │
│   (macOS)    │ ◄──────────────── │   (Docker)   │ ◄──────────── │          │
└──────────────┘                   └──────────────┘               └──────────┘
     │                                   │
     │                                   │
     ▼                                   ▼
 Read: {"temp": 25}              Publish: /greenhouse/sensors
 Write: {"fan": true}             Subscribe: /greenhouse/actuators
```

### Test Commands

```bash
# In Docker container
docker exec -it ros2-jazzy-bridge bash

# Source ROS
source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

# Monitor topics
ros2 topic list
ros2 topic hz /greenhouse/sensor/environment
ros2 topic echo /greenhouse/actuator/fan

# Check nodes
ros2 node list
```

---

## Common Issues

### Issue 1: Docker Not Starting

```bash
# Check Docker daemon
docker info

# Restart Docker
killall Docker
open -a Docker

# Or reset Docker
# Docker Desktop → Troubleshoot → Reset to factory defaults
```

### Issue 2: Container Can't Start

```bash
# Check port conflicts
lsof -i :9999

# Stop existing container
docker stop ros2-jazzy-bridge
docker rm ros2-jazzy-bridge

# Restart
./scripts/docker_start.sh
```

### Issue 3: Connection Refused

```bash
# Check if bridge is listening inside container
docker exec ros2-jazzy-bridge netstat -tlnp | grep 9999

# Check port mapping
docker port ros2-jazzy-bridge

# Should show: 9999/tcp -> 0.0.0.0:9999
```

### Issue 4: Permission Denied

```bash
# Fix script permissions
chmod +x scripts/*.sh

# Fix Docker socket (macOS)
sudo chown $USER /var/run/docker.sock
```

### Issue 5: Build Fails

```bash
# Clean and rebuild
rm -rf build install log
./scripts/build.sh

# If colcon not found, install it
pip3 install colcon-common-extensions
```

---

## Next Steps

### 1. Try Different Demos

```bash
# Greenhouse demo
./scripts/run_demo.sh --greenhouse --mock

# Arm manipulation demo
./scripts/run_demo.sh --arm --mock

# Without mock (requires hardware)
./scripts/run_demo.sh --greenhouse
```

### 2. Run Tests

```bash
./scripts/run_tests.sh
```

### 3. Create Custom Plugin

```python
# my_robot_plugin.py
from openclaw_ros_bridge import BasePlugin

class MyRobotPlugin(BasePlugin):
    def run(self):
        while self.is_running:
            data = self.sensor_hal.read("env")
            # Your logic here
            time.sleep(1.0)
```

### 4. Deploy to Hardware

```bash
# When ready for real hardware
export MOCK_MODE=false
export HAL_HARDWARE=dht22  # or your sensor
./scripts/run_demo.sh --greenhouse
```

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `docker ps` | Check running containers |
| `docker logs ros2-jazzy-bridge` | View container logs |
| `docker exec -it ros2-jazzy-bridge bash` | Enter container shell |
| `./scripts/docker_start.sh` | Start ROS container |
| `./scripts/docker_start.sh --stop` | Stop container |
| `./scripts/run_demo.sh --greenhouse --mock` | Run demo |
| `ros2 topic list` | List ROS topics |
| `ros2 node list` | List ROS nodes |

---

## Summary

You now have:
- ✅ Docker Desktop running on macOS
- ✅ OpenClaw CLI installed and configured
- ✅ OpenClaw ROS Bridge skill installed
- ✅ ROS2 Jazzy container running
- ✅ Demo working in mock mode
- ✅ Custom agent script for AI integration

**Next:** Connect your OpenClaw AI agent to control robots via ROS!
