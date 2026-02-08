---
name: openclaw-ros-bridge
description: Universal ROS1/ROS2 bridge for OpenClaw AI agents to control robots and embodied intelligence systems.
homepage: https://github.com/webthree549-bot/openclaw-ros-bridge
metadata:
  {
    "openclaw":
      {
        "emoji": "🤖",
        "requires": { "bins": ["docker"] },
        "install":
          [
            {
              "id": "docker",
              "kind": "manual",
              "label": "Docker Desktop (macOS)",
              "instruction": "Install Docker Desktop from https://www.docker.com/products/docker-desktop",
            },
          ],
      },
  }
---

# OpenClaw ROS Bridge

Connect OpenClaw AI agents to ROS1/ROS2 robots and embodied intelligence systems.

## What It Does

This skill enables OpenClaw to:
- **Control robots** via ROS (Robot Operating System)
- **Read sensor data** (temperature, cameras, LiDAR, etc.)
- **Send actuator commands** (motors, arms, grippers)
- **Run in simulation** (mock mode without physical hardware)
- **Deploy to embedded systems** (Jetson, Raspberry Pi)

## Quick Start

### 1. Start ROS Bridge Container

```bash
# Start Docker container with ROS2 Jazzy
./scripts/docker_start.sh

# Or start with specific ROS version
./scripts/docker_start.sh --jazzy    # ROS2 Jazzy (default)
./scripts/docker_start.sh --humble   # ROS2 Humble
./scripts/docker_start.sh --noetic   # ROS1 Noetic

# Use the helper script (if available)
ros2-jazzy-bridge
```

### 2. Run a Demo (Mock Mode - No Hardware Required)

```bash
cd <path-to-openclaw-ros-bridge>

# With mock mode (no ROS/hardware required)
./scripts/run_demo.sh --greenhouse --mock

# With local ROS installation
./scripts/run_demo.sh --greenhouse

# With Docker container (auto-detected)
./scripts/run_demo.sh --greenhouse
```

### 3. Check ROS Topics

```bash
# Inside Docker container
ros2 topic list
ros2 topic echo /greenhouse/sensors

# Or from host using helper
ros2-jazzy-bridge ros2 topic list
```

## Available Commands

### Container Management

```bash
# Start Docker container with bridge
./scripts/docker_start.sh              # ROS2 Jazzy (default)
./scripts/docker_start.sh --jazzy      # ROS2 Jazzy
./scripts/docker_start.sh --humble     # ROS2 Humble
./scripts/docker_start.sh --noetic     # ROS1 Noetic

# Stop container
./scripts/docker_start.sh --stop

# Remove container
./scripts/docker_start.sh --rm

# Run ROS commands inside container
ros2-jazzy-bridge ros2 topic list
ros2-jazzy-bridge ros2 node list
ros2-jazzy-bridge ros2 doctor

# Check container status
docker ps | grep ros
```

### Demos

```bash
cd <path-to-openclaw-ros-bridge>

# Greenhouse (agricultural robotics)
export MOCK_MODE=true
./scripts/run_demo.sh --greenhouse

# Arm manipulation (industrial robotics)
export MOCK_MODE=true
./scripts/run_demo.sh --arm
```

### Development

```bash
cd <path-to-openclaw-ros-bridge>

# Build the project
./scripts/build.sh

# Run tests
./scripts/run_tests.sh

# Check version detection
python3 -c "
from openclaw_ros_bridge import version_manager
print(f'ROS: {version_manager.ROS_TYPE} {version_manager.ROS_DISTRO}')
print(f'OpenClaw: {version_manager.OC_VER}')
"
```

## Configuration

### Environment Variables

```bash
# ROS Version (auto-detected if not set)
export ROS_DISTRO=jazzy          # humble, jazzy, noetic
export ROS_TYPE=ros2             # ros1, ros2

# OpenClaw Version
export OPENCLAW_VERSION=v2       # v1, v2

# Operation Mode
export MOCK_MODE=true            # true = simulation, false = real hardware
export HAL_HARDWARE=auto         # auto, dht22, bme280, robotiq_2f_85, etc.
```

### Config Files

All configs are in `config/`:
- `ros2_config.yaml` - ROS2 Humble/Jazzy settings
- `openclaw_config.yaml` - TCP ports, timeouts
- `hal_config.yaml` - Hardware abstraction settings
- `fault_config.yaml` - Recovery policies

## Common Workflows

### Workflow 1: Test Without Hardware (Mock Mode)

```bash
cd <path-to-openclaw-ros-bridge>
export MOCK_MODE=true
export ROS_DISTRO=jazzy
./scripts/run_demo.sh --greenhouse

# In another terminal
ros2-jazzy-bridge ros2 topic echo /greenhouse/sensors
```

### Workflow 2: Read Sensor Data

```python
from openclaw_ros_bridge import sensor_hal, ros2_comm

# Initialize
sensor_hal.init_hardware()
ros_comm = get_ros_communicator()

# Read temperature/humidity
data = sensor_hal.read("env")
print(f"Temp: {data['temperature']}°C, Humidity: {data['humidity']}%")

# Publish to ROS topic
ros_comm.publish("/sensors/environment", SensorMsg, data)
```

### Workflow 3: Control Actuators

```python
from openclaw_ros_bridge import actuator_hal

# Initialize
actuator_hal.init_hardware()

# Turn on fan
actuator_hal.write({"fan": True})

# Open valve
actuator_hal.write({"valve": True})

# Emergency stop
actuator_hal.safe_state()
```

### Workflow 4: Create Custom Plugin

```python
# File: my_robot_plugin.py
from openclaw_ros_bridge import BasePlugin

class MyRobotPlugin(BasePlugin):
    def run(self):
        while self.is_running:
            # Read sensors
            sensor_data = self.sensor_hal.read("env")
            
            # Your AI logic here
            if sensor_data['temperature'] > 30:
                self.actuator_hal.write({"fan": True})
            
            # Send to OpenClaw
            self.send_to_openclaw(sensor_data)
            
            sleep(1.0)
```

## Docker Deployment

```bash
cd <path-to-openclaw-ros-bridge>

# Build images
./scripts/docker_build.sh

# Start all services
docker-compose -f docker/docker-compose.yml up -d

# View logs
docker-compose -f docker/docker-compose.yml logs -f

# Stop
docker-compose -f docker/docker-compose.yml down
```

## Troubleshooting

### Container Won't Start

```bash
# Check Docker is running
docker info

# Restart container
docker restart ros2-jazzy-bridge

# Check logs
docker logs ros2-jazzy-bridge
```

### ROS Topics Not Showing

```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Check nodes are running
ros2 node list

# Check topic types
ros2 topic type /greenhouse/sensors
```

### Permission Issues

```bash
# Fix script permissions
chmod +x scripts/*.sh

# Fix Docker permissions (macOS)
sudo dseditgroup -o edit -a $USER -t user docker
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    OpenClaw AI Agent                        │
└──────────────────┬──────────────────────────────────────────┘
                   │ TCP/IP JSON
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              OpenClaw ROS Bridge                            │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   ROS2      │    │   ROS1      │    │    HAL      │     │
│  │Communicator │    │Communicator │    │(Hardware    │     │
│  └─────────────┘    └─────────────┘    │ Abstraction)│     │
│                                         └─────────────┘     │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              Physical Hardware                              │
│     (Sensors, Motors, Arms, Cameras, etc.)                  │
└─────────────────────────────────────────────────────────────┘
```

## Key Files

| File | Purpose |
|------|---------|
| `scripts/openclaw_assist_greenhouse.sh` | **🤖 Interactive AI-assisted control** |
| `scripts/gh_control.sh` | **Simple command-line control** |
| `scripts/demo_openclaw_greenhouse.sh` | **🌱 Automated demo** |
| `scripts/docker_start.sh` | Start ROS Docker container |
| `scripts/run_demo.sh` | Run ROS demos |
| `scripts/start_openclaw_server.sh` | Start TCP server for OpenClaw |
| `scripts/build.sh` | Build project |
| `config/` | All configuration files |
| `demo/` | Example plugins |

## Documentation

| Document | Description |
|----------|-------------|
| `README.md` | Project overview and quick start |
| `docs/DEMO_OPENCLAW_GREENHOUSE.md` | **🌱 Run OpenClaw + Greenhouse Demo** |
| `docs/MACOS_SETUP_GUIDE.md` | **Complete macOS setup from scratch** |
| `docs/DOCKER_PORT_SETUP.md` | **Docker port mapping for macOS** |
| `docs/ARCHITECTURE.md` | Detailed architecture and design patterns |
| `docs/MACOS_DOCKER_DEPLOYMENT.md` | macOS + Docker networking details |
| `docs/VERSION_AGNOSTIC.md` | Version-agnostic design principles |

### 🌱 Quick Demo: OpenClaw Controls Greenhouse

```bash
# One command to run the complete demo
./scripts/demo_openclaw_greenhouse.sh
```

This starts:
- Docker container with ROS2
- TCP server for OpenClaw connection
- AI agent that reads sensors and controls actuators

See `docs/DEMO_OPENCLAW_GREENHOUSE.md` for details.

### 🤖 Interactive Control via OpenClaw AI

Control the greenhouse through OpenClaw AI assistance:

```bash
# Start interactive assistant
./scripts/openclaw_assist_greenhouse.sh

# Or use quick commands
./scripts/gh_control.sh status          # Check conditions
./scripts/gh_control.sh fan on          # Turn on fan
./scripts/gh_control.sh valve open      # Water plants
./scripts/gh_control.sh auto            # AI autopilot mode
```

Then tell me what to do:
- "Check the temperature"
- "Turn on the fan"
- "Water the plants"
- "Run autopilot for 5 cycles"

### macOS + Docker Port Setup

For macOS users connecting OpenClaw to Docker:

```bash
# 1. Start container with port mapping
./scripts/docker_start.sh --jazzy

# 2. Start TCP server inside container
./scripts/start_openclaw_server.sh

# 3. OpenClaw on macOS connects to: localhost:9999
```

See `docs/DOCKER_PORT_SETUP.md` for detailed instructions.

### Quick Start for macOS Users

New to the project? Start here:

```bash
# 1. Read the complete setup guide
cat docs/MACOS_SETUP_GUIDE.md

# 2. Follow the 8-step setup process
#    (Docker → OpenClaw → Build → Test → Run)
```

## Tips

1. **Always use mock mode for testing** - No hardware required
2. **Check version detection first** - Run version_manager check
3. **Use Docker for isolation** - Avoid system ROS conflicts
4. **Read the logs** - Most issues are in `logs/` directory
5. **Start simple** - Get greenhouse demo working first
