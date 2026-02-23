# macOS Installation Guide

Agent ROS Bridge supports macOS via multiple installation methods. Choose the one that fits your use case.

## Quick Reference

| Method | ROS Support | Difficulty | Best For |
|--------|-------------|------------|----------|
| **Homebrew** | ❌ Client only | ⭐ Easy | AI agents connecting to remote ROS |
| **Docker** | ✅ Full | ⭐⭐ Medium | Complete isolated environment |
| **RoboStack** | ✅ Full | ⭐⭐⭐ Hard | Native ROS2 on macOS |
| **Direct pip** | ❌ Client only | ⭐ Easy | Development/testing |

---

## Method 1: Homebrew (Recommended for Client Mode)

For running the bridge as an AI agent client (connecting to remote ROS robots).

```bash
# Add OpenClaw tap
brew tap openclaw/tap

# Install Agent ROS Bridge
brew install agent-ros-bridge

# With all features
brew install agent-ros-bridge --with-all

# Start as service
brew services start agent-ros-bridge

# View logs
brew services info agent-ros-bridge
tail -f /opt/homebrew/var/log/agent-ros-bridge.log
```

**Note:** Homebrew installation does not include ROS. Use this for:
- Claude Desktop integration
- Connecting to remote ROS robots
- Development without local ROS

---

## Method 2: Docker Desktop (Recommended for Full ROS)

For running the complete bridge + ROS environment on macOS.

### Prerequisites
- [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop)
- At least 8GB RAM allocated to Docker

### Installation

```bash
# Clone repository
git clone https://github.com/openclaw/agent-ros-bridge.git
cd agent-ros-bridge

# Start with Docker Compose
docker-compose -f docker-compose.macos.yml up -d

# View logs
docker-compose -f docker-compose.macos.yml logs -f bridge

# Stop
docker-compose -f docker-compose.macos.yml down
```

### With Demo Dashboard

```bash
# Start bridge + demo dashboard
docker-compose -f docker-compose.macos.yml --profile demo up -d

# Open dashboard
open http://localhost:8773
```

### With ROS2

```bash
# Start bridge + ROS2 Jazzy
docker-compose -f docker-compose.macos.yml --profile ros up -d

# Enter ROS2 container
docker exec -it ros2-jazzy bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

---

## Method 3: RoboStack (Native ROS2 on macOS)

For native ROS2 support without Docker.

### Prerequisites
- [Miniforge](https://github.com/conda-forge/miniforge) (Conda for macOS)

### Installation

```bash
# Create conda environment
conda create -n ros_env python=3.10 -y
conda activate ros_env

# Install RoboStack ROS2
conda install -c conda-forge -c robostack ros-humble-desktop -y

# Activate ROS2
source $(conda info --base)/etc/profile.d/conda.sh
conda activate ros_env

# Install Agent ROS Bridge
pip install agent-ros-bridge[all]

# Run
python -m agent_ros_bridge.mcp
```

### Additional ROS2 Packages

```bash
# Navigation2
conda install -c conda-forge -c robostack ros-humble-navigation2

# MoveIt
conda install -c conda-forge -c robostack ros-humble-moveit

# Gazebo
conda install -c conda-forge -c robostack ros-humble-gazebo-ros
```

### Limitations

RoboStack on macOS has some limitations:
- No GUI tools (RViz, Gazebo GUI) — use web-based alternatives
- Some packages may not be available
- Performance may be slower than Linux

---

## Method 4: Direct pip (Client Only)

Simplest method for AI agent development.

```bash
# Install via pip
pip install agent-ros-bridge[mcp,websocket,grpc]

# Configure for remote ROS
export ROS_MASTER_URI=http://robot.local:11311  # ROS1
# or connect via rosbridge_server WebSocket for ROS2

# Run
python -m agent_ros_bridge.mcp
```

---

## Claude Desktop Configuration (macOS)

Edit the Claude Desktop config file:

```bash
# Open config
open ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

Add the ROS bridge:

```json
{
  "mcpServers": {
    "ros": {
      "command": "python3",
      "args": ["-m", "agent_ros_bridge.mcp"],
      "env": {
        "JWT_SECRET": "your-secret-key"
      }
    }
  }
}
```

If using Homebrew:

```json
{
  "mcpServers": {
    "ros": {
      "command": "/opt/homebrew/bin/agent-ros-bridge"
    }
  }
}
```

---

## Troubleshooting

### Port Already in Use

```bash
# Find process using port 8765
lsof -i :8765

# Kill process
kill -9 <PID>
```

### Docker Permission Denied

```bash
# Add user to docker group (requires logout/login)
sudo usermod -aG docker $USER
```

### RoboStack Import Errors

```bash
# Ensure conda environment is activated
conda activate ros_env

# Reinstall if needed
conda install -c conda-forge -c robostack ros-humble-desktop --force-reinstall
```

### Homebrew Service Not Starting

```bash
# Check logs
tail -f /opt/homebrew/var/log/agent-ros-bridge.log

# Restart service
brew services restart agent-ros-bridge
```

---

## Platform-Specific Notes

### Apple Silicon (M1/M2/M3)

All methods work on Apple Silicon:

- **Docker**: Automatically uses arm64 images (faster)
- **Homebrew**: Native arm64 binaries
- **RoboStack**: arm64 packages available

### Intel Macs

All methods supported. Rosetta 2 not required.

---

## Getting Help

- [GitHub Issues](https://github.com/openclaw/agent-ros-bridge/issues)
- [Discord Community](https://discord.gg/agent-ros-bridge)
- [RoboStack Documentation](https://robostack.github.io/)
- [ROS2 on macOS Guide](http://wiki.ros.org/ROS2/Installation/macOS)

---

## Next Steps

- [Quick Start Guide](../README.md#quick-start)
- [Configuration Reference](../config/bridge.yaml.example)
- [OpenClaw Integration](../README.md#openclaw-integration-privileged)
