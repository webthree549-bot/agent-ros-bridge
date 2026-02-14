# Native ROS Installation Guide

This guide covers installing and running Agent ROS Bridge on native Linux (Ubuntu) with ROS1 or ROS2.

## Supported Platforms

- **Ubuntu 20.04** (Focal) + ROS1 Noetic / ROS2 Foxy
- **Ubuntu 22.04** (Jammy) + ROS1 Noetic / ROS2 Humble
- **Ubuntu 24.04** (Noble) + ROS2 Jazzy

## Prerequisites

### 1. Install ROS

**ROS1 Noetic (Ubuntu 20.04):**
```bash
# Follow official ROS1 installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

**ROS2 Humble (Ubuntu 22.04):**
```bash
# Follow official ROS2 installation
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
```

**ROS2 Jazzy (Ubuntu 24.04):**
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Source ROS Environment

Add to your `.bashrc`:

**ROS1:**
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**ROS2:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Python Dependencies

```bash
# Install pip if not present
sudo apt install python3-pip python3-venv

# Create virtual environment (recommended)
python3 -m venv ~/agent-ros-bridge-venv
source ~/agent-ros-bridge-venv/bin/activate

# Install agent-ros-bridge
pip install agent-ros-bridge

# Or install from source
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[dev]"
```

## Running the Bridge

### With Auto-Detection

The bridge auto-detects ROS version from environment:

```bash
# ROS1
source /opt/ros/noetic/setup.bash
python run_bridge.py

# ROS2
source /opt/ros/humble/setup.bash
python run_bridge.py
```

### With Configuration File

Create `config/bridge.yaml`:

```yaml
bridge:
  name: "my_robot_bridge"
  
  transports:
    websocket:
      port: 8765
      host: "0.0.0.0"
  
  connectors:
    ros:
      auto_detect: true
      endpoints: []
```

Run:
```bash
python run_bridge.py
# Or with explicit config:
BRIDGE_CONFIG=config/bridge.yaml python run_bridge.py
```

### With Specific ROS Endpoint

```yaml
bridge:
  connectors:
    ros:
      auto_detect: false
      endpoints:
        - id: "turtlebot_01"
          ros_type: "ros2"
          ros_distro: "humble"
          host: "localhost"
          domain_id: 0
```

## Testing with Real Robot

### TurtleBot4 Example

1. **Start TurtleBot4:**
```bash
# On robot or simulation
ros2 launch turtlebot4_bringup robot.launch.py
```

2. **Start Bridge:**
```bash
source /opt/ros/humble/setup.bash
python run_bridge.py
```

3. **Connect from Web Dashboard:**
```bash
python dashboard/server.py
# Open http://localhost:8080
```

4. **Or use CLI:**
```bash
wscat -c ws://localhost:8765
> {"command": {"action": "list_robots"}}
```

### UR Robot Example

1. **Start UR ROS Driver:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.100
```

2. **Start Bridge:**
```bash
python run_bridge.py
```

## Troubleshooting

### Issue: "No ROS detected"

**Solution:** Source ROS setup file
```bash
source /opt/ros/YOUR_ROS_DISTRO/setup.bash
```

### Issue: "Cannot find rclpy/rospy"

**Solution:** Install ROS Python packages
```bash
# ROS2
sudo apt install python3-rclpy

# ROS1
sudo apt install python3-rospy
```

### Issue: Permission denied on WebSocket port

**Solution:** Use port > 1024 or run with sudo
```bash
# Use higher port
python run_bridge.py --port 8765
```

### Issue: ROS nodes not visible

**Solution:** Check ROS_DOMAIN_ID and network
```bash
# ROS2 - ensure same domain
export ROS_DOMAIN_ID=0

# Check nodes
ros2 node list
```

## Systemd Service (Production)

Create `/etc/systemd/system/agent-ros-bridge.service`:

```ini
[Unit]
Description=Agent ROS Bridge
After=network.target

[Service]
Type=simple
User=robot
Environment="ROS_DISTRO=humble"
Environment="ROS_DOMAIN_ID=0"
WorkingDirectory=/home/robot/agent-ros-bridge
ExecStart=/home/robot/agent-ros-bridge-venv/bin/python run_bridge.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable agent-ros-bridge
sudo systemctl start agent-ros-bridge
sudo systemctl status agent-ros-bridge
```

View logs:
```bash
sudo journalctl -u agent-ros-bridge -f
```

## Performance Tuning

### For High-Frequency Telemetry

```yaml
bridge:
  telemetry:
    enabled: true
    batch_size: 100
    flush_interval_ms: 100
```

### For Low-Latency Control

```yaml
bridge:
  transports:
    websocket:
      port: 8765
      # Disable Nagle's algorithm for low latency
      tcp_nodelay: true
```

### System Limits

```bash
# Increase file descriptors
ulimit -n 65535

# Add to /etc/security/limits.conf
# * soft nofile 65535
# * hard nofile 65535
```

## Security Hardening

### Enable Authentication

```yaml
bridge:
  transports:
    websocket:
      port: 8765
      auth:
        enabled: true
        jwt_secret: "your-secret-key"
```

Generate token:
```bash
python scripts/generate_token.py --secret "your-secret-key" --user admin
```

### Firewall Rules

```bash
# Allow WebSocket port
sudo ufw allow 8765/tcp

# Allow MQTT port (if using)
sudo ufw allow 1883/tcp

# Deny all other incoming
sudo ufw default deny incoming
sudo ufw enable
```

### TLS/SSL

```yaml
bridge:
  transports:
    websocket:
      port: 8765
      tls_cert: "/etc/ssl/certs/agent-ros-bridge.crt"
      tls_key: "/etc/ssl/private/agent-ros-bridge.key"
```

## Development Setup

For developing on native ROS:

```bash
# 1. Install ROS (see above)

# 2. Clone repo
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# 3. Create venv with system site packages (to access rclpy/rospy)
python3 -m venv --system-site-packages venv
source venv/bin/activate

# 4. Install in editable mode
pip install -e ".[dev]"

# 5. Run tests
pytest tests/ -v

# 6. Run with mock (no ROS required)
python demo/mock_bridge.py
```

## Next Steps

- See [USER_MANUAL.md](USER_MANUAL.md) for detailed usage
- See [API_REFERENCE.md](API_REFERENCE.md) for API docs
- See [MULTI_ROS.md](MULTI_ROS.md) for fleet management
