# Multi-ROS and Remote Robot Configuration

Agent ROS Bridge supports connecting to multiple ROS endpoints simultaneously, including mixed ROS1/ROS2 environments and remote robots over the network.

## Quick Start

### Single Local ROS (Auto-Detect)
```bash
python run_bridge.py
```

### Multiple ROS Endpoints

Create `config/bridge.yaml`:

```yaml
bridge:
  connectors:
    ros:
      auto_detect: false  # Disable auto-detection
      endpoints:
        # Local ROS2
        - id: "local_ros2"
          ros_type: "ros2"
          ros_distro: "jazzy"
          host: "localhost"
          domain_id: 0
        
        # Remote ROS2 robot
        - id: "warehouse_bot_01"
          ros_type: "ros2"
          ros_distro: "humble"
          host: "192.168.1.100"
          domain_id: 42
          topics:
            - "/cmd_vel"
            - "/odom"
            - "/scan"
        
        # Remote ROS1 robot
        - id: "legacy_arm"
          ros_type: "ros1"
          ros_distro: "noetic"
          host: "192.168.1.101"
          port: 11311
```

Then run:
```bash
python run_bridge.py
# Or with explicit config:
BRIDGE_CONFIG=config/bridge.yaml python run_bridge.py
```

## Configuration Options

### ROS2 Endpoint

| Option | Description | Default |
|--------|-------------|---------|
| `id` | Unique identifier for this endpoint | (required) |
| `ros_type` | `"ros2"` | (required) |
| `ros_distro` | `"jazzy"`, `"humble"`, `"iron"` | `"jazzy"` |
| `host` | Hostname or IP address | `"localhost"` |
| `domain_id` | ROS_DOMAIN_ID for this endpoint | `0` |
| `auto_discover` | Auto-discover topics | `true` |
| `topics` | List of topics to subscribe to | `[]` |

### ROS1 Endpoint

| Option | Description | Default |
|--------|-------------|---------|
| `id` | Unique identifier for this endpoint | (required) |
| `ros_type` | `"ros1"` | (required) |
| `ros_distro` | `"noetic"`, `"melodic"` | `"noetic"` |
| `host` | Hostname or IP address | `"localhost"` |
| `port` | ROS_MASTER port | `11311` |
| `auto_discover` | Auto-discover topics | `true` |
| `topics` | List of topics to subscribe to | `[]` |

## Docker Multi-ROS

Run multiple bridge instances with different ROS versions:

```bash
# ROS1 bridge
docker-compose --profile ros1 up ros1-bridge

# ROS2 bridge (different port)
docker-compose --profile ros2 up ros2-bridge
```

Or use a single bridge with configured endpoints:

```bash
# Edit config/bridge.yaml with your endpoints
docker-compose --profile multi up multi-bridge
```

## Network Requirements

### ROS2 Remote Discovery

ROS2 uses DDS for discovery. For remote robots:

1. **Same Domain ID**: Ensure `ROS_DOMAIN_ID` matches on both sides
2. **Multicast**: DDS requires multicast (or use Discovery Server)
3. **Firewall**: Open ports for DDS (typically 7400-7500)

Example for remote ROS2:
```yaml
- id: "remote_bot"
  ros_type: "ros2"
  host: "192.168.1.100"
  domain_id: 42
```

### ROS1 Remote Master

ROS1 uses a central master. For remote robots:

1. **ROS_MASTER_URI**: Point to the remote roscore
2. **ROS_HOSTNAME**: Set to this machine's IP
3. **Port 11311**: Default ROS master port

Example for remote ROS1:
```yaml
- id: "remote_arm"
  ros_type: "ros1"
  host: "192.168.1.101"
  port: 11311
```

## Querying Multiple Robots

Once connected, query all robots:

```bash
wscat -c ws://localhost:8765

# List all robots across all endpoints
> {"command": {"action": "list_robots"}}

# Get topics from a specific robot
> {"command": {"action": "get_topics", "parameters": {"endpoint_id": "warehouse_bot_01"}}}

# Send command to specific robot
> {"command": {"action": "publish", "parameters": {"endpoint_id": "legacy_arm", "topic": "/joint_states", "data": {...}}}}
```

## Use Cases

### Warehouse Fleet
- 10x ROS2 Humble AMRs (domain_id 0-9)
- 1x bridge connecting to all via domain-specific endpoints

### Mixed Legacy/Modern
- 3x ROS1 Noetic UR arms
- 2x ROS2 Humble mobile bases
- 1x bridge managing both generations

### Remote Teleoperation
- Bridge running in cloud (AWS/GCP)
- Connected to robots in warehouse via VPN
- Operators control from anywhere

## Troubleshooting

### ROS2: Cannot see remote topics
- Check `ROS_DOMAIN_ID` matches
- Verify multicast is enabled: `ros2 multicast receive` / `ros2 multicast send`
- Use Discovery Server for WAN: [ROS2 Discovery Server](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)

### ROS1: Connection refused
- Verify `roscore` is running on remote host
- Check firewall: `telnet 192.168.1.101 11311`
- Ensure `ROS_MASTER_URI` and `ROS_HOSTNAME` are set correctly

### Multiple bridges conflict
- Each bridge needs unique WebSocket port
- Use config: `transports.websocket.port: 8766` for second bridge
