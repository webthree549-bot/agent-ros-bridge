# MCP Quickstart

**Control ROS robots with Claude Desktop using natural language.**

---

## Prerequisites

1. **Claude Desktop** installed
2. **Agent ROS Bridge** v0.4.0+ installed
3. **ROS2** environment (or Docker)

---

## Installation

```bash
pip install agent-ros-bridge[mcp]
```

---

## Configure Claude Desktop

Edit `~/Library/Application Support/Claude/claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "ros": {
      "command": "python3",
      "args": ["-m", "agent_ros_bridge.mcp_server"],
      "env": {
        "JWT_SECRET": "your-secret-here"
      }
    }
  }
}
```

**Note:** On Mac, config location is:
- `~/Library/Application Support/Claude/claude_desktop_config.json`

---

## Usage

### 1. Start ROS

```bash
# Terminal 1: Start ROS2
docker run -it --rm --net=host ros:humble-ros-base ros2 topic list
```

### 2. Start MCP Server

```bash
# Terminal 2: Start MCP server
export JWT_SECRET=$(openssl rand -base64 32)
python3 -m agent_ros_bridge.mcp_server
```

### 3. Open Claude Desktop

Claude will automatically detect the ROS tools.

---

## Example Conversations

### Robot Navigation

**You:** "Navigate the robot to position (5, 3)"

**Claude:** I'll navigate the robot to coordinates (5, 3).

[Claude calls `ros_navigate` tool with x=5, y=3]

**Result:** Successfully navigated to position (5, 3). Current position confirmed via odometry.

---

### Check Robot Status

**You:** "What's the battery level?"

**Claude:** Let me check the battery status.

[Claude reads `ros://battery/status` resource]

**Result:** Battery level is 87%, estimated 2.3 hours remaining.

---

### Execute Action

**You:** "Move the arm to the home position"

**Claude:** Moving the robotic arm to home position.

[Claude calls `ros_move_arm` tool with position="home"]

**Result:** Arm successfully moved to home position. All joints within tolerance.

---

## Available Tools

MCP automatically exposes ROS actions as tools:

| Tool | Description | Example |
|------|-------------|---------|
| `ros_navigate` | Navigate to coordinates | Navigate to (x=5, y=3) |
| `ros_move_arm` | Move robotic arm | Move to "home" position |
| `ros_grasp` | Control gripper | Grasp object at position |
| `ros_patrol` | Start patrol route | Patrol area A |

---

## Available Resources

MCP automatically exposes ROS topics as resources:

| Resource | Type | Description |
|----------|------|-------------|
| `ros://battery/status` | JSON | Battery level and status |
| `ros://odom` | JSON | Robot position and velocity |
| `ros://scan` | JSON | LiDAR scan data |
| `ros://camera/image` | Binary | Camera image data |

---

## Troubleshooting

### Claude doesn't see tools

1. Check MCP server is running:
   ```bash
   ps aux | grep mcp_server
   ```

2. Restart Claude Desktop

3. Check config file syntax

### JWT errors

Ensure `JWT_SECRET` is set:
```bash
export JWT_SECRET=$(openssl rand -base64 32)
```

### ROS connection failed

Verify ROS is running:
```bash
ros2 topic list
```

---

## Next Steps

- Try the [MCP Garden Example](../examples/mcp-garden/)
- Read [MCP Architecture Guide](../docs/MCP_ARCHITECTURE.md)
- Explore [all examples](../examples/)

---

**You now have AI-powered robot control through natural language! 🤖**
