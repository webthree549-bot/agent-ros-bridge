# Agent ROS Bridge - OpenClaw Skill

Real-time robot control skill for OpenClaw with bidirectional streaming.

## Features

- 🔄 **Real-time telemetry** streaming to OpenClaw (2Hz)
- 🎮 **Bidirectional control** - OpenClaw → Robot → OpenClaw
- 🌐 **Web UI sync** - Synchronized state across all clients
- 🚨 **Emergency stop** - Instant broadcast to all connected clients
- 📊 **Progress updates** - Live command execution feedback

## Quick Start

### 1. Start Agent ROS Bridge

```bash
# Terminal 1: Start the main bridge
agent-ros-bridge --websocket-port 8765
```

### 2. Start OpenClaw Bridge

```bash
# Terminal 2: Start the OpenClaw integration bridge
python skills/agent-ros-bridge/scripts/openclaw_bridge.py --port 8766
```

### 3. Use in OpenClaw

```python
from skills.agent_ros_bridge.scripts.openclaw_bridge import OpenClawSkillInterface

async def example():
    # Connect
    interface = OpenClawSkillInterface(port=8766)
    await interface.connect()
    
    # Send command with real-time feedback
    result = await interface.send_command("Navigate to kitchen")
    print(f"Result: {result}")
    
    # Get live telemetry
    telemetry = await interface.get_telemetry()
    print(f"Battery: {telemetry['battery']}%")
    
    # Disconnect
    await interface.disconnect()
```

## Web UI Integration

The same bridge powers the web dashboard:

```javascript
const ws = new WebSocket('ws://localhost:8766', ['openclaw-bridge']);

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  
  if (data.type === 'telemetry_update') {
    updateRobotPosition(data.data.position);
  }
};
```

## Configuration

Edit `config/openclaw_bridge.yaml`:

```yaml
bridge:
  websocket_port: 8766
  agent_ros_port: 8765
  
  telemetry:
    update_rate: 2  # Hz
    fields:
      - position
      - battery
      - sensor_status
```

## Architecture

```
OpenClaw ←──WebSocket──→ OpenClaw Bridge ←──WebSocket──→ Agent ROS Bridge ←──→ Robot
                             │
Web UI ←─────────────────────┘
```

## Files

| File | Purpose |
|------|---------|
| `SKILL.md` | Full documentation |
| `scripts/openclaw_bridge.py` | Bridge server & interface |
| `config/openclaw_bridge.yaml` | Configuration |

## Version

- Skill: 0.7.0
- Requires: Agent ROS Bridge 0.6.5+
