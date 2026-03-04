# Greenhouse Quick Start Guide

## 5-Minute Setup

### 1. Start Agent ROS Bridge (2 min)

```bash
# Install
pip install agent-ros-bridge

# Configure
export JWT_SECRET=$(openssl rand -base64 32)

# Start
agent-ros-bridge --websocket-port 8765
```

### 2. Connect Robots (2 min)

```bash
# For each ROS2 robot
ros2 run <robot_package> <node> --ros-args \
  -p bridge_host:=localhost \
  -p bridge_port:=8765
```

### 3. Start Using Natural Language (1 min)

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter

bridge = Bridge()
adapter = bridge.get_openclaw_adapter()

# Teach locations
adapter.learn_location("zone_a", {"x": 10, "y": 5})
adapter.learn_location("zone_b", {"x": 30, "y": 5})

# Start commanding!
result = await adapter.execute_nl("Check tomatoes in Zone A")
```

## Common Commands

| Task | Natural Language Command |
|------|-------------------------|
| Inspection | "Check Zone A for issues" |
| Watering | "Water the cucumbers" |
| Harvesting | "Pick ripe tomatoes" |
| Patrol | "Patrol all zones" |
| Status | "Robot status report" |

## Example Session

```
You: "Check Zone A"
System: "Deploying Scout-1... Found 12 ripe tomatoes, 3 need water"

You: "Water them"
System: "Water-1 deployed to Zone A... Complete"

You: "Harvest ripe vegetables"
System: "Harvest-1 picking... 20 items collected"
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot not responding | Check battery, send to charging station |
| Command not understood | Use simpler language, check location names |
| Connection failed | Verify JWT_SECRET, check port 8765 |

## Next Steps

1. Read full use case: `USE_CASE_GREENHOUSE.md`
2. Configure your robots: Edit `config/greenhouse.yaml`
3. Train your team: 10-minute orientation
4. Go autonomous: Set up scheduled patrols

---
**Questions?** See full documentation or run `agent-ros-bridge --help`
