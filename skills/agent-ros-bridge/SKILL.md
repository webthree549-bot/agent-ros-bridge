---
name: agent-ros-bridge
description: Control ROS1/ROS2 robots via natural language. Use when users want to control robots, navigate, check sensors, manage fleets, or perform robot tasks. Supports ROS1 (Noetic) and ROS2 (Jazzy/Humble) through WebSocket, MQTT, or gRPC. Features include intelligent navigation, sensor interpretation, fleet coordination, and safety management.
---

# Agent ROS Bridge - Intelligent Robot Control

Control robots through natural language conversations with Agent ROS Bridge's universal gateway.

## Quick Start

### Start the Bridge
```bash
# Generate a secure secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start the bridge
agent-ros-bridge --websocket-port 8765
```

### Connect and Control

**Option 1: Natural Language (Recommended)**
```python
"Move forward 2 meters"
"Turn left 90 degrees"
"Navigate to the kitchen"
"What do you see?"
```

**Option 2: Direct API**
```python
import asyncio
import websockets
import json

async def control_robot():
    uri = "ws://localhost:8765?token=<JWT_TOKEN>"
    async with websockets.connect(uri) as ws:
        await ws.send(json.dumps({
            "command": {"action": "move", "parameters": {"direction": "forward", "distance": 2.0}}
        }))
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(control_robot())
```

---

## Natural Language Commands

### Movement Commands

#### Basic Movement
| Natural Language | Technical Equivalent |
|------------------|---------------------|
| "Move forward" | Publish to `/cmd_vel` with linear.x > 0 |
| "Move backward slowly" | Publish with linear.x < 0, reduced speed |
| "Turn left" | Publish with angular.z > 0 |
| "Turn right 90 degrees" | Rotate with angular.z for calculated duration |
| "Stop" | Publish zero velocity |
| "Spin around" | Rotate 360 degrees |

#### Intelligent Navigation
| Natural Language | Behavior |
|------------------|----------|
| "Go to the kitchen" | Navigate using Nav2 to named waypoint |
| "Drive to charging station" | Navigate and dock automatically |
| "Follow me" | Person-following mode |
| "Explore this room" | Autonomous exploration with mapping |
| "Return to base" | Navigate to home position |
| "Avoid obstacles" | Enable obstacle avoidance |

**Parameter Inference:**
- Speed: "slowly" = 0.1 m/s, "normal" = 0.5 m/s, "fast" = 1.0 m/s
- Distance: "a bit" = 0.5m, "a little" = 0.3m, specific numbers used exactly
- Direction: Context-aware (forward = current heading)

### Sensor Queries

#### Visual Perception
| Natural Language | Data Source | Interpretation |
|------------------|-------------|----------------|
| "What do you see?" | Camera + LiDAR | Scene description |
| "Is there anything in front?" | Forward sensors | Obstacle detection |
| "How far is the wall?" | LiDAR / Depth | Distance estimation |
| "Describe the room" | Camera + 360° scan | Spatial layout |
| "Can you see [object]?" | Object detection | Yes/No + location |

#### Status Queries
| Natural Language | Response |
|------------------|----------|
| "What's your status?" | Battery, location, current task |
| "How's your battery?" | Battery percentage + time remaining |
| "Where are you?" | Current location / coordinates |
| "What are you doing?" | Current action + progress |
| "Are you stuck?" | Stuck detection + recovery options |

### Fleet Management

#### Robot Selection
| Natural Language | Selection Criteria |
|------------------|-------------------|
| "Which robot is closest?" | Distance to target |
| "Which robot has most battery?" | Battery level |
| "Which robot is available?" | Idle status |
| "Send the best robot to [location]" | Multi-criteria optimization |

#### Multi-Robot Coordination
| Natural Language | Coordination Pattern |
|------------------|---------------------|
| "Search the building together" | Divide area, parallel search |
| "Form a convoy" | Follow-the-leader pattern |
| "Surround the area" | Distributed perimeter coverage |
| "Patrol in shifts" | Time-based rotation |

### Safety Commands

#### Emergency Actions
| Command | Action | Confirmation Required |
|---------|--------|----------------------|
| "Emergency stop!" | Immediate halt | No (always executes) |
| "Stop everything" | Cancel all tasks | No |
| "Return to safe mode" | Conservative behavior | No |

#### Safety Queries
| Natural Language | Check |
|------------------|-------|
| "Is it safe to move?" | Obstacle + environment check |
| "Check surroundings" | 360° safety scan |
| "Verify path is clear" | Path planning validation |

---

## Common Tasks by Scenario

### Home Environment

**Morning Routine:**
```
"Check all rooms"
"Bring me coffee from the kitchen"
"Check if doors are locked"
"Water the plants"
```

**Security Patrol:**
```
"Patrol the house every hour"
"Check for unusual sounds"
"Monitor the front door"
"Alert me if you see anyone"
```

### Office Environment

**Delivery Tasks:**
```
"Deliver this package to Room 302"
"Collect documents from the printer"
"Bring coffee to the conference room"
"Distribute mail to all offices"
```

**Meeting Support:**
```
"Escort the visitor to the meeting room"
"Prepare the conference room"
"Check AV equipment status"
"Guide guests to their seats"
```

### Warehouse/Industrial

**Inventory:**
```
"Scan shelf A-12"
"Count items in zone B"
"Find item SKU-12345"
"Report low stock items"
```

**Logistics:**
```
"Transport pallet to shipping dock"
"Organize incoming inventory"
"Prepare order for shipment"
"Load truck bay 3"
```

---

## Advanced Features

### Context Awareness

The skill maintains context across conversations:

```
User: "Go to the kitchen"
[Robot navigates to kitchen]

User: "Now bring me water"
[Robot knows it's in kitchen, searches for water]

User: "Return to base"
[Robot knows to return to starting position]
```

### Learning & Memory

**Location Learning:**
```
User: "Remember this as 'charging station'"
[Robot saves current location]

User: "Go to charging station"
[Robot navigates to learned location]
```

**Path Learning:**
```
User: "Learn this route to the office"
[Robot records path]

User: "Take the learned route"
[Robot follows recorded path]
```

### Autonomous Behaviors

**Exploration:**
```
"Explore autonomously for 10 minutes"
"Map this area"
"Find all rooms"
"Identify objects of interest"
```

**Patrol:**
```
"Patrol every 30 minutes"
"Check for anomalies"
"Report anything unusual"
"Maintain security perimeter"
```

---

## Safety & Best Practices

### Dangerous Actions (Require Confirmation)

These actions trigger safety confirmation:
- Movement in confined spaces (< 1m clearance)
- Speed > 1.5 m/s
- Operating near humans without detection
- Multi-robot coordination in tight spaces
- Autonomous exploration in unknown areas

### Emergency Procedures

**Immediate Stop:**
- Say "Emergency stop" or "Stop now"
- No confirmation required
- Halts all motion immediately

**Recovery:**
- "Resume previous task" - Continue after stop
- "Return to safe position" - Go to known safe location
- "Reset and start over" - Clear state and restart

### Safety Checklist

Before autonomous operations:
1. ✓ Environment scanned for obstacles
2. ✓ Humans accounted for (if present)
3. ✓ Emergency stop accessible
4. ✓ Battery sufficient for task
5. ✓ Communication link verified

---

## Technical Reference

### Connection Details

**WebSocket:**
- URL: `ws://localhost:8765?token=<JWT>`
- Protocol: JSON messages
- Auth: JWT token required

**MQTT:**
- Broker: Configurable (default: localhost:1883)
- Topics: `agent_ros_bridge/commands`, `agent_ros_bridge/telemetry`

**gRPC:**
- Port: 50051 (configurable)
- TLS: Supported with certificates

### Message Format

**Command:**
```json
{
  "command": {
    "action": "move",
    "parameters": {
      "direction": "forward",
      "distance": 1.0,
      "speed": 0.5
    }
  }
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "action": "move",
    "completed": true,
    "duration": 2.5
  }
}
```

### Available Actions

**Movement:**
- `move` - Linear movement
- `rotate` - Angular rotation
- `navigate` - Goal-based navigation
- `follow` - Person following
- `explore` - Autonomous exploration

**Sensors:**
- `get_camera_image` - Capture camera frame
- `get_lidar_scan` - Get LiDAR data
- `get_odometry` - Position and velocity
- `detect_objects` - Object detection

**Fleet:**
- `list_robots` - Available robots
- `select_robot` - Choose best robot
- `coordinate` - Multi-robot task
- `get_fleet_status` - Overall status

**Safety:**
- `emergency_stop` - Immediate halt
- `get_safety_status` - Safety system state
- `confirm_safe` - Safety verification

---

## Troubleshooting

### Common Issues

**"Connection refused"**
- Check if bridge is running
- Verify port 8765 is available
- Check firewall settings

**"Authentication failed"**
- Verify JWT token is valid
- Check token hasn't expired
- Ensure JWT_SECRET matches

**"Robot not responding"**
- Check ROS connection
- Verify robot is powered on
- Check for error messages

**"Navigation failed"**
- Check if map is loaded
- Verify goal is reachable
- Check for obstacles

### Debug Commands

```python
# Check bridge status
"Get bridge status"

# Check robot connection
"Verify robot connection"

# Test sensors
"Test all sensors"

# Diagnostic report
"Generate diagnostic report"
```

---

## Examples Gallery

### Example 1: Morning House Check
```
User: "Check the house"
→ Robot: "Starting house check. I'll visit each room and report."

[Robot moves through rooms]

Robot: "House check complete. Kitchen: normal. Living room: window 
        left open. Bedroom: normal. No issues detected."
```

### Example 2: Office Delivery
```
User: "Deliver this to Alice in Room 302"
→ Robot: "I'll deliver to Alice in Room 302. Navigating..."

[Robot navigates, finds Alice]

Robot: "Delivery complete. Alice acknowledged receipt. Returning to base."
```

### Example 3: Fleet Coordination
```
User: "Search the warehouse for item SKU-12345"
→ Robot: "Coordinating search with 3 robots. Dividing area..."

[Robots search in parallel]

Robot: "Item found by Robot-2 in Zone C, Shelf 12. All robots returning."
```

---

## References

- **ROS1 Guide**: See `references/ros1-guide.md` for ROS1-specific details
- **ROS2 Guide**: See `references/ros2-guide.md` for ROS2-specific details
- **API Reference**: See `docs/API_REFERENCE.md` for complete API
- **Architecture**: See `docs/ARCHITECTURE_V2.md` for system design

---

## Version Info

- **Skill Version**: 0.5.0
- **Agent ROS Bridge**: 0.5.0+
- **ROS1 Support**: Noetic
- **ROS2 Support**: Jazzy, Humble
- **Last Updated**: 2026-03-04
