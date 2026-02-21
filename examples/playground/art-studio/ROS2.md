# ROS2 Art Studio - Real ROS2 Deployment

This directory contains **real ROS2 Humble** nodes, not simulation.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Network                           │
├─────────────────────────────────────────────────────────────┤
│  ┌────────────────┐      ┌────────────────────────────┐   │
│  │ Agent Bridge   │◄────►│ ROS2 Art Bridge Node       │   │
│  │ (WebSocket     │      │ (Translates bridge ↔ ROS2) │   │
│  │  Port 8765)    │      └────────────────────────────┘   │
│  └────────────────┘                    │                    │
│         │                              │                    │
│         │                              ▼                    │
│         │                    ┌──────────────────┐          │
│         │                    │ ROS2 Topics:     │          │
│         │                    │ /art/emotion     │          │
│         │                    │ /art/strokes     │          │
│         │                    │ /art/canvas      │          │
│         │                    │ /art/robot_status│          │
│         │                    └──────────────────┘          │
│         │                              │                    │
│         │                              ▼                    │
│         │                    ┌──────────────────┐          │
│         │                    │ Painter Nodes    │          │
│         └───────────────────►│ (Real ROS2 nodes)│          │
│                              │ painter_1        │          │
│                              │ painter_2        │          │
│                              └──────────────────┘          │
└─────────────────────────────────────────────────────────────┘
```

## ROS2 Nodes

### 1. painter_node
**File:** `ros2/art_studio/art_studio/painter_node.py`

**ROS2 Interface:**
- **Subscribes:** `/art/emotion` (std_msgs/String)
- **Publishes:** `/art/strokes` (std_msgs/String - JSON)
- **Publishes:** `/art/robot_status` (std_msgs/String)

**Usage:**
```bash
ros2 run art_studio painter_node
```

### 2. canvas_node
**File:** `ros2/art_studio/art_studio/canvas_node.py`

**ROS2 Interface:**
- **Subscribes:** `/art/strokes` (aggregates from all painters)
- **Publishes:** `/art/canvas` (std_msgs/String - JSON canvas state)

**Usage:**
```bash
ros2 run art_studio canvas_node
```

### 3. art_bridge (Agent ROS Bridge Integration)
**File:** `art_brain_ros2.py`

**Purpose:** Connects Agent ROS Bridge to ROS2 topics
- Receives commands from Agent ROS Bridge via WebSocket
- Publishes to `/art/emotion` ROS2 topic
- Forwards ROS2 strokes to Agent ROS Bridge clients

## Running

### Prerequisites
```bash
# Set JWT secret
export JWT_SECRET=$(openssl rand -base64 32)
```

### Option A: Docker (Recommended)
```bash
docker-compose -f docker-compose.ros2.yml up --build
```

This starts:
- Agent ROS Bridge + ROS2 Bridge Node
- 2 Painter ROS2 nodes
- ROS2 topic monitor

### Option B: Native ROS2
```bash
# Terminal 1: Build and source
cd ros2/art_studio
colcon build
source install/setup.bash

# Terminal 2: Run canvas
ros2 run art_studio canvas_node

# Terminal 3: Run painter
ROBOT_ID=painter_1 ROBOT_STYLE=abstract ros2 run art_studio painter_node

# Terminal 4: Run bridge
cd ../..
export JWT_SECRET=$(openssl rand -base64 32)
python3 art_brain_ros2.py
```

## Verifying ROS2 Topics

```bash
# List all topics
ros2 topic list

# Expected output:
# /art/canvas
# /art/emotion
# /art/robot_status
# /art/strokes

# Monitor strokes
ros2 topic echo /art/strokes

# Publish test emotion
ros2 topic pub /art/emotion std_msgs/String '{data: "joy"}'
```

## Proof of Real ROS2

**Real ROS2 Humble base image:**
```dockerfile
FROM ros:humble-ros-base
```

**Real rclpy nodes:**
```python
import rclpy
from rclpy.node import Node

class PainterNode(Node):  # Real ROS2 node
    def __init__(self):
        super().__init__('painter_node')  # Node registration
        self.publisher = self.create_publisher(String, '/art/strokes', 10)
```

**Real ROS2 topic publishing:**
```python
msg = String()
msg.data = json.dumps(stroke)
self.publisher.publish(msg)  # Actual ROS2 publish
```

**Real ROS2 CLI tools:**
```bash
ros2 topic list
ros2 topic echo /art/strokes
ros2 node list
```

## Files

| File | Purpose |
|------|---------|
| `Dockerfile.ros2` | ROS2 Humble base with bridge |
| `docker-compose.ros2.yml` | Multi-node ROS2 deployment |
| `art_brain_ros2.py` | Bridge integration node |
| `ros2/art_studio/package.xml` | ROS2 package manifest |
| `ros2/art_studio/setup.py` | ROS2 package setup |
| `ros2/art_studio/art_studio/painter_node.py` | Real ROS2 painter node |
| `ros2/art_studio/art_studio/canvas_node.py` | Real ROS2 canvas node |

---

**Status:** ✅ Real ROS2 Humble nodes (not simulation)