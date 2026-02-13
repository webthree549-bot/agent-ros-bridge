# Naming Strategy for ROS Bridge Project

## Problem Analysis

**Current Issues:**
1. **"OpenClaw ROS Bridge"** - Implies it's part of OpenClaw platform, not a skill for it
2. **"ClawHub" references** - Redundant since all skills are on ClawHub
3. **Confusing positioning** - Unclear if it's a standalone tool or a skill

**Reserved Terminology:**
- ❌ `OpenClaw` - The AI agent platform
- ❌ `ClawHub` - The skill marketplace
- ❌ `OpenClaw-ROS` - Implies official integration

## Naming Principles

1. **Platform-Agnostic** - Works with any AI agent, not just OpenClaw
2. **Descriptive** - Clearly indicates what it does
3. **Professional** - Suitable for enterprise/production use
4. **Unique** - Distinct from existing ROS tools
5. **Extensible** - Not limited to ROS (supports MQTT, Modbus, etc.)

## Recommended Names (Ranked)

### 🥇 Primary Recommendation
**"Universal Robot Gateway"** (URG)
- ✅ Platform-agnostic
- ✅ Describes multi-robot, multi-protocol nature
- ✅ Professional
- ✅ Extensible beyond ROS
- ✅ Abbreviates well (URG)

### 🥈 Strong Alternatives

1. **"Robot Gateway"**
   - Simple, clear
   - Could be confused with other gateways

2. **"Embodied Gateway"**
   - Appeals to embodied AI community
   - May be too academic

3. **"Multi-Protocol Robot Bridge"**
   - Very descriptive
   - A bit long

4. **"Agent-Robot Gateway"**
   - Clear AI-to-robot purpose
   - Good for marketing

### 🥉 Other Options

- "BotGateway" - Too casual
- "RoboBridge" - Too informal
- "Machine Gateway" - Too broad
- "ROS Universal Bridge" - Too ROS-specific
- "AI Robot Connector" - Too generic

## Recommended Naming Strategy

### Primary Name
**Universal Robot Gateway** or **Universal Robot Gateway (URG)**

### Package/Repository Name
```
universal-robot-gateway
urg-gateway
robot-gateway
```

### Python Package
```python
import urg_gateway
# or
import robot_gateway
from robot_gateway import Gateway
```

### CLI Command
```bash
robot-gateway --demo
# or
urg-gateway --demo
# or
rgateway --demo
```

### Docker Image
```
robot-gateway:latest
universal-robot-gateway:latest
urg:latest
```

### Skill Name on ClawHub
```
robot-gateway        # Simple
universal-gateway    # Descriptive
ros-connector        # Specific
```

## Updated Terminology Guide

| Old Term | New Term | Notes |
|----------|----------|-------|
| OpenClaw ROS Bridge | Universal Robot Gateway | Primary name |
| openclaw-ros-bridge | robot-gateway | Repo name |
| openclaw_ros_bridge | robot_gateway | Python package |
| openclaw-gateway | robot-gateway | CLI command |
| `import openclaw_ros_bridge` | `import robot_gateway` | Import |
| "This skill..." | "This gateway..." | Description |
| "For OpenClaw..." | "For AI agents..." | Platform-agnostic |

## Brand Positioning

**Tagline:**
> "Universal Robot Gateway - Connect any AI agent to any robot"

**Description:**
> "A multi-protocol, multi-robot gateway enabling AI agents to control ROS-based robots, industrial arms, drones, and IoT devices."

**Key Messages:**
1. **Universal** - Not just OpenClaw, works with any AI agent
2. **Multi-Protocol** - WebSocket, gRPC, MQTT, TCP
3. **Multi-Robot** - ROS1, ROS2, industrial, drones
4. **Production-Ready** - Enterprise-grade, cloud-native

## Migration Path

1. **Repository Rename:** `openclaw-ros-bridge` → `robot-gateway`
2. **Package Rename:** `openclaw_ros_bridge` → `robot_gateway`
3. **CLI Rename:** `openclaw-gateway` → `robot-gateway`
4. **Documentation Update:** Remove all "OpenClaw" references
5. **Skill Submission:** Submit as `robot-gateway` on ClawHub

## Example Usage (After Rename)

```python
# Before
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway

# After
from robot_gateway import Gateway

# Usage
gateway = Gateway()
gateway.add_transport(WebSocketTransport(port=8765))
await gateway.start()
```

```bash
# Before
openclaw-gateway --demo

# After
robot-gateway --demo
```

## Recommendation

**Go with "Universal Robot Gateway" (URG)**

Short name: **robot-gateway**
Python package: **robot_gateway**
CLI: **robot-gateway**

This is:
- ✅ Professional
- ✅ Platform-agnostic
- ✅ Descriptive
- ✅ Unique
- ✅ Future-proof
