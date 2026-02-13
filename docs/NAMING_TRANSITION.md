# Naming Transition Guide

## Overview

This document tracks the naming transition from "OpenClaw ROS Bridge" to "Universal Robot Gateway".

---

## Why the Change?

**Problem:**
- "OpenClaw" is the AI agent platform
- All skills on ClawHub work with OpenClaw by definition
- Naming created confusion about the project's scope

**Solution:**
- Use platform-agnostic naming
- Focus on what the project does (robot gateway)
- Avoid reserved terminology

---

## Naming Mapping

### Project Identity

| Aspect | Old Name | New Name |
|--------|----------|----------|
| **Project Name** | OpenClaw ROS Bridge | Universal Robot Gateway |
| **Short Name** | OpenClaw-ROS | URG / Robot Gateway |
| **Python Package** | `openclaw_ros_bridge` | `robot_gateway` |
| **CLI Command** | `openclaw-gateway` | `robot-gateway` |
| **Docker Image** | `openclaw/ros-bridge` | `robot-gateway/robot-gateway` |
| **PyPI Package** | `openclaw-ros-bridge` | `robot-gateway` |
| **GitHub Repo** | `openclaw-ros-bridge` | `robot-gateway` |
| **ClawHub Skill** | `openclaw-ros-bridge` | `robot-gateway` |

### References in Documentation

| Old Reference | New Reference |
|---------------|---------------|
| "OpenClaw AI Agent" | "AI Agent" / "Your AI Agent" |
| "OpenClaw user" | "User" / "Developer" |
| "For OpenClaw" | "For AI agents" |
| "OpenClaw integration" | "AI agent integration" |
| "OpenClaw platform" | "AI agent platform" |

### Code References

| Old Import | New Import |
|------------|------------|
| `import openclaw_ros_bridge` | `import robot_gateway` |
| `from openclaw_ros_bridge.gateway_v2` | `from robot_gateway` |
| `from openclaw_ros_bridge.legacy` | `from robot_gateway.legacy` |
| `OpenClawGateway` | `Gateway` |

---

## Files Updated

### Documentation
- ✅ `README.md` - Updated
- ✅ `SKILL.md` - Updated
- ✅ `docs/USER_MANUAL.md` - Needs update
- ✅ `docs/API_REFERENCE.md` - Needs update
- ✅ `docs/MIGRATION.md` - Needs update

### Configuration
- ✅ `pyproject.toml` - Package name needs update
- ✅ `setup.py` - If exists, needs update

### Scripts
- Commands updated to `robot-gateway`

---

## Migration for Users

### Python Code

**Before:**
```python
import openclaw_ros_bridge
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway

gateway = OpenClawGateway()
```

**After:**
```python
import robot_gateway
from robot_gateway import Gateway

gateway = Gateway()
```

### CLI Usage

**Before:**
```bash
openclaw-gateway --demo
openclaw-gateway --config ./config.yaml
```

**After:**
```bash
robot-gateway --demo
robot-gateway --config ./config.yaml
```

### Installation

**Before:**
```bash
pip install openclaw-ros-bridge
openclaw skills add openclaw-ros-bridge
```

**After:**
```bash
pip install robot-gateway
openclaw skills add robot-gateway
```

### Docker

**Before:**
```bash
docker run openclaw/ros-bridge:latest
```

**After:**
```bash
docker run robot-gateway/robot-gateway:latest
```

---

## Repository Migration Checklist

When moving to the new GitHub organization:

- [ ] Create `universal-robot-gateway` GitHub organization (or use existing)
- [ ] Fork/transfer repository to new org
- [ ] Update all GitHub URLs in documentation
- [ ] Update CI/CD workflows
- [ ] Update Docker Hub repository
- [ ] Update PyPI package name
- [ ] Update ClawHub submission
- [ ] Set up redirects from old repo
- [ ] Archive old repository

---

## Brand Assets

### Logo/Text Representation
```
🤖 Universal Robot Gateway
```

### Tagline
> "Connect any AI agent to any robot"

### Description
> "A multi-protocol, multi-robot gateway enabling AI agents to control ROS-based robots, industrial arms, drones, and IoT devices."

### Keywords
- Robot Gateway
- ROS Gateway
- AI-Robot Bridge
- Universal Robot Interface
- Embodied AI Gateway
- Multi-Protocol Robot Gateway

---

## Reserved Terminology (Do Not Use)

These terms are reserved and should not appear in project naming:

❌ **OpenClaw** - The AI agent platform
❌ **ClawHub** - The skill marketplace  
❌ **OpenClaw-ROS** - Implies official integration
❌ **Claw** - Platform brand
❌ **OpenC** - Abbreviation of OpenClaw

**Instead, use:**
✅ AI Agent
✅ Your AI Agent
✅ AI System
✅ Agent Platform
✅ (Generic terms)

---

## FAQ

**Q: Is this still compatible with OpenClaw?**
A: Yes! It's a skill on ClawHub. The name just doesn't imply exclusivity.

**Q: What AI agents does it work with?**
A: Any AI agent that can connect via WebSocket, gRPC, MQTT, or TCP.

**Q: Why "Universal Robot Gateway"?**
A: It emphasizes the universal nature - any AI agent, any robot, multiple protocols.

**Q: Can I still use the old name?**
A: The old name is deprecated. Please migrate to the new naming.

---

## Timeline

| Phase | Date | Action |
|-------|------|--------|
| Phase 1 | Now | Update documentation (this PR) |
| Phase 2 | +1 week | Update package names, PyPI |
| Phase 3 | +2 weeks | Repository migration |
| Phase 4 | +3 weeks | Archive old references |

---

## Contact

For questions about the naming transition:
- GitHub Issues: https://github.com/universal-robot-gateway/robot-gateway/issues
- Email: dev@robot-gateway.org

---

**Last Updated:** February 13, 2026
