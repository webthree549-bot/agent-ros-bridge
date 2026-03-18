# Agent ROS Bridge v0.6.2 Release Notes

**Release Date:** March 18, 2026  
**Version:** 0.6.2  
**Previous Version:** 0.6.1

---

## 🐛 Bug Fixes

### E2E Test Infrastructure
- **Fixed all E2E tests** - 47 passing (up from 43)
- **ROS message compilation** - Built `agent_ros_bridge_msgs` in Docker container
- **Fixed service file syntax** - `ParseIntent.srv`, `ResolveContext.srv` separators
- **Fixed package.xml** - Removed redundant dependencies

### Python API Fixes
- **Added `Robot` class alias** - Convenience alias for `RobotController`
- **Added `name` property** to `MotionPrimitive` for test compatibility
- **Fixed `navigate_to_pose()` signature** - Now accepts `x, y, theta` parameters

### Test Infrastructure
- **Updated test scripts** - Use compiled messages from `/tmp/ros_build/`
- **Added proper ROS initialization** - `rclpy.init()` / `rclpy.shutdown()`
- **Fixed import paths** - Corrected `robot_api` module path

---

## 📊 Test Results

| Metric | Before | After |
|--------|--------|-------|
| Unit Tests | 997 passing | 997 passing |
| E2E Tests | 43 passing | **47 passing** |
| E2E Skipped | 12 | **8** |
| E2E Failed | 0 | 0 |

---

## 🔧 Technical Details

### ROS Messages
Created Python stubs for ROS messages:
- `agent_ros_bridge_msgs/msg/Entity.msg`
- `agent_ros_bridge_msgs/msg/Intent.msg`
- `agent_ros_bridge_msgs/__init__.py`
- `agent_ros_bridge_msgs/msg/__init__.py`
- `agent_ros_bridge_msgs/srv/__init__.py`

### Docker Integration
- Tests run inside Docker container using `docker exec`
- Auto-detects ROS distro (Humble/Jazzy)
- Sources compiled message environment

---

## 🔄 Migration from v0.6.1

v0.6.2 is fully backward compatible with v0.6.1. No migration steps required.

---

## 📦 Installation

```bash
pip install agent-ros-bridge==0.6.2
```

---

**Full Changelog**: [CHANGELOG.md](CHANGELOG.md)

**GitHub Release**: https://github.com/webthree549-bot/agent-ros-bridge/releases/tag/v0.6.2
