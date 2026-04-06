# Modular Architecture (v0.7.0)

Agent ROS Bridge is transitioning to a modular architecture for v0.7.0.

## Motivation

The monolithic structure had grown to 143 MB with unclear boundaries:
- High maintenance burden
- Users confused by scope
- All-or-nothing installation

## New Structure

### Core Package (Required)
```
agent_ros_bridge/          # ~30 MB
├── gateway/               # Protocol handling
├── shadow/                # Safety validation
├── safety/                # Human-in-the-loop
└── core/                  # Minimal core
```

### Optional Packages
```
agent_ros_bridge_fleet/    # Multi-robot orchestration ✅
agent_ros_bridge_sim/      # Gazebo integration ✅
agent_ros_bridge_tools/    # Tool ecosystem ✅ (5 NASA ROSA tools)
```

## Installation

**Core only:**
```bash
pip install agent-ros-bridge
```

**With fleet management:**
```bash
pip install agent-ros-bridge
pip install agent-ros-bridge-fleet
```

**Full installation:**
```bash
pip install agent-ros-bridge[all]
```

## Backward Compatibility

Existing code continues to work:

```python
# Old imports still work (with package installed)
from agent_ros_bridge.fleet import FleetOrchestrator

# New imports recommended
from agent_ros_bridge_fleet import FleetOrchestrator
```

The main package provides compatibility shims that try the new package first,
then fall back to local imports for development.

## Migration Guide

### For Users

No changes required. Existing imports work.

### For Contributors

Fleet-related changes now go to `agent-ros-bridge-fleet/`:

```bash
cd agent-ros-bridge-fleet/
# Make changes
pytest tests/
```

## Timeline

| Phase | Date | Action |
|-------|------|--------|
| v0.6.5 | 2026-03-30 | Foundation complete |
| v0.7.0 | 2026-04-27 | Fleet package extracted |
| v0.7.5 | 2026-05 | Sim package extracted |
| v0.8.0 | 2026-06 | Tools package extracted |

## Package Status

| Package | Status | Size | PyPI |
|---------|--------|------|------|
| agent_ros_bridge | Stable | ~30 MB | ✅ v0.6.5 |
| agent_ros_bridge_fleet | Dev | ~5 MB | 🚧 v0.7.0.dev1 |
| agent_ros_bridge_sim | Dev | ~50 MB | 🚧 v0.7.0.dev1 |
| agent_ros_bridge_tools | Dev | ~10 MB | 🚧 v0.7.0.dev1 |

### Tools Package Contents

The tools package includes 5 NASA ROSA-compatible tools:

| Tool | Purpose |
|------|---------|
| `rostopic_echo` | Echo messages from ROS topics |
| `rosservice_call` | Call ROS services |
| `rosnode_list` | List running ROS nodes |
| `rosparam_get` | Get ROS parameters |
| `rosbag_play` | Play recorded bag files |

All tools support both ROS1 and ROS2.

## Repository Structure

```
workspace/
├── agent_ros_bridge/              # Core package
│   ├── fleet/                     # Compatibility shim
│   └── ...
├── agent-ros-bridge-fleet/        # Separate package
│   ├── src/agent_ros_bridge_fleet/
│   ├── tests/
│   └── pyproject.toml
└── ...
```

## Benefits

1. **Smaller core**: Install only what you need
2. **Clearer boundaries**: Each package has a single purpose
3. **Independent releases**: Fleet can release faster than core
4. **Better testing**: Isolated test suites
5. **Easier contributions**: Focused codebase

## Questions?

See [GitHub Issues](https://github.com/webthree549-bot/agent-ros-bridge/issues) or [Documentation](https://webthree549-bot.github.io/agent-ros-bridge/).
