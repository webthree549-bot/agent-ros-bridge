# Quickstart Example

**The fastest way to see Agent ROS Bridge in action.**

## What It Does

Runs a simulated robot bridge that responds to WebSocket commands. No ROS installation required.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh
```

Or manually:
```bash
python mock_bridge.py
```

## Test

```bash
# In another terminal
wscat -c ws://localhost:8765

# Then type:
{"command": {"action": "list_robots"}}
{"command": {"action": "ping"}}
```

## What's Happening

This runs the bridge in **mock mode** — a simulated robot environment that behaves like real ROS but without any ROS installation. Perfect for:
- First-time users
- Testing the bridge API
- Development without hardware

## Next Steps

- Try the [fleet example](../fleet/) for multi-robot
- Try the [auth example](../auth/) for JWT security
- See [Docker vs Native](../../docs/DOCKER_VS_NATIVE.md) for deployment options
