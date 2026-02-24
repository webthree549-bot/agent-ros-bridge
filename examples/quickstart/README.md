# Quickstart Example

**The fastest way to see Agent ROS Bridge in action — no ROS installation needed.**

## What It Does

Runs a simulated robot bridge in an isolated Docker container that responds to WebSocket commands.

## Requirements

- Docker Desktop
- `JWT_SECRET` environment variable

## Run

```bash
# Set required JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Option 1: Docker (isolated, recommended)
docker-compose up

# Option 2: Direct Python (JWT_SECRET must be set)
pip install agent-ros-bridge websockets
python simulated_robot.py
```

## Test

```bash
# Generate an auth token
python ../../scripts/generate_token.py --secret $JWT_SECRET --role operator

# Connect via wscat (npm install -g wscat)
wscat -c "ws://localhost:8766?token=<YOUR_TOKEN>"

# Try commands:
{"command": {"action": "list_robots"}}
{"command": {"action": "get_topics"}}
{"command": {"action": "move", "parameters": {"direction": "forward", "distance": 1.0}}}
```

## What's Happening

The `simulated_robot.py` script starts a `Bridge` with a `SimulatedRobotPlugin` — a safe
testing environment that behaves like a real ROS robot without any ROS installation.

## Next Steps

- [fleet/](../fleet/) — multi-robot fleet orchestration
- [auth/](../auth/) — JWT security examples
- [arm/](../arm/) — robot arm control
- [docs/NATIVE_ROS.md](../../docs/NATIVE_ROS.md) — connecting to a real ROS robot
