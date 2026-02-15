# Quickstart Example

**The fastest way to see Agent ROS Bridge in action.**

## What It Does

Runs a simulated robot bridge in an isolated Docker container that responds to WebSocket commands. No ROS installation required.

## Requirements

- Docker Desktop
- JWT_SECRET environment variable

## Run

```bash
# Set required JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Run in Docker (isolated, secure)
docker-compose up

# Or run the Python script directly (requires JWT_SECRET)
export JWT_SECRET=$(openssl rand -base64 32)
python simulated_robot.py
```

## Test

```bash
# Generate a token
python ../../scripts/generate_token.py --secret $JWT_SECRET --role operator

# Connect with token
wscat -c "ws://localhost:8765?token=YOUR_TOKEN_HERE"

# Then type:
{"command": {"action": "list_robots"}}
{"command": {"action": "ping"}}
```

## What's Happening

This runs the bridge with a **simulated robot** — a safe testing environment that behaves like real ROS but without any ROS installation. JWT authentication is always enforced. Perfect for:
- First-time users
- Testing the bridge API
- Development without hardware

## Security Note

This example requires JWT_SECRET and uses authentication. The Docker container provides network isolation.

## Next Steps

- Try the [fleet example](../fleet/) for multi-robot
- Try the [auth example](../auth/) for JWT security examples
- See [Docker vs Native](../../docs/DOCKER_VS_NATIVE.md) for deployment options
