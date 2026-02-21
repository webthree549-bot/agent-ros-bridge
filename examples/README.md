# Agent ROS Bridge - Examples

**Ready-to-run examples demonstrating all features.**

## Quick Start

All examples run in **Docker containers** for security and isolation.

```bash
# Set required JWT secret
export JWT_SECRET=$(openssl rand -base64 32)

# Start here - Docker-based, secure
cd quickstart
docker-compose up
```

## Examples Overview

### 🎮 Playground Examples (Full Demos)

Real-world scenarios with complete Docker setups:

| Example | What It Shows | Runtime | Complexity |
|---------|--------------|---------|------------|
| [playground/talking-garden](./playground/talking-garden/) | LLM talks to IoT plants | Docker | Medium |
| [playground/mars-colony](./playground/mars-colony/) | Multi-robot Mars mission | Docker | High |
| [playground/theater-bots](./playground/theater-bots/) | AI director + robot actors | Docker | Medium |
| [playground/art-studio](./playground/art-studio/) | Human/robot collaborative art | Docker | Medium |

### 🚀 Quick Examples (Feature Demos)

Focused examples showing specific features:

| Example | What It Shows | Runtime | Time to Launch |
|---------|--------------|---------|----------------|
| [quickstart](./quickstart/) | Basic bridge usage | Docker | 10 seconds |
| [fleet](./fleet/) | Multi-robot coordination | Docker | 10 seconds |
| [auth](./auth/) | JWT authentication | Docker | 10 seconds |
| [arm](./arm/) | Robotic arm control | Docker | 10 seconds |

## Running Any Example

```bash
cd <example-name>

# Set JWT secret (required)
export JWT_SECRET=$(openssl rand -base64 32)

# Run in Docker
docker-compose up
```

Each example:
- Runs in an **isolated Docker container**
- **Requires JWT_SECRET** (authentication always enforced)
- Uses **simulated robots** (no ROS installation needed)
- Binds to **localhost only** by default

## What These Examples Use

All examples use **simulated robot environments** in Docker containers:
- Respond to real WebSocket/MQTT commands
- Behave like real ROS systems
- Require JWT authentication (no exceptions)
- Run in network isolation

This lets you:
- Learn the API securely
- Test integrations quickly
- Develop without hardware

## Security First

**JWT_SECRET is always required.** Examples will fail to start without it.

```bash
# Generate a secure secret
export JWT_SECRET=$(openssl rand -base64 32)
```

## Production Deployment

When you're ready for real robots:

| Deployment | See |
|------------|-----|
| Native Ubuntu + ROS | [NATIVE_ROS.md](../docs/NATIVE_ROS.md) |
| Docker containers | [DOCKER_VS_NATIVE.md](../docs/DOCKER_VS_NATIVE.md) |
| Cloud (AWS/GCP) | [User Manual - Deployment](../docs/USER_MANUAL.md) |

## Troubleshooting

**Port already in use?**
```bash
# Find and kill process
lsof -i :8765
kill <PID>
```

**Docker not running?**
```bash
# Start Docker Desktop
docker ps
```

## Contributing

Add new examples by creating a folder with:
- `README.md` — What it does, how to run
- `docker-compose.yml` — Docker orchestration
- `run.sh` — Executable launch script (optional)
- `*.py` — Example code

See [CONTRIBUTING.md](../CONTRIBUTING.md)
