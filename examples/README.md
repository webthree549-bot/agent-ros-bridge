# Agent ROS Bridge - Examples

**Ready-to-run examples demonstrating all features.**

## Quick Start

```bash
# Start here - zero dependencies, instant launch
cd quickstart
./run.sh
```

Then open http://localhost:8765 or use `wscat -c ws://localhost:8765`

## Examples Overview

| Example | What It Shows | Runtime | Time to Launch |
|---------|--------------|---------|----------------|
| [quickstart](./quickstart/) | Basic bridge usage | Native Python | 5 seconds |
| [fleet](./fleet/) | Multi-robot coordination | Native Python | 5 seconds |
| [auth](./auth/) | JWT authentication | Native Python | 5 seconds |
| [mqtt_iot](./mqtt_iot/) | IoT sensor integration | Native Python | 5 seconds |
| [actions](./actions/) | ROS navigation/actions | Native Python | 5 seconds |
| [arm](./arm/) | Robotic arm control | Native Python | 5 seconds |
| [metrics](./metrics/) | Prometheus monitoring | Native Python | 5 seconds |

## Running Any Example

```bash
cd <example-name>
./run.sh
```

Each example is **self-contained** and runs without Docker or ROS installation.

## What These Examples Use

All examples run in **mock mode** — simulated robot environments that:
- Respond to real WebSocket/MQTT commands
- Behave like real ROS systems
- Require zero setup

This lets you:
- Learn the API instantly
- Test integrations quickly
- Develop without hardware

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

**Python not found?**
```bash
# Use python3 explicitly
python3 run.sh
```

## Contributing

Add new examples by creating a folder with:
- `README.md` — What it does, how to run
- `run.sh` — Executable launch script
- `*.py` — Example code

See [CONTRIBUTING.md](../CONTRIBUTING.md)
