# Docker vs Native ROS Deployment

## Quick Comparison

| Factor | Docker | Native |
|--------|--------|--------|
| **Setup** | One command: `docker-compose up` | Install ROS, dependencies, configure |
| **Isolation** | ✅ Clean — no system conflicts | ❌ Can conflict with system ROS |
| **ROS Versions** | ✅ Run ROS1 + ROS2 side-by-side | ⚠️ Complex (dual ROS setup) |
| **Performance** | ⚠️ 5-10% overhead, networking | ✅ Native speed, direct hardware |
| **Hardware Access** | ⚠️ USB/serial needs `--device` | ✅ Direct access |
| **Development** | ⚠️ Edit → rebuild → restart | ✅ Edit → run immediately |
| **Debugging** | ⚠️ Attach to container | ✅ Direct, familiar tools |
| **CI/CD** | ✅ Reproducible, version locked | ⚠️ Environment dependent |
| **Production** | ⚠️ Extra orchestration (K8s) | ✅ Direct deployment |

## When to Use Each

### Docker — Best For
- **Getting started quickly** — No ROS installation needed
- **CI/CD testing** — Reproducible builds
- **Multi-ROS environments** — ROS1 + ROS2 containers
- **Team consistency** — Same environment everywhere
- **Mac/Windows users** — ROS is Linux-only natively

### Native — Best For
- **Production deployment** — Direct hardware, no overhead
- **Development workflow** — Fast iteration, IDE integration
- **Performance-critical** — Real-time robotics, low latency
- **Hardware integration** — Direct GPU, FPGA, sensor access
- **Existing ROS infrastructure** — Already have ROS installed

## Recommended Workflow

```
Development          Testing              Production
    │                    │                    │
    ▼                    ▼                    ▼
┌─────────┐        ┌─────────┐          ┌─────────┐
│  Native │   ──▶   │ Docker  │    ──▶   │  Native │
│  Ubuntu │        │   CI    │          │  Ubuntu │
│  + ROS  │        │  Tests  │          │  Server │
└─────────┘        └─────────┘          └─────────┘
   Fast edit        Reproducible        Direct hardware
   Debug easy       Multi-ROS tests      No overhead
```

## Your Project Supports Both

| Command | Use Case |
|---------|----------|
| `python demo/mock_bridge.py` | Development, testing logic |
| `docker-compose up ros2-bridge` | Quick start, demos |
| `python run_bridge.py` + native ROS | Production deployment |

## Quick Start Guide

### Docker (Recommended for First-Time Users)

```bash
# Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# Start ROS2 bridge in Docker
docker-compose --profile ros2 up ros2-bridge

# Test connection
wscat -c ws://localhost:8767
> {"command": {"action": "ping"}}
```

### Native (Recommended for Production)

See [NATIVE_ROS.md](NATIVE_ROS.md) for detailed setup.

```bash
# Install ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# Install bridge
pip install agent-ros-bridge

# Start bridge
source /opt/ros/humble/setup.bash
python -m agent_ros_bridge

# Test connection
wscat -c ws://localhost:8765
> {"command": {"action": "list_robots"}}
```

## Decision Matrix

| Scenario | Recommendation |
|----------|----------------|
| Just trying it out | **Docker** — No installation hassle |
| Developing new features | **Native** — Fast iteration |
| Running CI/CD tests | **Docker** — Reproducible |
| Production deployment | **Native** — Performance + hardware |
| Multi-robot fleet | **Both** — Docker for cloud, Native for edge |
| Windows/Mac user | **Docker** — ROS is Linux-only |

## Performance Considerations

### Docker Overhead
- **Network**: ~1-2ms additional latency
- **CPU**: ~5% overhead for most workloads
- **Memory**: Container overhead ~50-100MB
- **Real-time**: Not suitable for hard real-time requirements

### When Native Wins
- High-frequency control loops (>100Hz)
- Direct GPU acceleration (CUDA, TensorRT)
- Real-time kernel requirements (PREEMPT_RT)
- USB device passthrough complexity

## Migration Path

You can move from Docker to Native seamlessly:

1. **Develop in Docker** — Test logic, APIs
2. **Validate with Native** — Use validation script
3. **Deploy to Production** — Native on Ubuntu server

The bridge code is identical — only the environment changes.

## Validation

Before production deployment, validate your native ROS setup:

```bash
# Run validation script
python scripts/validate_ros_setup.py

# Expected output:
# ✅ ROS2 Humble detected
# ✅ rclpy available
# ✅ Bridge imports successfully
# ✅ Test node creation OK
# ✅ All validations passed!
```

See [NATIVE_ROS.md](NATIVE_ROS.md) for full validation steps.
