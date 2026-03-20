# Docker Testing Guide for Agent ROS Bridge

## Overview

AI modules require ROS2 (rclpy) which is only available inside the Docker container. This document explains how to run tests with ROS2 dependencies.

## Container Status

```bash
# Check if container is running
docker ps | grep ros2_humble

# Current status: ✅ Running (Up 38+ hours)
```

## Running Tests in Docker

### 1. Basic Test Run
```bash
# Source ROS2 and run tests
docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/ -v"
```

### 2. With Coverage (requires pytest-cov install)
```bash
# Install pytest-cov first
docker exec ros2_humble bash -c "pip3 install --break-system-packages pytest-cov"

# Run with coverage
docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/ -v --cov=agent_ros_bridge.ai"
```

### 3. Specific Test File
```bash
docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/test_intent_parser.py -v"
```

## Docker Configuration

### Image Details
- **Image:** `agent-ros-bridge:ros2-humble`
- **Base:** `ros:jazzy-ros-base`
- **ROS Distro:** Jazzy
- **Python:** 3.12

### Installed ROS2 Packages
- `ros-jazzy-demo-nodes-cpp`
- `ros-jazzy-demo-nodes-py`
- `ros-jazzy-nav2-bringup`
- `ros-jazzy-nav2-simple-commander`

### Container Setup
- **Name:** `ros2_humble`
- **Volume:** Project mounted at `/workspace` (read-only)
- **Working Dir:** `/workspace`

## Managing the Container

```bash
# Start container
./scripts/docker-manager.sh start

# Stop container
./scripts/docker-manager.sh stop

# Enter container shell
./scripts/docker-manager.sh shell

# Build image
./scripts/docker-manager.sh build

# View logs
./scripts/docker-manager.sh logs
```

## Test Results Summary

### AI Module Tests (in Docker)
- **test_intent_parser.py:** 10 tests (some passing)
- **test_motion_primitives.py:** 15 tests (some passing)

### Issues Found
1. Pytest config `asyncio_mode` not recognized in container
2. Some API mismatches between tests and implementation
3. Need to install pytest-cov for coverage

## Next Steps

1. Fix API mismatches in AI tests
2. Install pytest-cov in Docker image
3. Update Dockerfile to include test dependencies
4. Run full test suite in container for accurate coverage

## Quick Commands

```bash
# Check container status
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" | grep ros2

# Run E2E tests
./scripts/test-e2e.sh

# Run unit tests (local, no ROS2)
pytest tests/unit/ -v --ignore=tests/unit/ai/

# Run AI tests (in Docker)
docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 -m pytest tests/unit/ai/ -v"
```
