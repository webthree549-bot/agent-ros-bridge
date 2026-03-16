# Docker Strategy for Agent ROS Bridge

## Overview

This project uses Docker for ROS2 integration testing. E2E tests require a local Docker setup and are **skipped in CI** by design.

## Philosophy

- **Unit Tests**: Run everywhere (local, CI) - no Docker required
- **Integration Tests**: Run everywhere - use mocks/stubs
- **E2E Tests**: Run locally only - require Docker setup

## Quick Start

### 1. Build the ROS2 Docker Image

```bash
./scripts/docker/build-ros2-image.sh
```

This creates a fixed `agent-ros-bridge:ros2-humble` image with:
- ROS2 Humble base
- Nav2 navigation stack
- Required message packages

### 2. Start the ROS2 Container

```bash
./scripts/docker/start-ros2.sh
```

Or use the Docker manager:

```bash
./scripts/docker-manager.sh start
```

### 3. Run E2E Tests

```bash
# Run all E2E tests
./scripts/test-e2e.sh

# Run specific E2E test
python -m pytest tests/e2e/test_agent_ros_bridge_e2e.py -v
```

### 4. Stop the Container

```bash
./scripts/docker-manager.sh stop
```

## Docker Image Strategy

### Fixed Base Image

We use a **fixed base image** approach:

1. **Base Image**: `agent-ros-bridge:ros2-humble-base`
   - Built once, cached in CI
   - Contains ROS2 Humble + Nav2
   - Rarely changes

2. **Dev Image**: `agent-ros-bridge:ros2-humble-dev`
   - Extends base image
   - Mounts source code as volume
   - Used for development

### Building Images

```bash
# Build base image (rarely needed)
docker build -f docker/Dockerfile.ros2-base -t agent-ros-bridge:ros2-humble-base .

# Build dev image
docker build -f docker/Dockerfile.ros2-dev -t agent-ros-bridge:ros2-humble-dev .
```

## CI/CD Strategy

### GitHub Actions

- **Unit Tests**: Run on every push/PR
- **Integration Tests**: Run on every push/PR
- **E2E Tests**: **SKIPPED** in CI (require local Docker)

### Why Skip E2E in CI?

1. **Docker-in-Docker complexity**: Running ROS2 in Docker within CI is fragile
2. **Resource intensive**: Gazebo + Nav2 requires significant CPU/GPU
3. **Flaky tests**: Simulation timing varies in CI environments
4. **Local-first design**: E2E tests are for development validation

## Testing Locally

### Prerequisites

- Docker 20.10+
- 4GB+ RAM available for container
- (Optional) NVIDIA Docker runtime for GPU support

### Running Tests

```bash
# 1. Start ROS2 container
./scripts/docker-manager.sh start

# 2. Run E2E tests
pytest tests/e2e/ -v

# 3. Stop container
./scripts/docker-manager.sh stop
```

### Development Workflow

```bash
# Terminal 1: Start ROS2 container
./scripts/docker-manager.sh start

# Terminal 2: Run tests as you develop
pytest tests/e2e/test_navigation_e2e.py -v -k test_nav2

# Terminal 3: Interactive ROS2 shell
./scripts/docker/exec-ros2.sh
```

## Troubleshooting

### Container not starting

```bash
# Check Docker daemon
docker ps

# Check logs
docker logs ros2_humble

# Rebuild image
./scripts/docker/build-ros2-image.sh --no-cache
```

### Nav2 not found

```bash
# Enter container and check
docker exec -it ros2_humble bash
ros2 pkg list | grep nav2

# If missing, rebuild image
./scripts/docker/build-ros2-image.sh
```

### Tests skipping when they shouldn't

```bash
# Check container status
docker ps --filter name=ros2_humble

# Verify Nav2 installation
docker exec ros2_humble bash -c "ros2 pkg list | grep nav2"
```

## File Structure

```
docker/
├── Dockerfile.ros2-base      # Fixed base image
├── Dockerfile.ros2-dev       # Development image
└── docker-compose.yml        # Multi-container setup

scripts/
├── docker/
│   ├── build-ros2-image.sh   # Build ROS2 image
│   └── start-ros2.sh         # Start container
├── docker-manager.sh         # Manage Docker lifecycle
└── test-e2e.sh              # Run E2E tests

tests/
├── unit/                    # Unit tests (no Docker)
├── integration/             # Integration tests (no Docker)
└── e2e/                     # E2E tests (Docker required)
    ├── test_agent_ros_bridge_e2e.py
    ├── test_gazebo_e2e.py
    ├── test_navigation_e2e.py
    └── test_ros2_real_bridge.py
```

## Advanced Usage

### Custom ROS2 Packages

Add packages to `docker/Dockerfile.ros2-base`:

```dockerfile
RUN apt-get install -y \
    ros-humble-your-package \
    ros-humble-another-package
```

Then rebuild:

```bash
./scripts/docker/build-ros2-image.sh
```

### GPU Support

For NVIDIA GPU support in Gazebo:

```bash
# Install nvidia-docker2
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

# Run with GPU
./scripts/docker-manager.sh start --gpu
```

### Multiple Containers

For fleet testing:

```bash
docker-compose -f docker/docker-compose.yml up -d
```

This starts multiple ROS2 containers for multi-robot testing.

## Maintenance

### Updating ROS2 Version

1. Update `docker/Dockerfile.ros2-base`
2. Change base image tag
3. Rebuild: `./scripts/docker/build-ros2-image.sh`
4. Update this documentation

### Cleaning Up

```bash
# Stop and remove container
./scripts/docker-manager.sh stop

# Remove image
docker rmi agent-ros-bridge:ros2-humble-base

# Clean all
docker system prune -a
```
