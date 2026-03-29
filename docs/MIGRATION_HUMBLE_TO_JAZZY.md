# Migration Guide: Humble to Jazzy

**Date:** March 24, 2026  
**Version:** v0.6.5+  
**Status:** Required for Docker users

---

## Overview

As of v0.6.5, Agent ROS Bridge has standardized on **ROS2 Jazzy** (Ubuntu 24.04). This guide helps you migrate from the old Humble-based setup.

## What's Changed

| Component | Old (Humble) | New (Jazzy) |
|-----------|--------------|-------------|
| Container name | `ros2_humble` | `ros2_jazzy` |
| Image name | `agent-ros-bridge:ros2-humble` | `agent-ros-bridge:ros2-jazzy` |
| ROS version | Humble (22.04) | Jazzy (24.04) |
| Package prefix | `ros-humble-*` | `ros-jazzy-*` |
| Script location | Root `docker-manager.sh` | `scripts/docker/docker-manager.sh` |

## Migration Steps

### 1. Stop and Remove Old Container

```bash
# Stop old Humble container
docker stop ros2_humble

# Remove old container
docker rm ros2_humble

# (Optional) Remove old image to save space
docker rmi agent-ros-bridge:ros2-humble
```

### 2. Update Scripts

Pull the latest code:

```bash
git pull origin main
```

### 3. Build New Jazzy Image

```bash
./scripts/docker/build-ros2-image.sh
```

This will create `agent-ros-bridge:ros2-jazzy`.

### 4. Start New Container

```bash
./scripts/docker/docker-manager.sh start
```

Or use the deprecated wrapper (still works):

```bash
./docker-manager.sh start  # redirects to new location
```

### 5. Verify Installation

```bash
# Check container is running
docker ps --filter name=ros2_jazzy

# Verify ROS2 version
docker exec ros2_jazzy bash -c "source /opt/ros/jazzy/setup.bash && ros2 --version"

# Check Nav2
docker exec ros2_jazzy bash -c "ros2 pkg list | grep nav2"
```

## Package Name Changes

When installing additional packages, use the new prefix:

```bash
# Old (Humble)
sudo apt install ros-humble-nav2-bringup

# New (Jazzy)
sudo apt install ros-jazzy-nav2-bringup
```

## Dockerfile Updates

If you have custom Dockerfiles, update the base image:

```dockerfile
# Old
FROM osrf/ros:humble-desktop-full

# New
FROM osrf/ros:jazzy-desktop-full
```

And update package names:

```dockerfile
# Old
RUN apt-get install -y ros-humble-nav2-bringup

# New
RUN apt-get install -y ros-jazzy-nav2-bringup
```

## Troubleshooting

### Container name conflicts

If you get an error about container name already in use:

```bash
# Remove old container
docker rm -f ros2_humble

# Try again
./scripts/docker/docker-manager.sh start
```

### Package not found

Some packages may not be available for Jazzy yet. Check:

```bash
# Search for packages
apt-cache search ros-jazzy | grep <package-name>
```

If not available, you may need to build from source.

### Import errors

If you see `ImportError` for ROS2 modules inside the container:

```bash
# Make sure to source ROS2
source /opt/ros/jazzy/setup.bash

# Or add to your ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

## Benefits of Jazzy

- **Ubuntu 24.04 LTS**: 5-year support cycle
- **Improved Nav2**: Better navigation performance
- **Gazebo Harmonic**: Latest simulation features
- **Better Python support**: Improved rclpy bindings

## Rollback (if needed)

If you need to use Humble temporarily:

```bash
# Old scripts still work with explicit image tag
CONTAINER_NAME=ros2_humble IMAGE_NAME=osrf/ros:humble-desktop ./scripts/docker/docker-manager.sh start
```

Note: The code itself works with both versions, but Docker infrastructure is now Jazzy-focused.

## Questions?

- Check `docs/DOCKER_RESEARCH_AND_PLAN.md` for full details
- Open an issue on GitHub
- See `DOCKER_FIX_SUMMARY.md` for technical details

---

*Last updated: March 24, 2026*
