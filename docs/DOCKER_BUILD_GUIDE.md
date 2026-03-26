# Docker Container Build Guide

## Quick Start (Recommended)

### Option 0: Use Pre-built Image (Fastest - If Available)

If you or someone on your team has already built the image with Nav2 installed:

```bash
# Quick start with pre-built image
./scripts/quickstart-docker.sh

# Or manually:
docker run -d \
  --name ros2_jazzy \
  --privileged \
  -p 8765:8765 \
  -v $(pwd):/workspace:rw \
  agent-ros-bridge:jazzy-with-nav2 \
  bash -c "source /opt/ros/jazzy/setup.bash && tail -f /dev/null"
```

### Option 1: Use Official ROS2 Image (Fastest)

```bash
# 1. Pull the official image (~8 GB download)
docker pull osrf/ros:jazzy-desktop-full

# 2. Run container
docker run -d \
  --name ros2_jazzy \
  --hostname ros2-jazzy \
  --privileged \
  -p 8765:8765 \
  -p 11311:11311 \
  -p 9090:9090 \
  -v $(pwd):/workspace:rw \
  -v /dev/shm:/dev/shm \
  osrf/ros:jazzy-desktop-full \
  bash -c "echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && tail -f /dev/null"

# 3. Enter container
docker exec -it ros2_jazzy bash

# 4. Install Nav2 (inside container)
apt-get update
apt-get install -y ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander ros-jazzy-turtlebot3-gazebo

# 5. Save for later (optional but recommended)
docker commit ros2_jazzy agent-ros-bridge:jazzy-with-nav2

# 6. Test
source /opt/ros/jazzy/setup.bash
python3 /workspace/scripts/test_real_gazebo_integration.py
```

### Option 2: Use Build Script

```bash
# Run interactive build script
./scripts/build-docker-image.sh

# Follow prompts to choose build strategy
```

### Option 3: Build from Dockerfile

```bash
# Using the project's Dockerfile
docker build -f docker/Dockerfile.ros2.jazzy -t agent-ros-bridge:ros2-jazzy .

# Or the simulation Dockerfile
docker build -f docker/Dockerfile.simulation -t agent-ros-bridge:ros2-jazzy .
```

---

## What We Tried Earlier

Earlier attempts failed due to:

1. **Network timeouts** - ROS package repository (packages.ros.org) timing out
2. **Large downloads** - 8+ GB image takes 30-60 minutes to download
3. **Build complexity** - Installing all packages in one layer caused issues

**Solution:** Use official pre-built image + install extras inside container

---

## Recommended Approach

### For Development (Quick)

Use official image + manual package installation:

```bash
# Start with official image
docker run -it --name ros2_dev osrf/ros:jazzy-desktop-full bash

# Inside container, install what you need
apt-get update
apt-get install -y ros-jazzy-nav2-bringup
apt-get install -y ros-jazzy-turtlebot3-gazebo
# ... etc
```

### For Production (CI/CD)

Build custom image once, push to registry:

```bash
# Build
docker build -f docker/Dockerfile.simulation -t agent-ros-bridge:jazzy .

# Tag for registry
docker tag agent-ros-bridge:jazzy ghcr.io/webthree549-bot/agent-ros-bridge:jazzy

# Push
docker push ghcr.io/webthree549-bot/agent-ros-bridge:jazzy
```

---

## Container Management

### Start/Stop/Restart

```bash
# Start
./scripts/docker/docker-manager.sh start

# Or manually
docker start ros2_jazzy

# Stop
docker stop ros2_jazzy

# Restart
docker restart ros2_jazzy
```

### Enter Container

```bash
# Interactive shell
docker exec -it ros2_jazzy bash

# Or use script
./scripts/docker/docker-manager.sh shell
```

### Check Status

```bash
./scripts/docker/docker-manager.sh status

# Or
docker ps --filter name=ros2_jazzy
docker logs ros2_jazzy
```

---

## Validation

After building/starting container:

```bash
# Enter container
docker exec -it ros2_jazzy bash

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Check ROS2 version
ros2 --version

# Check Nav2
ros2 pkg list | grep nav2

# Run integration test
python3 /workspace/scripts/test_real_gazebo_integration.py
```

---

## Saving Your Container

After installing packages inside the container, save it as an image to avoid reinstalling:

```bash
# 1. Exit container (keep it running)
exit

# 2. Commit container to new image
docker commit ros2_jazzy agent-ros-bridge:jazzy-with-nav2

# 3. Verify
docker images | grep agent-ros-bridge

# 4. Stop old container
docker stop ros2_jazzy
docker rm ros2_jazzy

# 5. Use saved image anytime
docker run -d \
  --name ros2_jazzy \
  --privileged \
  -p 8765:8765 \
  -v /Users/webthree/.openclaw/workspace/agent-ros-bridge:/workspace:rw \
  agent-ros-bridge:jazzy-with-nav2 \
  bash -c "source /opt/ros/jazzy/setup.bash && tail -f /dev/null"
```

This saves ~30 minutes of package installation every time!

## Troubleshooting

### "No space left on device"

```bash
# Clean up old images
docker system prune -a

# Check disk space
df -h
```

### "Connection timed out" during build

```bash
# Use mirror or retry
docker pull osrf/ros:jazzy-desktop-full

# If packages.ros.org fails, try again later
```

### "Container already exists"

```bash
# Remove old container
docker rm -f ros2_jazzy

# Then recreate
```

### "Permission denied"

```bash
# Run with sudo or add user to docker group
sudo usermod -aG docker $USER
# Log out and back in
```

---

## Image Sizes

| Image | Size | Use Case |
|-------|------|----------|
| `ros:jazzy-ros-base` | ~1 GB | Minimal, headless |
| `osrf/ros:jazzy-desktop` | ~4 GB | With GUI tools |
| `osrf/ros:jazzy-desktop-full` | ~8 GB | Full Gazebo + Nav2 |
| Custom build | ~10 GB | With all extras |

---

## Next Steps

Once container is running:

1. **Test Gazebo P0 features:**
   ```bash
   python3 scripts/test_real_gazebo_integration.py
   ```

2. **Start Gazebo simulation:**
   ```bash
   gz sim -s -r &
   ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True
   ```

3. **Run real scenario validation:**
   ```bash
   python3 scripts/run_gate2_validation.py --scenarios 100 --real-gazebo
   ```

---

## Files

- `scripts/build-docker-image.sh` - Interactive build script
- `docker/Dockerfile.ros2.jazzy` - Jazzy base Dockerfile
- `docker/Dockerfile.simulation` - Full simulation Dockerfile
- `scripts/docker/docker-manager.sh` - Container management

---

*Last updated: March 25, 2026*
