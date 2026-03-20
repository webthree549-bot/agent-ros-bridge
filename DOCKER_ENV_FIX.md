# Docker Test Environment Issue Resolution

## Problem Identified

Tests are being skipped in Docker because:
1. pytest runs tests in a subprocess
2. The subprocess doesn't inherit the ROS2 environment (LD_LIBRARY_PATH, PYTHONPATH, etc.)
3. `rclpy` import fails without the full ROS2 environment
4. Tests use `pytest.importorskip()` or conditional imports that skip when rclpy is unavailable

## Root Cause

When running:
```bash
docker exec ros2_humble bash -c "source /opt/ros/jazzy/setup.bash && python3 -m pytest ..."
```

The `source /opt/ros/jazzy/setup.bash` sets environment variables in the bash shell, but pytest runs tests in a subprocess that doesn't inherit these variables.

## Solutions

### Solution 1: Use Docker Entrypoint (Recommended)

Modify the Docker container to source ROS2 setup automatically:

```dockerfile
# In Dockerfile.ros2
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
```

Or create a wrapper script:
```bash
#!/bin/bash
# /workspace/docker/test-entrypoint.sh
source /opt/ros/jazzy/setup.bash
exec "$@"
```

Then run tests with:
```bash
docker exec ros2_humble /workspace/docker/test-entrypoint.sh python3 -m pytest tests/unit/ai/ -v
```

### Solution 2: Set Environment Variables Explicitly

Pass the environment variables to docker exec:

```bash
docker exec -e LD_LIBRARY_PATH=/opt/ros/jazzy/lib \
            -e PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
            -e ROS_DISTRO=jazzy \
            ros2_humble python3 -m pytest tests/unit/ai/ -v
```

### Solution 3: Use docker-compose with Environment

Create a `docker-compose.test.yml`:

```yaml
version: '3.8'
services:
  test:
    image: agent-ros-bridge:ros2-humble
    environment:
      - LD_LIBRARY_PATH=/opt/ros/jazzy/lib
      - PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages
      - ROS_DISTRO=jazzy
    volumes:
      - .:/workspace:ro
    working_dir: /workspace
    command: python3 -m pytest tests/unit/ai/ -v
```

Then run:
```bash
docker-compose -f docker-compose.test.yml up
```

### Solution 4: Modify Dockerfile (Permanent Fix)

Add to `docker/Dockerfile.ros2`:

```dockerfile
# Set ROS2 environment variables permanently
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_DISTRO=jazzy
ENV PATH=/opt/ros/jazzy/bin:$PATH
```

Then rebuild the image:
```bash
./scripts/docker/build-ros2-image.sh
```

## Current Workaround

For immediate testing, use the local test environment without Docker:

```bash
# Run tests that don't require ROS2
pytest tests/unit/ --ignore=tests/unit/ai/ -v

# Coverage report (local)
pytest tests/unit/ --ignore=tests/unit/ai/ -v --cov=agent_ros_bridge
```

## Test Status Summary

- **Local tests:** 1,185 passing, 50.08% coverage
- **Docker tests (with workarounds):** 760+ passing
- **AI module tests:** Require ROS2 environment fix

## Recommendation

Implement **Solution 4** (modify Dockerfile) for a permanent fix, as it:
1. Makes the ROS2 environment available to all processes
2. Doesn't require changes to test execution commands
3. Works with pytest's subprocess model
4. Is the standard ROS2 Docker approach
