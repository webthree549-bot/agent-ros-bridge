# Version Agnostic Architecture

This document describes how `openclaw-ros-bridge` achieves version agnosticism for both ROS and OpenClaw.

## Overview

The framework is designed to work seamlessly across:
- **ROS Versions**: ROS1 Noetic, ROS2 Humble, ROS2 Jazzy (and future distributions)
- **OpenClaw Versions**: v1.x, v2.x

Without requiring code changes when switching versions.

## How It Works

### 1. Version Detection (Single Source of Truth)

**File**: `openclaw_ros_bridge/version/version_manager.py`

The `VersionManager` singleton detects versions at runtime:

```python
# Environment variables (highest priority)
export ROS_DISTRO=jazzy          # humble, jazzy, noetic
export ROS_TYPE=ros2             # ros1, ros2
export OPENCLAW_VERSION=v2       # v1, v2
export MOCK_MODE=true            # true, false

# Auto-detection (if env vars not set)
- Checks /opt/ros/{distro}/setup.bash existence
- Reads ROS_DISTRO from environment
- Falls back to defaults from config/
```

### 2. ROS Version Agnosticism

**Communication Layer**: `openclaw_ros_bridge/communication/`

```python
# Unified API - same code works for ROS1/ROS2
from openclaw_ros_bridge import get_ros_communicator

ros_comm = get_ros_communicator()  # Auto-detects ROS1/ROS2
ros_comm.publish("/topic", msg_type, data)
ros_comm.subscribe("/topic", msg_type, callback)
```

**Implementation**:
- `ros1_communicator.py` - Wraps rospy
- `ros2_communicator.py` - Wraps rclpy
- `get_ros_communicator()` - Returns correct communicator based on VersionManager

**Message Conversion**: `msg_converter.py`
- Handles ROS1/ROS2 message format differences
- Auto-detects ROS version for proper serialization

### 3. OpenClaw Version Agnosticism

**Data Converter**: `openclaw_ros_bridge/converter/data_converter.py`

```python
# Auto-adapts to OpenClaw v1.x or v2.x
converter = data_converter  # Singleton, auto-configured

# v1.x: Adds prefix to data keys (e.g., "oc_temp" instead of "temp")
# v2.x: Uses raw keys (e.g., "temp")
json_data = converter.ros2oc(ros_msg, "sensor_data")
```

**Configuration**: `config/openclaw_config.yaml`
```yaml
default_version: "v2"
openclaw_versions:
  v1:
    tcp_port: 8888
    recv_buffer_size: 4096
    reconnect_attempts: 10
  v2:
    tcp_port: 9999
    recv_buffer_size: 8192
    reconnect_attempts: 15
```

### 4. HAL (Hardware Abstraction Layer)

**Files**: `openclaw_ros_bridge/hal/`

The HAL auto-detects hardware from config:
```python
sensor_hal.read("env")  # Works with DHT22, BME280, DS18B20, etc.
actuator_hal.write({"fan": True})  # Works with any actuator
```

Hardware model is resolved from:
1. `HAL_HARDWARE` environment variable
2. Config file (`config/hal_config.yaml`)
3. Auto-detection (if supported)
4. Falls back to mock mode

## Usage Examples

### Switching ROS Versions

```bash
# ROS2 Jazzy
export ROS_DISTRO=jazzy
./scripts/run_demo.sh --greenhouse

# ROS2 Humble
export ROS_DISTRO=humble
./scripts/run_demo.sh --greenhouse

# ROS1 Noetic
export ROS_DISTRO=noetic
./scripts/run_demo.sh --greenhouse
```

**Same code, different ROS version** - no modifications needed.

### Switching OpenClaw Versions

```bash
# OpenClaw v2.x
export OPENCLAW_VERSION=v2
./scripts/run_demo.sh --greenhouse

# OpenClaw v1.x
export OPENCLAW_VERSION=v1
./scripts/run_demo.sh --greenhouse
```

### Docker (Isolated Versions)

```bash
# ROS2 Jazzy + OpenClaw v2
docker-compose -f docker/docker-compose.yml up ros2-jazzy-bridge

# ROS2 Humble + OpenClaw v2
docker-compose -f docker/docker-compose.yml up ros2-humble-bridge

# ROS1 Noetic + OpenClaw v1
docker-compose -f docker/docker-compose.yml up ros1-bridge
```

## Adding New ROS Distributions

To add support for a new ROS distribution (e.g., ROS2 K-Turtle):

1. **Add config** in `config/ros2_config.yaml`:
```yaml
ros2_versions:
  kturtle:
    env_path: "/opt/ros/kturtle/setup.bash"
    build_cmd: "colcon build --symlink-install"
    node_prefix: "ros2_"
```

2. **Create Dockerfile** (optional): `docker/Dockerfile.ros2.kturtle`

3. **Update version manager** (if needed): Add "kturtle" to valid distros list

**No business logic code changes required** - the framework auto-adapts.

## Adding New OpenClaw Versions

To add support for OpenClaw v3.x:

1. **Add config** in `config/openclaw_config.yaml`:
```yaml
openclaw_versions:
  v3:
    tcp_host: "127.0.0.1"
    tcp_port: 10000
    recv_buffer_size: 16384
    reconnect_attempts: 20
```

2. **Set version**: `export OPENCLAW_VERSION=v3`

The data converter will auto-detect and handle v3 format.

## Testing Version Agnosticism

```bash
# Test with different ROS versions
for distro in noetic humble jazzy; do
    export ROS_DISTRO=$distro
    export MOCK_MODE=true
    python3 -c "from openclaw_ros_bridge import version_manager; print(f'ROS: {version_manager.ROS_TYPE} {version_manager.ROS_DISTRO}')"
done

# Test with different OpenClaw versions
for ver in v1 v2; do
    export OPENCLAW_VERSION=$ver
    python3 -c "from openclaw_ros_bridge import data_converter; print(f'OpenClaw: {data_converter.oc_version}')"
done
```

## Architecture Benefits

1. **Single Codebase**: Same Python code runs on ROS1/ROS2/OpenClaw v1/v2
2. **Easy Testing**: Mock mode works across all versions
3. **Future-Proof**: New versions only need config additions
4. **No Vendor Lock-in**: Switch ROS/OpenClaw versions without rewrites
5. **CI/CD Friendly**: Test matrix across versions automatically

## Troubleshooting

**Issue**: Version not detected correctly
```bash
# Check detection
python3 -c "
from openclaw_ros_bridge.version.version_manager import version_manager
print(f'ROS: {version_manager.ROS_TYPE} {version_manager.ROS_DISTRO}')
print(f'OpenClaw: {version_manager.OC_VER}')
print(f'Mock: {version_manager.MOCK_MODE}')
"
```

**Issue**: Need to force specific version
```bash
export ROS_DISTRO_MANUAL=jazzy  # Override auto-detection
export OPENCLAW_VERSION=v2      # Force OpenClaw v2
```

## Summary

The framework achieves version agnosticism through:
- **Runtime detection** via `VersionManager` singleton
- **Unified APIs** that hide ROS1/ROS2 differences
- **Config-driven** version-specific settings
- **Strategy pattern** for different protocol versions
- **Environment variables** for easy version switching

Write once, run anywhere (ROS1/ROS2/OpenClaw v1/v2).
