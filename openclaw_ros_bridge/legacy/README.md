# Legacy v1 Code

This directory contains the legacy v1 codebase preserved for backward compatibility.

## Status

⚠️ **DEPRECATED**: This code is deprecated and will be removed in a future version.

## Migration

Please migrate to v2:

```python
# Old (v1)
from openclaw_ros_bridge import openclaw_server

# New (v2)
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
```

See [Migration Guide](../docs/MIGRATION.md) for details.

## Contents

- `base/` - Base classes and utilities
- `communication/` - TCP server and ROS communicators
- `hal/` - Hardware abstraction layer
- `ros1/` - ROS1 support
- `ros2/` - ROS2 support
- `converter/` - Data converters
- `fault/` - Fault recovery
- `monitor/` - Performance monitoring

## Support Timeline

| Version | Status | Support Until |
|---------|--------|---------------|
| v1.x | Deprecated | December 2026 |
| v2.x | Current | Active |

## Usage

If you must use v1 code temporarily:

```python
from openclaw_ros_bridge.legacy import openclaw_server
from openclaw_ros_bridge.legacy.hal import sensor_hal
```

**Warning:** v1 code will not receive new features. Only critical bug fixes will be applied.

---

For new projects, always use v2:

```python
from openclaw_ros_bridge.gateway_v2 import OpenClawGateway
```
