# OpenClaw-ROS Bridge Architecture

This document explains how the project works end-to-end.

## Overview

OpenClaw-ROS Bridge is a **version-agnostic robotics middleware** that connects OpenClaw AI agents with ROS1/ROS2 ecosystems. It provides seamless hardware integration, real-time scheduling, fault resilience, and full-stack observability.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         OpenClaw AI Agent                               │
│                    (External AI system / TCP)                           │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │ JSON over TCP
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     OpenClaw ROS Bridge                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌───────────┐ │
│  │   ROS2      │    │   ROS1      │    │    HAL      │    │  Version  │ │
│  │Communicator │    │Communicator │    │(Hardware    │    │  Manager  │ │
│  └─────────────┘    └─────────────┘    │ Abstraction)│    │           │ │
│                                        └─────────────┘    └───────────┘ │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                  │
│  │   Plugin    │    │   Fault     │    │  Monitor    │                  │
│  │   System    │    │  Recovery   │    │   & Logs    │                  │
│  └─────────────┘    └─────────────┘    └─────────────┘                  │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│              Physical Hardware                                          │
│     (Sensors, Motors, Arms, Cameras, etc.)                              │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 1. Version Manager (The Brain)

**File:** `openclaw_ros_bridge/version/version_manager.py`

The Version Manager is a **Singleton** that auto-detects and manages all version configurations at runtime.

### What it does:
- **Auto-detects ROS version** from environment (`ROS_DISTRO`)
- **Loads YAML configs** from `config/` directory:
  - `ros1_config.yaml` - ROS1 Noetic settings
  - `ros2_config.yaml` - ROS2 Humble/Jazzy settings  
  - `openclaw_config.yaml` - TCP ports, timeouts
  - `hal_config.yaml` - Hardware abstraction settings
  - `fault_config.yaml` - Recovery policies
  - `global_config.yaml` - Shared settings
- **Resolves versions** at runtime:
  - ROS type: `ros1` or `ros2`
  - ROS distro: `noetic`, `humble`, or `jazzy`
  - OpenClaw version: `v1` or `v2`
  - HAL hardware: `auto`, `dht22`, `bme280`, `robotiq_2f_85`, etc.
  - Mock mode: `true` or `false`

### Usage:
```python
from openclaw_ros_bridge.version.version_manager import version_manager

print(f"ROS: {version_manager.ROS_TYPE} {version_manager.ROS_DISTRO}")
print(f"OpenClaw: {version_manager.OC_VER}")
print(f"Mock Mode: {version_manager.MOCK_MODE}")

# Get ROS-specific config
topic_map = version_manager.get_ros_param("topic_map", {})
```

---

## 2. Communication Layer (The Translator)

**Files:**
- `openclaw_ros_bridge/communication/ros2_communicator.py`
- `openclaw_ros_bridge/communication/ros1_communicator.py`
- `openclaw_ros_bridge/communication/openclaw_communicator.py`

The communication layer provides a **unified API** regardless of ROS version.

### ROS2 Communicator
```python
from openclaw_ros_bridge.communication import get_ros_communicator

ros_comm = get_ros_communicator()

# Subscribe to topic
ros_comm.subscribe("/sensor/temperature", Float32, callback)

# Publish to topic
ros_comm.publish("/actuator/fan", Bool, {"data": True})
```

### Key Features:
- **Lazy initialization** - ROS node only created when first used
- **Topic caching** - Publishers/subscribers reused per topic
- **QoS profiles** - Configurable quality of service
- **Conditional imports** - ROS1 and ROS2 communicators loaded based on environment

---

## 3. HAL - Hardware Abstraction Layer

**Files:**
- `openclaw_ros_bridge/hal/sensor_hal.py`
- `openclaw_ros_bridge/hal/actuator_hal.py`
- `openclaw_ros_bridge/hal/base_hal.py`

HAL abstracts hardware differences behind unified interfaces.

### Sensor HAL
```python
from openclaw_ros_bridge.hal import sensor_hal

# Initialize (auto-detects hardware from config)
sensor_hal.init_hardware()

# Read sensor data
data = sensor_hal.read("env")  # Returns: {"temperature": 25.0, "humidity": 50.0}
```

### Actuator HAL
```python
from openclaw_ros_bridge.hal import actuator_hal

# Initialize
actuator_hal.init_hardware()

# Control actuators
actuator_hal.write({"fan": True, "valve": False})

# Emergency stop
actuator_hal.safe_state()
```

### Key Features:
- **Mock mode** - Generates fake data when `MOCK_MODE=true`
- **Rate limiting** - Prevents excessive sensor polling
- **Auto-detection** - Detects hardware type from config
- **BaseHAL pattern** - Common interface for all hardware types

---

## 4. Plugin System (User Extension Point)

**File:** `openclaw_ros_bridge/plugin_base/base_plugin.py`

Plugins are the main way users extend the framework for specific robots.

### BasePlugin provides:
- **Dependency injection** - Auto-wires ROS comm, HAL, OpenClaw comm
- **Lifecycle management** - `UNINITIALIZED → INITIALIZED → RUNNING → STOPPED`
- **Graceful shutdown** - Safe state on stop

### Example Plugin:
```python
from openclaw_ros_bridge.plugin_base.base_plugin import BasePlugin, PluginStatus

class GreenhousePlugin(BasePlugin):
    def init_plugin(self) -> bool:
        # Custom initialization
        self.fan_threshold = 28.0
        return True
    
    def run(self) -> None:
        # Main loop
        while self.status == PluginStatus.RUNNING:
            # Read sensors
            data = self.sensor_hal.read("env")
            
            # Control logic
            if data["temperature"] > self.fan_threshold:
                self.actuator_hal.write({"fan": True})
            
            # Send to OpenClaw
            self.oc_comm.send(data)
            
            time.sleep(1.0)
```

### Lifecycle:
```
┌─────────────────┐
│  UNINITIALIZED  │
└────────┬────────┘
         │ init_core()
         ▼
┌─────────────────┐
│   INITIALIZED   │
└────────┬────────┘
         │ init_plugin()
         ▼
┌─────────────────┐
│     RUNNING     │ ◄── main loop
└────────┬────────┘
         │ graceful_stop()
         ▼
┌─────────────────┐
│     STOPPED     │
└─────────────────┘
```

---

## 5. Fault Recovery

**Files:**
- `openclaw_ros_bridge/fault/recovery_manager.py`
- `openclaw_ros_bridge/fault/recovery_strategies.py`

Fault recovery uses the **Strategy Pattern** for different failure types.

### Recovery Strategies:
- **ROSRecoveryStrategy** - Reconnects ROS nodes
- **OpenClawRecoveryStrategy** - Reconnects TCP to OpenClaw
- **HALRecoveryStrategy** - Reinitializes hardware

### Usage:
```python
from openclaw_ros_bridge.fault import recovery_manager

# Manual recovery
success = recovery_manager.recover("ros_disconnect")
success = recovery_manager.recover("openclaw_disconnect")
success = recovery_manager.recover("hal_disconnect")
```

### Auto-Recovery:
Recovery is triggered automatically when:
- ROS node dies
- OpenClaw TCP connection drops
- Hardware becomes unresponsive

Configuration in `config/fault_config.yaml`:
```yaml
global:
  recovery_enabled: true
  max_recovery_attempts: 3
```

---

## 6. Monitoring & Observability

**Files:**
- `openclaw_ros_bridge/monitor/performance_monitor.py`
- `openclaw_ros_bridge/monitor/state_monitor.py`
- `openclaw_ros_bridge/monitor/dashboard.py`

### Performance Monitor
Tracks system metrics:
- CPU usage
- Memory usage
- Message latency
- Message throughput

```python
from openclaw_ros_bridge.monitor import perf_monitor

# Track latency
perf_monitor.track_latency("sensor_read")

# Increment message count
perf_monitor.increment_msg_count()

# Get report
metrics = perf_monitor.get_metrics_report()
```

### State Monitor
Tracks component health:
- ROS connection state
- OpenClaw connection state
- Sensor HAL state
- Actuator HAL state

### Dashboard
Real-time text-based display of system status.

---

## 7. Data Flow (Complete Picture)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         OpenClaw AI Agent                               │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │ JSON over TCP
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                     OpenClaw Communicator                               │
│              (Protocol: v1.x vs v2.x auto-detected)                     │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        Plugin (Your Code)                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  Greenhouse │  │    Arm      │  │   Custom    │  │    Base     │    │
│  │    Plugin   │  │   Plugin    │  │   Plugin    │  │   Plugin    │    │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    ▼             ▼             ▼
           ┌────────────┐ ┌────────────┐ ┌────────────┐
           │ Sensor HAL │ │ Actuator   │ │ ROS Comm   │
           │ (read)     │ │ HAL (write)│ │ (pub/sub)  │
           └────────────┘ └────────────┘ └────────────┘
                    │             │             │
                    ▼             ▼             ▼
           ┌──────────────────────────────────────────┐
           │         Physical Hardware                │
           │  (DHT22, Motors, Cameras, Arms, etc.)    │
           └──────────────────────────────────────────┘
```

---

## 8. Demo Execution Flow

```
./scripts/run_demo.sh --greenhouse --mock
         │
         ▼
┌─────────────────┐
│  run_demo.sh    │
│  (detects env)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  ros2 launch    │
│  greenhouse_    │
│  demo.launch.py │
└────────┬────────┘
         │
         ▼
┌───────────────────────────────────────┐
│  Launch sequence:                     │
│  1. core.launch.py → performance_monitor│
│  2. greenhouse_plugin                 │
└────────┬──────────────────────────────┘
         │
         ▼
┌───────────────────────────────────────┐
│  greenhouse_plugin:                   │
│  1. VersionManager() → detects mock   │
│  2. init_core() → ROS node + HAL      │
│  3. init_plugin() → custom init       │
│  4. run() → main loop                 │
│     - read sensor_hal                 │
│     - auto-control actuators          │
│     - sleep(1)                        │
└───────────────────────────────────────┘
```

---

## Key Design Patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **Singleton** | VersionManager, HAL, Monitors | One shared instance across framework |
| **Abstract Base Class** | BasePlugin, BaseRecoveryStrategy | Enforce API contract for extensions |
| **Strategy** | Recovery strategies, ROS1/ROS2 comms | Swappable implementations |
| **Adapter** | ROS1Communicator/ROS2Communicator | Unified API across ROS versions |
| **Factory** | `get_ros_communicator()` | Runtime object creation based on version |

---

## Configuration Hierarchy

Configuration is loaded in this priority order:

1. **Environment Variables** (highest priority)
   - `ROS_DISTRO=jazzy`
   - `MOCK_MODE=true`
   - `OPENCLAW_VERSION=v2`

2. **Config Files** (YAML)
   - `config/global_config.yaml`
   - `config/ros1_config.yaml` or `config/ros2_config.yaml`
   - `config/openclaw_config.yaml`
   - `config/hal_config.yaml`
   - `config/fault_config.yaml`

3. **Hardcoded Defaults** (lowest priority)
   - ROS2 Humble
   - Mock mode disabled
   - OpenClaw v2

---

## Summary

OpenClaw-ROS Bridge provides:

1. **Version Detection** - Auto-detects ROS1/ROS2 at runtime
2. **Hardware Abstraction** - HAL unifies sensors/actuators
3. **Plugin Standardization** - BasePlugin enforces consistent API
4. **Fault Recovery** - Auto-reconnects and recovers from failures
5. **Observability** - Performance and state monitoring
6. **AI Integration** - Seamless OpenClaw TCP communication

The result: Write once, run on any ROS version with any hardware.
