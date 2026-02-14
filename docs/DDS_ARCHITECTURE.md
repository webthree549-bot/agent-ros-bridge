# ROS/DDS Architecture

## How It Works

Agent ROS Bridge connects to ROS2 via `rclpy`, which internally uses **DDS** (Data Distribution Service) as its middleware:

```
Your Agent          ROS Bridge          ROS2 Stack           Network
    │                  │                    │                  │
    │  WebSocket       │      rclpy         │     DDS          │
    │══════════════════▶════════════════════▶══════════════════▶
    │  (JSON/Protobuf) │   (ROS2 API)       │  (FastDDS/etc)   │
```

## What is DDS?

DDS is a publish-subscribe messaging standard used by ROS2:
- **FastDDS** — Default in Humble/Jazzy (eProsima)
- **CycloneDDS** — Eclipse foundation, popular alternative
- **RTI Connext** — Commercial, used in some industrial settings
- **GurumDDS** — Korean alternative

## Why We Don't Support Native DDS Directly

| Approach | Pros | Cons |
|----------|------|------|
| **ROS2 (current)** | Mature API, tools, debugging | Slight overhead |
| **Native DDS** | Lower latency, direct control | Complex API, vendor fragmentation |

**Decision:** Use ROS2's DDS abstraction. It provides:
- Topic discovery and introspection (`ros2 topic list`)
- Message serialization (automatic)
- QoS policies (reliable/best-effort)
- Tooling (rviz, rqt, ros2 cli)

## DDS Compatibility

You still get DDS benefits:
- **Real-time capable** — DDS supports real-time constraints
- **Distributed** — Nodes on different hosts communicate seamlessly
- **QoS** — Configure reliability, durability, deadlines

## Swapping DDS Implementations

If you need a different DDS vendor:

```bash
# Install alternative
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Switch to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python run_bridge.py

# Verify
ros2 doctor | grep middleware
```

## For Non-ROS DDS Systems

If you need to connect to a pure DDS system (no ROS):

1. **Use ros1_bridge pattern** — Create a DDS-ROS2 bridge node
2. **Use CycloneDDS C API** — Direct integration (advanced)
3. **Use OpenDDS/RTI** — Vendor-specific SDKs

## Summary

- ✅ ROS2 **is** DDS under the hood
- ✅ Your bridge already uses DDS via rclpy
- ✅ Swap DDS vendors via `RMW_IMPLEMENTATION`
- ❌ No need for native DDS support in the bridge
- 📝 Document this for users who ask about DDS
