# Protocol Examples

Examples for multi-protocol robot communication.

## Examples

### multiprotocol_iot_fleet.py
Coordinating robots across 4 protocols.

**Protocols:**
- WebSocket (port 8765) - Real-time video/control
- gRPC (port 50051) - High-performance RPC
- MQTT (port 1883) - IoT messaging
- TCP (port 9999) - Raw data streaming

**Features:**
- Protocol-agnostic commands
- Heterogeneous robot support
- Real-time performance optimization
- Mixed fleet coordination

**Run:**
```bash
python examples/protocols/multiprotocol_iot_fleet.py
pytest tests/examples/protocols/test_multiprotocol_iot_fleet.py -v
```

### grpc_example.py
High-performance RPC communication.

**Use Case:** Precise robot arm control

## Protocol Selection Guide

| Robot Type | Protocol | Latency | Use Case |
|------------|----------|---------|----------|
| Drone | WebSocket | <50ms | Video + control |
| AGV | MQTT | <100ms | IoT messaging |
| Robot Arm | gRPC | <10ms | Precise control |
| Sensors | TCP | <5ms | Data streaming |

---

*For heterogeneous robot fleets with different communication needs.*
