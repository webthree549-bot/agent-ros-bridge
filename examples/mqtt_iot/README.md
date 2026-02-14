# MQTT IoT Example

**IoT sensor integration via MQTT.**

## What It Does

Demonstrates connecting IoT sensors to the bridge via MQTT protocol.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed
- MQTT broker (optional - uses mock if not available)

## Run

```bash
./run.sh
```

## Test

```bash
# If you have mosquitto
mosquitto_pub -t "robots/tb4_001/cmd" -m '{"action": "move", "parameters": {"direction": "forward"}}'

# Or use the bridge's MQTT transport
python mqtt_demo.py
```

## What's Happening

This demonstrates:
- **MQTT Transport**: Pub/sub messaging
- **IoT Integration**: Sensor data ingestion
- **Protocol Bridge**: MQTT to WebSocket gateway

## Next Steps

- Set up [Mosquitto](https://mosquitto.org/) broker
- Integrate with industrial sensors
- Read [User Manual - MQTT](../../docs/USER_MANUAL.md#mqtt-transport)
