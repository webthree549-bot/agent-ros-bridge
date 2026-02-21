# MQTT IoT Demo

Demonstrates MQTT transport for IoT sensor integration with Agent ROS Bridge.

## Running

```bash
export JWT_SECRET=$(openssl rand -base64 32)
docker-compose up
```

## Services

- MQTT Broker: localhost:1883
- WebSocket: localhost:8770

## Testing

Publish sensor data:
```bash
mosquitto_pub -t "robots/telemetry/sensor_01" -m '{"temperature": 25.5}'
```

Subscribe to commands:
```bash
mosquitto_sub -t "robots/commands/#"
```

Connect via WebSocket:
```bash
wscat -c ws://localhost:8770
```
