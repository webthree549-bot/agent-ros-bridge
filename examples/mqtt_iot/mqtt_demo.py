#!/usr/bin/env python3
"""MQTT Transport Demo

Demonstrates MQTT transport for IoT sensor integration.
Requires an MQTT broker (e.g., Mosquitto).

Setup:
    # Install Mosquitto (macOS)
    brew install mosquitto
    brew services start mosquitto

    # Or use Docker
    docker run -it -p 1883:1883 eclipse-mosquitto

Usage:
    python demo/mqtt_demo.py
"""

import asyncio
import json
import logging
import time

from agent_ros_bridge import Bridge, Header, Message, Telemetry
from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mqtt_demo")


class SensorSimulator:
    """Simulates IoT sensors publishing to MQTT"""

    def __init__(self, broker="localhost", port=1883):
        self.broker = broker
        self.port = port
        self.running = False

    async def start(self):
        """Start publishing sensor data"""
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            logger.error("paho-mqtt not installed. Run: pip install paho-mqtt")
            return

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="sensor_simulator")
        client.connect(self.broker, self.port)
        client.loop_start()

        self.running = True
        logger.info(f"Sensor simulator connected to {self.broker}:{self.port}")

        try:
            while self.running:
                # Simulate greenhouse sensors
                temp = 22.0 + (time.time() % 10)  # 22-32°C
                humidity = 50.0 + (time.time() % 20)  # 50-70%

                telemetry = {
                    "id": f"sensor_{int(time.time())}",
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
                    "telemetry": {
                        "topic": "greenhouse/sensors",
                        "data": {
                            "temperature": round(temp, 2),
                            "humidity": round(humidity, 2),
                            "sensor_id": "gh_01",
                        },
                        "quality": 0.95,
                    },
                }

                client.publish("robots/telemetry/gh_01", json.dumps(telemetry))
                logger.info(f"Published: temp={temp:.1f}°C, humidity={humidity:.1f}%")

                await asyncio.sleep(5)  # Publish every 5 seconds

        finally:
            client.loop_stop()
            client.disconnect()

    def stop(self):
        self.running = False


async def handle_message(message, identity):
    """Handle incoming MQTT messages"""
    logger.info(f"📩 From {identity.name}: {message}")

    if message.command:
        # Execute command and return response
        cmd = message.command.action
        params = message.command.parameters

        if cmd == "get_sensor_data":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="sensor_response",
                    data={"status": "ok", "last_reading": {"temp": 25.5, "humidity": 60}},
                ),
            )

    if message.telemetry:
        # Process telemetry
        data = message.telemetry.data
        logger.info(f"📊 Telemetry: {data}")

    return None


async def main():
    bridge = Bridge()

    # WebSocket for web dashboard
    ws_transport = WebSocketTransport({"port": 8770})
    ws_transport.message_handler = handle_message
    bridge.transport_manager.register(ws_transport)

    # MQTT for IoT sensors
    mqtt_transport = MQTTTransport(
        {
            "host": "localhost",
            "port": 1883,
            "subscriptions": ["robots/telemetry/#", "robots/commands/#"],
            "client_id": "agent_ros_bridge_demo",
        }
    )
    mqtt_transport.message_handler = handle_message
    bridge.transport_manager.register(mqtt_transport)

    # Start bridge
    await bridge.start()

    # Start sensor simulator
    simulator = SensorSimulator()
    sim_task = asyncio.create_task(simulator.start())

    print("=" * 60)
    print("📡 MQTT TRANSPORT DEMO")
    print("=" * 60)
    print("WebSocket: ws://localhost:8770")
    print("MQTT: localhost:1883")
    print("")
    print("This demo shows:")
    print("  • MQTT broker connection")
    print("  • IoT sensor publishing telemetry")
    print("  • WebSocket for web dashboard")
    print("  • Bidirectional MQTT communication")
    print("")
    print("Test commands:")
    print(
        '  mosquitto_pub -t "robots/commands/test" -m \'{"command":{"action":"get_sensor_data"}}\''
    )
    print("  wscat -c ws://localhost:8770")
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️  Stopping...")
        simulator.stop()
        sim_task.cancel()
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
