#!/usr/bin/env python3
"""MQTT Transport for OpenClaw Gateway.

Connects to MQTT brokers for IoT sensor integration and
low-bandwidth robot communication.
"""

import asyncio
import json
import logging
from datetime import datetime
from typing import Any, Dict, Optional

try:
    import paho.mqtt.client as mqtt

    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

from agent_ros_bridge.gateway_v2.core import (
    Command,
    Event,
    Header,
    Identity,
    Message,
    Telemetry,
    Transport,
)

logger = logging.getLogger("transport.mqtt")


class MQTTTransport(Transport):
    """MQTT transport for IoT and sensor integration."""

    def __init__(self, config: Dict[str, Any]):
        super().__init__("mqtt", config)
        self.broker_host = config.get("host", "localhost")
        self.broker_port = config.get("port", 1883)
        self.username = config.get("username")
        self.password = config.get("password")
        self.client_id = config.get("client_id", f"agent_ros_bridge_{id(self)}")
        self.tls_enabled = config.get("tls", False)
        self.tls_ca = config.get("tls_ca")
        self.tls_cert = config.get("tls_cert")
        self.tls_key = config.get("tls_key")

        # Topic mappings
        self.command_topic = config.get("command_topic", "robots/commands")
        self.telemetry_topic = config.get("telemetry_topic", "robots/telemetry")
        self.event_topic = config.get("event_topic", "robots/events")

        # Subscriptions
        self.subscribed_topics = config.get("subscriptions", ["#"])

        self.client = None
        self._connected = False
        self._message_queue: asyncio.Queue = asyncio.Queue()
        self._identities: Dict[str, Identity] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    async def start(self) -> bool:
        """Start MQTT client and connect to broker."""
        if not MQTT_AVAILABLE:
            logger.error("paho-mqtt not installed. Run: pip install paho-mqtt>=1.6.0")
            return False

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id)

        # Set callbacks
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        # Authentication
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)

        # TLS
        if self.tls_enabled:
            if self.tls_ca:
                self.client.tls_set(
                    ca_certs=self.tls_ca, certfile=self.tls_cert, keyfile=self.tls_key
                )
            else:
                self.client.tls_set()

        try:
            # Capture the running event loop so paho callbacks can safely enqueue
            self._loop = asyncio.get_event_loop()

            # Connect in background thread
            self.client.connect(self.broker_host, self.broker_port)
            self.client.loop_start()

            # Wait for connection
            for _ in range(50):  # 5 seconds timeout
                if self._connected:
                    break
                await asyncio.sleep(0.1)

            if not self._connected:
                logger.error(
                    f"Failed to connect to MQTT broker at {self.broker_host}:{self.broker_port}"
                )
                return False

            self.running = True
            logger.info(f"MQTT transport connected to {self.broker_host}:{self.broker_port}")

            # Subscribe to topics
            for topic in self.subscribed_topics:
                self.client.subscribe(topic)
                logger.info(f"Subscribed to MQTT topic: {topic}")

            # Start message processing loop
            asyncio.create_task(self._process_messages())

            return True

        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
            return False

    async def stop(self) -> None:
        """Stop MQTT client."""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        self.running = False
        self._connected = False
        logger.info("MQTT transport stopped")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connect callback (API v2 compatible)."""
        if rc == 0:
            self._connected = True
            logger.info("Connected to MQTT broker")
        else:
            logger.error(f"MQTT connection failed with code: {rc}")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnect callback (API v2 compatible)."""
        self._connected = False
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnection (rc={rc})")

    def _on_message(self, client, userdata, msg):
        """MQTT message received callback (runs in background thread)."""
        try:
            # Parse message
            payload = json.loads(msg.payload.decode("utf-8"))

            # Create identity for this MQTT client
            client_id = f"mqtt_{msg.topic.replace('/', '_')}"
            if client_id not in self._identities:
                self._identities[client_id] = Identity(
                    id=client_id,
                    name=f"mqtt_client_{client_id[:20]}",
                    roles=["mqtt"],
                    metadata={
                        "transport": "mqtt",
                        "topic": msg.topic,
                        "broker": f"{self.broker_host}:{self.broker_port}",
                    },
                )

            identity = self._identities[client_id]

            # Convert to Message
            message = self._mqtt_to_message(payload, msg.topic)

            # put_nowait is not thread-safe; use call_soon_threadsafe from paho's thread
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(self._message_queue.put_nowait, (message, identity))

        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON in MQTT message on topic {msg.topic}")
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")

    async def _process_messages(self):
        """Process incoming MQTT messages."""
        while self.running:
            try:
                message, identity = await asyncio.wait_for(self._message_queue.get(), timeout=1.0)

                if self.message_handler:
                    response = await self.message_handler(message, identity)
                    if response:
                        await self.send(response, identity.id)

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Error in MQTT message processing: {e}")

    async def send(self, message: Message, recipient: str) -> bool:
        """Publish message to MQTT topic."""
        if not self._connected or not self.client:
            logger.warning("MQTT not connected, cannot send")
            return False

        try:
            # Determine topic based on message type
            if message.command:
                topic = f"{self.command_topic}/{recipient}"
            elif message.telemetry:
                topic = f"{self.telemetry_topic}/{recipient}"
            elif message.event:
                topic = f"{self.event_topic}/{recipient}"
            else:
                topic = f"{self.telemetry_topic}/{recipient}"

            payload = self._message_to_mqtt(message)

            # Publish with QoS 1 (at least once)
            result = self.client.publish(topic, json.dumps(payload), qos=1)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                return True
            else:
                logger.warning(f"MQTT publish failed: {result.rc}")
                return False

        except Exception as e:
            logger.error(f"Error sending MQTT message: {e}")
            return False

    async def broadcast(self, message: Message) -> int:
        """Broadcast to all MQTT subscribers."""
        success = 0
        for client_id in self._identities:
            if await self.send(message, client_id):
                success += 1
        return success

    def _mqtt_to_message(self, payload: Dict[str, Any], topic: str) -> Message:
        """Convert MQTT payload to Message."""
        header = Header(
            message_id=payload.get("id", ""),
            timestamp=datetime.fromisoformat(payload.get("timestamp"))
            if "timestamp" in payload
            else datetime.utcnow(),
            source=topic,
            target="bridge",
        )

        command = None
        telemetry = None
        event = None

        if "command" in payload:
            cmd = payload["command"]
            command = Command(
                action=cmd.get("action", ""),
                parameters=cmd.get("parameters", {}),
                timeout_ms=cmd.get("timeout_ms", 5000),
                priority=cmd.get("priority", 5),
            )

        if "telemetry" in payload:
            tel = payload["telemetry"]
            telemetry = Telemetry(
                topic=tel.get("topic", topic), data=tel.get("data"), quality=tel.get("quality", 1.0)
            )

        if "event" in payload:
            evt = payload["event"]
            event = Event(
                event_type=evt.get("type", ""),
                severity=evt.get("severity", "info"),
                data=evt.get("data", {}),
            )

        return Message(
            header=header,
            command=command,
            telemetry=telemetry,
            event=event,
            metadata=payload.get("metadata", {}),
        )

    def _message_to_mqtt(self, message: Message) -> Dict[str, Any]:
        """Convert Message to MQTT payload."""
        payload = {
            "id": message.header.message_id,
            "timestamp": message.header.timestamp.isoformat(),
            "metadata": message.metadata,
        }

        if message.command:
            payload["command"] = {
                "action": message.command.action,
                "parameters": message.command.parameters,
                "timeout_ms": message.command.timeout_ms,
                "priority": message.command.priority,
            }

        if message.telemetry:
            payload["telemetry"] = {
                "topic": message.telemetry.topic,
                "data": message.telemetry.data,
                "quality": message.telemetry.quality,
            }

        if message.event:
            payload["event"] = {
                "type": message.event.event_type,
                "severity": message.event.severity,
                "data": message.event.data,
            }

        return payload
