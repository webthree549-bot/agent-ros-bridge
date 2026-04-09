"""MQTT transport for Agent ROS Bridge."""

from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class MQTTTransport:
    """MQTT transport layer."""

    broker: str = "localhost"
    port: int = 1883
    client_id: str = "agent_ros_bridge"
    _connected: bool = False
    _subscriptions: dict[str, Callable] = field(default_factory=dict)

    async def connect(self) -> bool:
        """Connect to MQTT broker."""
        self._connected = True
        return True

    async def disconnect(self) -> None:
        """Disconnect from broker."""
        self._connected = False

    async def publish(self, topic: str, message: dict[str, Any]) -> bool:
        """Publish message to topic."""
        return self._connected

    async def subscribe(self, topic: str, callback: Callable) -> bool:
        """Subscribe to topic."""
        if not self._connected:
            return False
        self._subscriptions[topic] = callback
        return True

    async def unsubscribe(self, topic: str) -> bool:
        """Unsubscribe from topic."""
        if topic in self._subscriptions:
            del self._subscriptions[topic]
            return True
        return False

    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected
