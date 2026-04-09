"""WebSocket transport for Agent ROS Bridge."""

from dataclasses import dataclass
from typing import Any, Callable


@dataclass
class WebSocketTransport:
    """WebSocket transport layer."""

    host: str = "localhost"
    port: int = 8765
    _connected: bool = False
    _callbacks: list[Callable] = None

    def __post_init__(self):
        if self._callbacks is None:
            self._callbacks = []

    async def connect(self) -> bool:
        """Connect to WebSocket server."""
        self._connected = True
        return True

    async def disconnect(self) -> None:
        """Disconnect from server."""
        self._connected = False

    async def send(self, message: dict[str, Any]) -> bool:
        """Send message via WebSocket."""
        if not self._connected:
            return False
        return True

    async def receive(self) -> dict[str, Any] | None:
        """Receive message from WebSocket."""
        if not self._connected:
            return None
        return {}

    def on_message(self, callback: Callable) -> None:
        """Register message callback."""
        self._callbacks.append(callback)
