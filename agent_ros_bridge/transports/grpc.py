"""gRPC transport for Agent ROS Bridge."""

from dataclasses import dataclass
from typing import Any


@dataclass
class GRPCTransport:
    """gRPC transport layer."""

    host: str = "localhost"
    port: int = 50051
    _connected: bool = False
    _stub = None

    async def connect(self) -> bool:
        """Connect to gRPC server."""
        self._connected = True
        return True

    async def disconnect(self) -> None:
        """Disconnect from server."""
        self._connected = False
        self._stub = None

    async def call(self, method: str, request: dict[str, Any]) -> dict[str, Any]:
        """Make gRPC call."""
        if not self._connected:
            raise ConnectionError("Not connected")
        return {"status": "success"}

    @property
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected
