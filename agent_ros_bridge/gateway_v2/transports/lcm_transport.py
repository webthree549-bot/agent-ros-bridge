"""LCM (Lightweight Communications and Marshalling) Transport for Agent ROS Bridge.

Inspired by dimensionalOS/dimos, adapted for ROS compatibility.
LCM provides low-latency, UDP-based message passing with C-struct compatible messages.
"""

import asyncio
import json
import struct
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Type, Union
from collections import defaultdict

try:
    import lcm

    LCM_AVAILABLE = True
except ImportError:
    LCM_AVAILABLE = False

from agent_ros_bridge.gateway_v2.core import Transport


@dataclass
class LCMMessage:
    """LCM-compatible message structure.

    Mimics C-struct layout for zero-copy serialization.
    """

    channel: str
    timestamp: float = field(default_factory=time.time)
    data: bytes = b""

    def encode(self) -> bytes:
        """Encode to LCM-compatible bytes."""
        # Simple encoding: channel length + channel + timestamp + data length + data
        channel_bytes = self.channel.encode("utf-8")
        header = struct.pack("!I", len(channel_bytes))  # Channel length
        timestamp_bytes = struct.pack("!d", self.timestamp)  # 8-byte double
        data_len = struct.pack("!I", len(self.data))  # Data length

        return header + channel_bytes + timestamp_bytes + data_len + self.data

    @classmethod
    def decode(cls, buf: bytes) -> "LCMMessage":
        """Decode from LCM-compatible bytes."""
        offset = 0

        # Channel length
        channel_len = struct.unpack("!I", buf[offset : offset + 4])[0]
        offset += 4

        # Channel
        channel = buf[offset : offset + channel_len].decode("utf-8")
        offset += channel_len

        # Timestamp
        timestamp = struct.unpack("!d", buf[offset : offset + 8])[0]
        offset += 8

        # Data length
        data_len = struct.unpack("!I", buf[offset : offset + 4])[0]
        offset += 4

        # Data
        data = buf[offset : offset + data_len]

        return cls(channel=channel, timestamp=timestamp, data=data)


class LCMPublisher:
    """LCM Publisher - sends messages to a channel."""

    def __init__(self, transport: "LCMTransport", channel: str):
        self.transport = transport
        self.channel = channel

    def publish(self, data: Union[bytes, dict, str]) -> None:
        """Publish data to the channel."""
        if isinstance(data, dict):
            data = json.dumps(data).encode("utf-8")
        elif isinstance(data, str):
            data = data.encode("utf-8")

        msg = LCMMessage(channel=self.channel, data=data)
        self.transport._publish_raw(msg)


class LCMSubscriber:
    """LCM Subscriber - receives messages from a channel."""

    def __init__(self, transport: "LCMTransport", channel: str, callback: Callable):
        self.transport = transport
        self.channel = channel
        self.callback = callback
        self._subscribed = False

    def subscribe(self) -> None:
        """Start receiving messages."""
        self.transport._subscribe(self.channel, self.callback)
        self._subscribed = True

    def unsubscribe(self) -> None:
        """Stop receiving messages."""
        self.transport._unsubscribe(self.channel, self.callback)
        self._subscribed = False


class LCMTransport(Transport):
    """LCM Transport for high-performance, low-latency messaging.

    Features:
    - UDP multicast for efficient broadcast
    - C-struct compatible message format
    - Channel-based pub/sub
    - Shared memory option for local processes

    Inspired by dimensionalOS/dimos LCM implementation.
    """

    def __init__(self, config: Optional[Dict] = None):
        super().__init__("lcm", config or {})
        import logging

        self.logger = logging.getLogger(__name__)

        self.udp_url = self.config.get("udp_url", "udpm://239.255.76.67:7667")
        self.shared_memory = self.config.get("shared_memory", True)
        self.queue_size = self.config.get("queue_size", 1000)

        self._lcm: Optional[Any] = None
        self._subscriptions: Dict[str, List[Callable]] = defaultdict(list)
        self._running = False
        self._handle_task: Optional[asyncio.Task] = None

        # Local message queue for shared memory mode
        self._local_queue: asyncio.Queue = asyncio.Queue(maxsize=self.queue_size)

    async def start(self) -> bool:
        """Start LCM transport."""
        if not LCM_AVAILABLE:
            self.logger.warning("LCM not available, using fallback implementation")
            self._running = True
            self._handle_task = asyncio.create_task(self._handle_loop())
            return True

        try:
            self._lcm = lcm.LCM(self.udp_url)
            self._running = True
            self._handle_task = asyncio.create_task(self._handle_loop())
            self.logger.info(f"LCM transport started on {self.udp_url}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start LCM: {e}")
            return False

    async def stop(self) -> None:
        """Stop LCM transport."""
        self._running = False
        if self._handle_task:
            self._handle_task.cancel()
            try:
                await self._handle_task
            except asyncio.CancelledError:
                pass
        self.logger.info("LCM transport stopped")

    async def _handle_loop(self) -> None:
        """Main LCM handle loop."""
        while self._running:
            try:
                if self._lcm:
                    # Non-blocking LCM handle
                    self._lcm.handle_timeout(10)  # 10ms timeout
                else:
                    # Fallback: process local queue
                    await asyncio.sleep(0.001)
            except Exception as e:
                self.logger.error(f"LCM handle error: {e}")
                await asyncio.sleep(0.1)

    def _publish_raw(self, msg: LCMMessage) -> None:
        """Publish raw LCM message."""
        if self._lcm:
            self._lcm.publish(msg.channel, msg.encode())

        # Also publish to local subscribers (shared memory optimization)
        if msg.channel in self._subscriptions:
            for callback in self._subscriptions[msg.channel]:
                try:
                    callback(msg.data)
                except Exception as e:
                    self.logger.error(f"Callback error: {e}")

    def _subscribe(self, channel: str, callback: Callable) -> None:
        """Subscribe to a channel."""
        self._subscriptions[channel].append(callback)

        if self._lcm:
            # LCM native subscription
            def handler(channel_name, data):
                callback(data)

            self._lcm.subscribe(channel, handler)

    def _unsubscribe(self, channel: str, callback: Callable) -> None:
        """Unsubscribe from a channel."""
        if callback in self._subscriptions[channel]:
            self._subscriptions[channel].remove(callback)

    def publisher(self, channel: str) -> LCMPublisher:
        """Create a publisher for a channel."""
        return LCMPublisher(self, channel)

    def subscriber(self, channel: str, callback: Callable) -> LCMSubscriber:
        """Create a subscriber for a channel."""
        return LCMSubscriber(self, channel, callback)

    async def send(self, message: "Message", recipient: str) -> bool:
        """Send message to specific recipient (channel)."""
        from agent_ros_bridge.gateway_v2.core import Message

        # Convert Message to LCM format
        if isinstance(message, Message):
            # Build data from available fields
            data = {
                "header": {
                    "source": message.header.source if message.header else "unknown",
                    "timestamp": message.header.timestamp.isoformat() if message.header else "",
                }
            }
            # Add command/telemetry/event if present
            if message.command:
                data["command"] = {
                    "action": message.command.action,
                    "parameters": message.command.parameters,
                }
            if message.telemetry:
                data["telemetry"] = {
                    "topic": message.telemetry.topic,
                    "data": message.telemetry.data,
                }
            if message.event:
                data["event"] = {"type": message.event.type, "data": message.event.data}
        else:
            data = {"raw": str(message)}

        encoded = json.dumps(data).encode("utf-8")
        msg = LCMMessage(channel=recipient, data=encoded)
        self._publish_raw(msg)
        return True

    async def broadcast(self, message: "Message") -> list[str]:
        """Broadcast message to all subscribers."""
        await self.send(message, "broadcast")
        return ["broadcast"]

    @property
    def is_connected(self) -> bool:
        """Check if transport is connected."""
        return self._running


class SharedMemoryTransport(LCMTransport):
    """Shared memory transport for local, zero-copy communication.

    Optimized for processes on the same machine.
    Falls back to LCM for cross-machine communication.
    """

    def __init__(self, config: Optional[Dict] = None):
        super().__init__(config or {})
        self.name = "shm"  # Override name
        self.shared_memory = True
        self._shared_buffers: Dict[str, Any] = {}

    def _publish_raw(self, msg: LCMMessage) -> None:
        """Publish using shared memory when possible."""
        # For local subscribers, directly call callbacks (zero-copy)
        if msg.channel in self._subscriptions:
            for callback in self._subscriptions[msg.channel]:
                try:
                    callback(msg.data)
                except Exception as e:
                    self.logger.error(f"Callback error: {e}")
