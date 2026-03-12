"""Tests for LCM Transport - TDD approach.

Following TDD principles:
1. Write tests first (this file)
2. Implement minimal code to pass
3. Refactor
"""

import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
import struct

from agent_ros_bridge.gateway_v2.transports.lcm_transport import (
    LCMMessage,
    LCMPublisher,
    LCMSubscriber,
    LCMTransport,
    SharedMemoryTransport,
)


class TestLCMMessage:
    """Test LCMMessage encoding/decoding."""

    def test_message_creation(self):
        """Can create LCM message."""
        msg = LCMMessage(channel="test_channel", data=b"test_data")

        assert msg.channel == "test_channel"
        assert msg.data == b"test_data"
        assert msg.timestamp > 0

    def test_message_encode_decode(self):
        """Can encode and decode message."""
        original = LCMMessage(channel="test", data=b"hello")

        encoded = original.encode()
        decoded = LCMMessage.decode(encoded)

        assert decoded.channel == original.channel
        assert decoded.data == original.data
        assert abs(decoded.timestamp - original.timestamp) < 0.001

    def test_message_encode_structure(self):
        """Encoded message has correct structure."""
        msg = LCMMessage(channel="test", data=b"data")
        encoded = msg.encode()

        # Should have: channel_len (4) + channel + timestamp (8) + data_len (4) + data
        channel_len = struct.unpack("!I", encoded[0:4])[0]
        assert channel_len == 4  # "test"

        channel = encoded[4:8].decode("utf-8")
        assert channel == "test"

        timestamp = struct.unpack("!d", encoded[8:16])[0]
        assert timestamp > 0

        data_len = struct.unpack("!I", encoded[16:20])[0]
        assert data_len == 4  # "data"

        data = encoded[20:24]
        assert data == b"data"


class TestLCMPublisher:
    """Test LCM Publisher."""

    def test_publisher_creation(self):
        """Can create publisher."""
        transport = Mock()
        pub = LCMPublisher(transport, "test_channel")

        assert pub.transport == transport
        assert pub.channel == "test_channel"

    def test_publish_bytes(self):
        """Can publish bytes."""
        transport = Mock()
        pub = LCMPublisher(transport, "test_channel")

        pub.publish(b"test_data")

        transport._publish_raw.assert_called_once()
        call_args = transport._publish_raw.call_args[0][0]
        assert call_args.channel == "test_channel"
        assert call_args.data == b"test_data"

    def test_publish_string(self):
        """Can publish string."""
        transport = Mock()
        pub = LCMPublisher(transport, "test_channel")

        pub.publish("test_string")

        call_args = transport._publish_raw.call_args[0][0]
        assert call_args.data == b"test_string"

    def test_publish_dict(self):
        """Can publish dict (JSON encoded)."""
        transport = Mock()
        pub = LCMPublisher(transport, "test_channel")

        pub.publish({"key": "value"})

        call_args = transport._publish_raw.call_args[0][0]
        import json

        assert json.loads(call_args.data) == {"key": "value"}


class TestLCMSubscriber:
    """Test LCM Subscriber."""

    def test_subscriber_creation(self):
        """Can create subscriber."""
        transport = Mock()
        callback = Mock()
        sub = LCMSubscriber(transport, "test_channel", callback)

        assert sub.transport == transport
        assert sub.channel == "test_channel"
        assert sub.callback == callback
        assert not sub._subscribed

    def test_subscribe(self):
        """Can subscribe to channel."""
        transport = Mock()
        callback = Mock()
        sub = LCMSubscriber(transport, "test_channel", callback)

        sub.subscribe()

        transport._subscribe.assert_called_once_with("test_channel", callback)
        assert sub._subscribed

    def test_unsubscribe(self):
        """Can unsubscribe from channel."""
        transport = Mock()
        callback = Mock()
        sub = LCMSubscriber(transport, "test_channel", callback)
        sub.subscribe()

        sub.unsubscribe()

        transport._unsubscribe.assert_called_once_with("test_channel", callback)
        assert not sub._subscribed


class TestLCMTransportInit:
    """Test LCM Transport initialization."""

    def test_default_config(self):
        """Transport initializes with defaults."""
        transport = LCMTransport({})

        assert transport.udp_url == "udpm://239.255.76.67:7667"
        assert transport.shared_memory is True
        assert transport.queue_size == 1000
        assert not transport._running

    def test_custom_config(self):
        """Transport initializes with custom config."""
        transport = LCMTransport(
            {"udp_url": "udpm://239.255.0.1:8000", "shared_memory": False, "queue_size": 500}
        )

        assert transport.udp_url == "udpm://239.255.0.1:8000"
        assert transport.shared_memory is False
        assert transport.queue_size == 500


class TestLCMTransportLifecycle:
    """Test LCM Transport lifecycle."""

    @pytest.mark.asyncio
    async def test_start_stop(self):
        """Can start and stop transport."""
        transport = LCMTransport({})

        result = await transport.start()
        assert result is True
        assert transport._running

        await transport.stop()
        assert not transport._running

    @pytest.mark.asyncio
    async def test_double_start(self):
        """Double start is safe."""
        transport = LCMTransport({})

        await transport.start()
        result = await transport.start()

        assert result is True
        await transport.stop()


class TestLCMTransportPubSub:
    """Test LCM Transport pub/sub functionality."""

    def test_subscribe_adds_callback(self):
        """Subscribe adds callback to registry."""
        transport = LCMTransport({})
        callback = Mock()

        transport._subscribe("test_channel", callback)

        assert callback in transport._subscriptions["test_channel"]

    def test_unsubscribe_removes_callback(self):
        """Unsubscribe removes callback from registry."""
        transport = LCMTransport({})
        callback = Mock()
        transport._subscribe("test_channel", callback)

        transport._unsubscribe("test_channel", callback)

        assert callback not in transport._subscriptions["test_channel"]

    def test_publish_raw_notifies_subscribers(self):
        """Publish notifies local subscribers."""
        transport = LCMTransport({})
        callback = Mock()
        transport._subscribe("test_channel", callback)

        msg = LCMMessage(channel="test_channel", data=b"test_data")
        transport._publish_raw(msg)

        callback.assert_called_once_with(b"test_data")

    def test_publish_raw_no_subscribers(self):
        """Publish with no subscribers doesn't fail."""
        transport = LCMTransport({})

        msg = LCMMessage(channel="empty_channel", data=b"data")
        transport._publish_raw(msg)  # Should not raise


class TestLCMTransportPublisherSubscriber:
    """Test publisher/subscriber factory methods."""

    def test_publisher_factory(self):
        """Can create publisher via factory."""
        transport = LCMTransport({})
        pub = transport.publisher("test_channel")

        assert isinstance(pub, LCMPublisher)
        assert pub.channel == "test_channel"

    def test_subscriber_factory(self):
        """Can create subscriber via factory."""
        transport = LCMTransport({})
        callback = Mock()
        sub = transport.subscriber("test_channel", callback)

        assert isinstance(sub, LCMSubscriber)
        assert sub.channel == "test_channel"


class TestLCMTransportSend:
    """Test LCM Transport send methods."""

    @pytest.mark.asyncio
    async def test_send_dict(self):
        """Can send dict message."""
        from agent_ros_bridge.gateway_v2.core import Message, Header, Command

        transport = LCMTransport({})
        await transport.start()

        # Create a proper Message object
        msg = Message(
            header=Header(source="test"),
            command=Command(action="test", parameters={"key": "value"}),
        )

        result = await transport.send(msg, "test_channel")
        assert result is True

        await transport.stop()

    @pytest.mark.asyncio
    async def test_broadcast(self):
        """Can broadcast message."""
        from agent_ros_bridge.gateway_v2.core import Message, Header, Command

        transport = LCMTransport({})
        await transport.start()

        msg = Message(
            header=Header(source="test"),
            command=Command(action="test", parameters={"type": "test"}),
        )

        recipients = await transport.broadcast(msg)
        assert "broadcast" in recipients

        await transport.stop()


class TestLCMTransportProperties:
    """Test LCM Transport properties."""

    def test_name_property(self):
        """Transport name is 'lcm'."""
        transport = LCMTransport({})
        assert transport.name == "lcm"

    def test_is_connected_when_running(self):
        """is_connected True when running."""
        transport = LCMTransport({})
        transport._running = True

        assert transport.is_connected

    def test_is_connected_when_stopped(self):
        """is_connected False when stopped."""
        transport = LCMTransport({})
        transport._running = False

        assert not transport.is_connected


class TestSharedMemoryTransport:
    """Test Shared Memory Transport."""

    def test_shm_creation(self):
        """Can create shared memory transport."""
        transport = SharedMemoryTransport({})

        assert transport.name == "shm"
        assert transport.shared_memory is True

    def test_shm_publish_to_local_subscribers(self):
        """SHM transport directly calls local subscribers."""
        transport = SharedMemoryTransport({})
        callback = Mock()
        transport._subscribe("test", callback)

        msg = LCMMessage(channel="test", data=b"data")
        transport._publish_raw(msg)

        callback.assert_called_once_with(b"data")
