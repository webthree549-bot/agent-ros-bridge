#!/usr/bin/env python3
"""Integration tests for Agent ROS Bridge

Tests the bridge against a real ROS2 environment running in Docker.

Usage:
    # Start ROS2 container first
    docker-compose --profile ros2 up ros2-bridge

    # Run tests
    pytest tests/integration/ -v

Or with mock bridge (no Docker required):
    pytest tests/integration/ -v --mock
"""

import asyncio
import json
import os
import sys

import pytest

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


class TestWebSocketIntegration:
    """Integration tests for WebSocket transport"""

    @pytest.fixture
    async def ws_client(self):
        """Create WebSocket client connection"""
        import os as os_module

        websockets = None  # Initialize to avoid UnboundLocalError
        try:
            import websockets
        except ImportError:
            pytest.skip("websockets not installed")

        # Determine bridge URL
        bridge_url = os_module.environ.get("BRIDGE_URL", "ws://localhost:8766")

        # Disable SOCKS proxy if set
        orig_proxy = os_module.environ.pop("ALL_PROXY", None)
        orig_http_proxy = os_module.environ.pop("HTTP_PROXY", None)
        orig_https_proxy = os_module.environ.pop("HTTPS_PROXY", None)

        try:
            async with websockets.connect(bridge_url) as ws:
                yield ws
        except Exception as e:
            pytest.skip(f"Cannot connect to bridge at {bridge_url}: {e}")
        finally:
            # Restore proxy settings
            if orig_proxy:
                os_module.environ["ALL_PROXY"] = orig_proxy
            if orig_http_proxy:
                os_module.environ["HTTP_PROXY"] = orig_http_proxy
            if orig_https_proxy:
                os_module.environ["HTTPS_PROXY"] = orig_https_proxy

    @pytest.mark.asyncio
    async def test_connection_established(self, ws_client):
        """Test that WebSocket connection is established"""
        # Connection should be open
        assert ws_client.open

    @pytest.mark.asyncio
    async def test_list_robots_command(self, ws_client):
        """Test list_robots command returns robot list"""
        # Send command
        await ws_client.send(json.dumps({"command": {"action": "list_robots"}}))

        # Wait for response
        response = json.loads(await ws_client.recv())

        # Verify response structure
        assert "telemetry" in response or "event" in response

        if "telemetry" in response:
            assert "topic" in response["telemetry"]
            assert response["telemetry"]["topic"] == "robots"

    @pytest.mark.asyncio
    async def test_get_topics_command(self, ws_client):
        """Test get_topics command returns topic list"""
        await ws_client.send(json.dumps({"command": {"action": "get_topics"}}))

        response = json.loads(await ws_client.recv())

        assert "telemetry" in response
        assert response["telemetry"]["topic"] == "topics"
        assert "topics" in response["telemetry"]["data"]

    @pytest.mark.asyncio
    async def test_invalid_json_rejected(self, ws_client):
        """Test that invalid JSON is rejected gracefully"""
        await ws_client.send("not valid json")

        response = json.loads(await ws_client.recv())

        assert "error" in response or "event" in response


class TestMQTTIntegration:
    """Integration tests for MQTT transport"""

    @pytest.fixture
    async def mqtt_client(self):
        """Create MQTT client connection"""
        mqtt = None  # Initialize to avoid UnboundLocalError
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            pytest.skip("paho-mqtt not installed")

        broker_host = os.environ.get("MQTT_BROKER", "localhost")
        broker_port = int(os.environ.get("MQTT_PORT", "1883"))

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

        # Track received messages
        messages = []

        def on_message(client, userdata, msg):
            messages.append(json.loads(msg.payload.decode()))

        client.on_message = on_message

        try:
            client.connect(broker_host, broker_port)
            client.loop_start()

            yield client, messages

            client.loop_stop()
            client.disconnect()
        except Exception as e:
            pytest.skip(f"Cannot connect to MQTT broker at {broker_host}:{broker_port}: {e}")

    @pytest.mark.asyncio
    async def test_mqtt_publish(self, mqtt_client):
        """Test publishing to MQTT topic"""
        client, messages = mqtt_client

        # Subscribe to test topic
        client.subscribe("test/integration")

        # Publish message
        test_msg = {"command": {"action": "test"}}
        result = client.publish("test/integration", json.dumps(test_msg))

        # Wait for message to be received
        await asyncio.sleep(0.5)

        # Verify message was sent
        assert result.rc == 0  # MQTT_ERR_SUCCESS


class TestBridgeLifecycle:
    """Integration tests for bridge lifecycle"""

    @pytest.fixture
    async def bridge(self):
        """Create and start bridge instance"""
        bridge = Bridge()

        # Add WebSocket transport
        ws_transport = WebSocketTransport(
            {
                "port": 8769,  # Different port to avoid conflicts
                "host": "localhost",
            }
        )
        bridge.transport_manager.register(ws_transport)

        # Start bridge
        await bridge.start()

        yield bridge

        # Cleanup
        await bridge.stop()

    @pytest.mark.asyncio
    async def test_bridge_starts_successfully(self, bridge):
        """Test that bridge starts without errors"""
        assert bridge.running is True

    @pytest.mark.asyncio
    async def test_bridge_stops_gracefully(self, bridge):
        """Test that bridge stops without errors"""
        await bridge.stop()
        assert bridge.running is False


class TestAuthenticationIntegration:
    """Integration tests for authentication"""

    @pytest.mark.asyncio
    async def test_auth_required_without_token(self):
        """Test that auth-enabled bridge rejects connections without token"""
        websockets = None  # Initialize to avoid UnboundLocalError
        try:
            import websockets
        except ImportError:
            pytest.skip("websockets not installed")

        # This test requires auth-enabled bridge
        auth_bridge_url = os.environ.get("AUTH_BRIDGE_URL", "ws://localhost:8768")

        try:
            async with websockets.connect(auth_bridge_url) as ws:
                # Should be rejected immediately
                await asyncio.sleep(0.5)
                assert not ws.open, "Connection should be closed without auth"
        except websockets.exceptions.InvalidStatusCode as e:
            # Expected: 4001 (auth required)
            assert e.status_code == 4001

    @pytest.mark.asyncio
    async def test_auth_accepted_with_valid_token(self):
        """Test that valid JWT token is accepted"""
        websockets = None  # Initialize to avoid UnboundLocalError
        try:
            import websockets
        except ImportError:
            pytest.skip("websockets not installed")

        auth_bridge_url = os.environ.get("AUTH_BRIDGE_URL", "ws://localhost:8768")
        jwt_token = os.environ.get("TEST_JWT_TOKEN")

        if not jwt_token:
            pytest.skip("TEST_JWT_TOKEN not set")

        try:
            async with websockets.connect(f"{auth_bridge_url}?token={jwt_token}") as ws:
                # Should stay connected
                await asyncio.sleep(0.5)
                assert ws.open, "Connection should remain open with valid token"

                # Test sending a command
                await ws.send(json.dumps({"command": {"action": "list_robots"}}))

                response = json.loads(await ws.recv())
                assert "telemetry" in response or "event" in response

        except Exception as e:
            pytest.fail(f"Connection with valid token failed: {e}")


# Skip all integration tests if running in CI without Docker
@pytest.fixture(autouse=True)
def skip_if_no_docker():
    """Skip integration tests if Docker is not available"""
    if os.environ.get("SKIP_INTEGRATION_TESTS"):
        pytest.skip("Integration tests disabled (SKIP_INTEGRATION_TESTS set)")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
