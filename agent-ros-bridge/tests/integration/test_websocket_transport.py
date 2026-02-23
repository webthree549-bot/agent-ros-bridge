"""Integration tests for WebSocket transport."""

import pytest
import asyncio

from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


@pytest.mark.integration
@pytest.mark.asyncio
class TestWebSocketTransport:
    """Test WebSocket transport integration."""
    
    async def test_transport_creation(self):
        """Test creating WebSocket transport."""
        config = {
            "host": "localhost",
            "port": 8765,
            "auth": {"enabled": False}
        }
        
        transport = WebSocketTransport(config)
        
        assert transport.host == "localhost"
        assert transport.port == 8765
        assert transport.auth_enabled is False
    
    async def test_transport_with_tls_config(self):
        """Test WebSocket transport with TLS configuration."""
        config = {
            "host": "0.0.0.0",
            "port": 8765,
            "tls": {
                "cert": "/tmp/test.crt",
                "key": "/tmp/test.key"
            },
            "auth": {"enabled": True}
        }
        
        transport = WebSocketTransport(config)
        
        assert transport.tls_config is not None
        assert transport.auth_enabled is True
    
    async def test_client_tracking(self):
        """Test client connection tracking."""
        transport = WebSocketTransport({
            "host": "localhost",
            "port": 8765
        })
        
        # Initially no clients
        assert transport.get_client_count() == 0
    
    async def test_message_handler_registration(self):
        """Test registering message handlers."""
        transport = WebSocketTransport({"host": "localhost", "port": 8765})
        
        @transport.on_message
        async def handler(websocket, data):
            pass
        
        assert len(transport._message_handlers) == 1
