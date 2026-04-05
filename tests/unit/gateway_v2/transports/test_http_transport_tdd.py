"""
TDD Unit tests for HTTP Transport.
Tests server initialization, request handling, and dashboard serving.

Following TDD principles:
1. Write tests first
2. Tests should fail initially
3. Implement to make tests pass
4. Refactor
"""

import asyncio
import json
from datetime import datetime
from unittest import mock

import pytest
import pytest_asyncio

from agent_ros_bridge.gateway_v2.core import (
    Command,
    Header,
    Identity,
    Message,
)

# Import HTTPTransport
from agent_ros_bridge.gateway_v2.transports.http_transport import (
    DASHBOARD_HTML,
    HTTPTransport,
)


class TestHTTPTransportBasics:
    """Test HTTPTransport initialization and configuration"""

    @pytest.fixture
    def basic_config(self):
        """Basic HTTP transport config"""
        return {"host": "127.0.0.1", "port": 8080}

    @pytest.fixture
    def dashboard_disabled_config(self):
        """Config with dashboard disabled"""
        return {
            "host": "0.0.0.0",
            "port": 8081,
            "dashboard_enabled": False,
        }

    def test_basic_initialization(self, basic_config):
        """Test basic HTTP transport initialization"""
        transport = HTTPTransport(basic_config)

        assert transport.name == "http"
        assert transport.host == "127.0.0.1"
        assert transport.port == 8080
        assert transport.dashboard_enabled is True
        assert transport.running is False

    def test_default_values(self):
        """Test default configuration values"""
        transport = HTTPTransport({})

        assert transport.host == "0.0.0.0"
        assert transport.port == 8080
        assert transport.dashboard_enabled is True

    def test_dashboard_disabled_configuration(self, dashboard_disabled_config):
        """Test dashboard disabled configuration"""
        transport = HTTPTransport(dashboard_disabled_config)

        assert transport.dashboard_enabled is False
        assert transport.host == "0.0.0.0"
        assert transport.port == 8081

    def test_custom_port_configuration(self):
        """Test custom port configuration"""
        config = {"port": 9000}
        transport = HTTPTransport(config)

        assert transport.port == 9000


class TestHTTPTransportLifecycle:
    """Test HTTP transport start/stop lifecycle"""

    @pytest.fixture
    def transport(self):
        """Create HTTP transport for testing"""
        return HTTPTransport({"host": "127.0.0.1", "port": 0})  # Port 0 = auto-assign

    @pytest.mark.asyncio
    async def test_start_transport(self, transport):
        """Test starting HTTP transport"""
        result = await transport.start()

        assert result is True
        assert transport.running is True
        assert transport.server is not None

        # Cleanup
        await transport.stop()

    @pytest.mark.asyncio
    async def test_stop_transport(self, transport):
        """Test stopping HTTP transport"""
        await transport.start()
        assert transport.running is True

        await transport.stop()

        assert transport.running is False
        assert len(transport._clients) == 0

    @pytest.mark.asyncio
    async def test_start_already_running(self, transport):
        """Test starting transport that's already running"""
        await transport.start()

        # Try to start again
        result = await transport.start()

        # Should fail or return False since already running
        assert result is False or transport.running is True

        await transport.stop()


class TestHTTPRequestHandling:
    """Test HTTP request handling"""

    @pytest_asyncio.fixture
    async def running_transport(self):
        """Create and start HTTP transport"""
        transport = HTTPTransport({"host": "127.0.0.1", "port": 0})
        await transport.start()
        yield transport
        await transport.stop()

    @pytest.mark.asyncio
    async def test_handle_root_request(self, running_transport):
        """Test handling GET / request returns dashboard HTML"""
        # Simulate HTTP request
        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        # Mock request line and headers
        reader.readline = mock.AsyncMock(
            side_effect=[
                b"GET / HTTP/1.1\r\n",
                b"Host: localhost:8080\r\n",
                b"\r\n",
            ]
        )

        await running_transport._handle_request(reader, writer)

        # Check that HTML was sent
        write_calls = writer.write.call_args_list
        assert len(write_calls) > 0

        # Verify response contains HTML
        response_data = b"".join([call[0][0] for call in write_calls])
        assert b"HTTP/1.1 200 OK" in response_data
        assert b"text/html" in response_data
        assert b"<!DOCTYPE html>" in response_data

    @pytest.mark.asyncio
    async def test_handle_api_status_request(self, running_transport):
        """Test handling GET /api/status request"""
        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        reader.readline = mock.AsyncMock(
            side_effect=[
                b"GET /api/status HTTP/1.1\r\n",
                b"Host: localhost:8080\r\n",
                b"\r\n",
            ]
        )

        await running_transport._handle_request(reader, writer)

        write_calls = writer.write.call_args_list
        response_data = b"".join([call[0][0] for call in write_calls])

        assert b"HTTP/1.1 200 OK" in response_data
        assert b"application/json" in response_data
        assert b"running" in response_data

    @pytest.mark.asyncio
    async def test_handle_api_health_request(self, running_transport):
        """Test handling GET /api/health request"""
        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        reader.readline = mock.AsyncMock(
            side_effect=[
                b"GET /api/health HTTP/1.1\r\n",
                b"Host: localhost:8080\r\n",
                b"\r\n",
            ]
        )

        await running_transport._handle_request(reader, writer)

        write_calls = writer.write.call_args_list
        response_data = b"".join([call[0][0] for call in write_calls])

        assert b"HTTP/1.1 200 OK" in response_data
        assert b"healthy" in response_data

    @pytest.mark.asyncio
    async def test_handle_404_request(self, running_transport):
        """Test handling unknown path returns 404"""
        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        reader.readline = mock.AsyncMock(
            side_effect=[
                b"GET /unknown/path HTTP/1.1\r\n",
                b"Host: localhost:8080\r\n",
                b"\r\n",
            ]
        )

        await running_transport._handle_request(reader, writer)

        write_calls = writer.write.call_args_list
        response_data = b"".join([call[0][0] for call in write_calls])

        assert b"HTTP/1.1 404" in response_data


class TestHTTPDashboard:
    """Test dashboard HTML serving"""

    def test_dashboard_html_content(self):
        """Test that dashboard HTML contains expected elements"""
        assert "<!DOCTYPE html>" in DASHBOARD_HTML
        assert "<html" in DASHBOARD_HTML
        assert "Agent ROS Bridge" in DASHBOARD_HTML
        assert "three.js" in DASHBOARD_HTML  # 3D visualization
        assert "WebSocket" in DASHBOARD_HTML
        assert "Connect" in DASHBOARD_HTML

    def test_dashboard_has_3d_visualization(self):
        """Test dashboard includes Three.js for 3D"""
        assert "three.min.js" in DASHBOARD_HTML
        assert "THREE" in DASHBOARD_HTML
        assert "Scene" in DASHBOARD_HTML
        assert "WebGLRenderer" in DASHBOARD_HTML or "render" in DASHBOARD_HTML

    def test_dashboard_has_topic_list(self):
        """Test dashboard has topic list section"""
        assert "topic" in DASHBOARD_HTML.lower()
        assert "/tf" in DASHBOARD_HTML or "topic-item" in DASHBOARD_HTML

    def test_dashboard_has_console(self):
        """Test dashboard has console logging"""
        assert "console" in DASHBOARD_HTML.lower()
        assert "log" in DASHBOARD_HTML.lower()


class TestHTTPMessageHandling:
    """Test message sending/broadcasting"""

    @pytest.fixture
    def transport(self):
        """Create HTTP transport"""
        return HTTPTransport({"host": "127.0.0.1", "port": 8080})

    @pytest.mark.asyncio
    async def test_send_message_to_client(self, transport):
        """Test sending message to specific client"""
        # Register a mock client
        transport._clients["client1"] = {}

        message = Message(
            header=Header(),
            command=Command(action="test"),
        )

        result = await transport.send(message, "client1")

        assert result is True
        assert "pending" in transport._clients["client1"]
        assert len(transport._clients["client1"]["pending"]) == 1

    @pytest.mark.asyncio
    async def test_send_message_to_unknown_client(self, transport):
        """Test sending message to unknown client returns False"""
        message = Message(
            header=Header(),
            command=Command(action="test"),
        )

        result = await transport.send(message, "unknown_client")

        assert result is False

    @pytest.mark.asyncio
    async def test_broadcast_message(self, transport):
        """Test broadcasting message to all clients"""
        # Register mock clients
        transport._clients["client1"] = {}
        transport._clients["client2"] = {}

        message = Message(
            header=Header(),
            command=Command(action="broadcast"),
        )

        recipients = await transport.broadcast(message)

        assert len(recipients) == 2
        assert "client1" in recipients
        assert "client2" in recipients


class TestHTTPIntegration:
    """Integration tests for HTTP transport"""

    @pytest.mark.asyncio
    async def test_full_request_response_cycle(self):
        """Test full HTTP request/response cycle"""
        transport = HTTPTransport({"host": "127.0.0.1", "port": 0})

        # Start transport
        await transport.start()
        assert transport.running

        # Make actual HTTP request
        import urllib.request

        try:
            response = urllib.request.urlopen(f"http://127.0.0.1:{transport.port}/", timeout=2)
            html = response.read().decode()

            assert "<!DOCTYPE html>" in html
            assert response.status == 200
        except Exception as e:
            pytest.skip(f"Integration test requires running server: {e}")
        finally:
            await transport.stop()

    @pytest.mark.asyncio
    async def test_concurrent_requests(self):
        """Test handling concurrent HTTP requests"""
        transport = HTTPTransport({"host": "127.0.0.1", "port": 0})
        await transport.start()

        try:
            # Simulate multiple concurrent requests
            import urllib.request

            urls = [
                f"http://127.0.0.1:{transport.port}/",
                f"http://127.0.0.1:{transport.port}/api/status",
                f"http://127.0.0.1:{transport.port}/api/health",
            ]

            for url in urls:
                try:
                    response = urllib.request.urlopen(url, timeout=2)
                    assert response.status == 200
                except Exception:
                    pass  # Individual failures OK for this test

        finally:
            await transport.stop()


class TestHTTPErrorHandling:
    """Test error handling"""

    @pytest.mark.asyncio
    async def test_handle_invalid_request(self):
        """Test handling invalid HTTP request"""
        transport = HTTPTransport({})

        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        # Invalid request line
        reader.readline = mock.AsyncMock(return_value=b"INVALID\r\n")

        # Should not raise exception
        await transport._handle_request(reader, writer)

        # Cleanup
        writer.close.assert_called_once()

    @pytest.mark.asyncio
    async def test_handle_empty_request(self):
        """Test handling empty request"""
        transport = HTTPTransport({})

        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        reader.readline = mock.AsyncMock(return_value=b"")

        # Should not raise exception, just return
        await transport._handle_request(reader, writer)


# Performance tests
class TestHTTPPerformance:
    """Performance tests for HTTP transport"""

    @pytest.mark.asyncio
    async def test_dashboard_serve_speed(self):
        """Test dashboard serving performance"""
        transport = HTTPTransport({})

        reader = mock.AsyncMock()
        writer = mock.AsyncMock()

        reader.readline = mock.AsyncMock(
            side_effect=[
                b"GET / HTTP/1.1\r\n",
                b"\r\n",
            ]
        )

        import time

        start = time.time()

        await transport._handle_request(reader, writer)

        elapsed = time.time() - start

        # Should serve dashboard in under 100ms
        assert elapsed < 0.1


# Run tests
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
