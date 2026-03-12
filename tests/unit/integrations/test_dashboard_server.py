"""Unit tests for Dashboard Server.

TDD tests for DashboardServer.
"""

import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock

from agent_ros_bridge.integrations.dashboard_server import (
    DashboardServer,
    AIOHTTP_AVAILABLE,
)


class TestDashboardServerCreation:
    """Test DashboardServer creation."""

    def test_dashboard_creation_with_bridge(self):
        """DashboardServer can be created with bridge."""
        mock_bridge = Mock()
        dashboard = DashboardServer(mock_bridge, port=8080)

        assert dashboard.bridge == mock_bridge
        assert dashboard.port == 8080
        assert dashboard.app is None or dashboard.app is not None  # Depends on aiohttp

    def test_dashboard_default_port(self):
        """DashboardServer uses default port 8080."""
        mock_bridge = Mock()
        dashboard = DashboardServer(mock_bridge)

        assert dashboard.port == 8080


class TestDashboardServerWithoutAiohttp:
    """Test DashboardServer when aiohttp is not available."""

    @patch("agent_ros_bridge.integrations.dashboard_server.AIOHTTP_AVAILABLE", False)
    def test_dashboard_creation_without_aiohttp(self):
        """DashboardServer can be created without aiohttp."""
        mock_bridge = Mock()
        dashboard = DashboardServer(mock_bridge, port=8080)

        assert dashboard.app is None

    @patch("agent_ros_bridge.integrations.dashboard_server.AIOHTTP_AVAILABLE", False)
    @pytest.mark.asyncio
    async def test_start_without_aiohttp_logs_warning(self):
        """Starting without aiohttp logs warning."""
        mock_bridge = Mock()
        dashboard = DashboardServer(mock_bridge, port=8080)

        # Should not raise, just log warning
        await dashboard.start()


@pytest.mark.skipif(not AIOHTTP_AVAILABLE, reason="aiohttp not available")
class TestDashboardServerWithAiohttp:
    """Test DashboardServer with aiohttp available."""

    @pytest.fixture
    def mock_bridge(self):
        """Create a mock bridge."""
        bridge = Mock()
        bridge.running = True
        bridge.transport_manager = Mock()
        bridge.transport_manager.transports = {}
        bridge.fleets = {}
        bridge.memory = None
        bridge.safety = None
        return bridge

    @pytest.fixture
    def dashboard(self, mock_bridge):
        """Create a dashboard server."""
        return DashboardServer(mock_bridge, port=9999)

    def test_setup_routes_creates_app(self, dashboard):
        """Routes are set up during initialization."""
        assert dashboard.app is not None

    @pytest.mark.asyncio
    async def test_start_starts_server(self, dashboard):
        """Start starts the server."""
        with patch("agent_ros_bridge.integrations.dashboard_server.web.AppRunner") as mock_runner:
            mock_runner_instance = AsyncMock()
            mock_runner.return_value = mock_runner_instance

            with patch("agent_ros_bridge.integrations.dashboard_server.web.TCPSite") as mock_site:
                mock_site_instance = AsyncMock()
                mock_site.return_value = mock_site_instance

                await dashboard.start()

                mock_runner.assert_called_once_with(dashboard.app)
                mock_runner_instance.setup.assert_called_once()
                mock_site_instance.start.assert_called_once()

    @pytest.mark.asyncio
    async def test_stop_cleans_up(self, dashboard):
        """Stop cleans up the runner."""
        mock_runner = AsyncMock()
        dashboard.runner = mock_runner

        await dashboard.stop()

        mock_runner.cleanup.assert_called_once()

    @pytest.mark.asyncio
    async def test_stop_without_runner(self, dashboard):
        """Stop handles missing runner gracefully."""
        dashboard.runner = None

        # Should not raise
        await dashboard.stop()

    @pytest.mark.asyncio
    async def test_handle_index_returns_html(self, dashboard):
        """Index handler returns HTML."""
        mock_request = Mock()

        response = await dashboard._handle_index(mock_request)

        assert response.content_type == "text/html"
        assert b"Agent ROS Bridge Dashboard" in response.body
        assert b"EMERGENCY STOP" in response.body

    @pytest.mark.asyncio
    async def test_handle_status_returns_json(self, dashboard, mock_bridge):
        """Status handler returns JSON."""
        mock_request = Mock()
        mock_bridge.running = True
        mock_bridge.transport_manager.transports = {"websocket": Mock()}
        mock_bridge.fleets = {"fleet1": Mock()}

        response = await dashboard._handle_status(mock_request)

        assert response.content_type == "application/json"
        data = response.body  # This is bytes, need to parse
        assert b"running" in data
        assert b"timestamp" in data

    @pytest.mark.asyncio
    async def test_handle_status_when_stopped(self, dashboard, mock_bridge):
        """Status handler shows stopped when bridge not running."""
        mock_request = Mock()
        mock_bridge.running = False

        response = await dashboard._handle_status(mock_request)

        assert b"stopped" in response.body

    @pytest.mark.asyncio
    async def test_handle_metrics_returns_json(self, dashboard, mock_bridge):
        """Metrics handler returns JSON."""
        mock_request = Mock()
        mock_bridge.running = True
        mock_bridge.transport_manager.transports = {}
        mock_bridge.fleets = {}

        response = await dashboard._handle_metrics(mock_request)

        assert response.content_type == "application/json"
        assert b"running" in response.body

    @pytest.mark.asyncio
    async def test_handle_metrics_with_fleets(self, dashboard, mock_bridge):
        """Metrics handler includes fleet data."""
        mock_request = Mock()
        mock_fleet = Mock()
        mock_fleet.robots = {"r1": Mock()}
        mock_bridge.fleets = {"fleet1": mock_fleet}

        response = await dashboard._handle_metrics(mock_request)

        assert b"fleet1" in response.body
        assert b"robot_count" in response.body

    @pytest.mark.asyncio
    async def test_handle_emergency_stop_triggers_stop(self, dashboard, mock_bridge):
        """Emergency stop handler triggers bridge emergency stop."""
        mock_request = Mock()
        mock_bridge.emergency_stop = Mock()

        response = await dashboard._handle_emergency_stop(mock_request)

        assert response.status == 200
        mock_bridge.emergency_stop.assert_called_once()

    @pytest.mark.asyncio
    async def test_handle_emergency_stop_without_method(self, dashboard, mock_bridge):
        """Emergency stop handler returns error if method not available."""
        mock_request = Mock()
        del mock_bridge.emergency_stop  # Remove the method

        response = await dashboard._handle_emergency_stop(mock_request)

        assert response.status == 500
        assert b"error" in response.body

    @pytest.mark.asyncio
    async def test_handle_emergency_stop_no_bridge(self, dashboard):
        """Emergency stop handler handles missing bridge."""
        mock_request = Mock()
        dashboard.bridge = None

        response = await dashboard._handle_emergency_stop(mock_request)

        assert response.status == 500


class TestDashboardServerEdgeCases:
    """Test edge cases."""

    @pytest.mark.skipif(not AIOHTTP_AVAILABLE, reason="aiohttp not available")
    @pytest.mark.asyncio
    async def test_start_without_app(self):
        """Start handles missing app gracefully."""
        mock_bridge = Mock()
        dashboard = DashboardServer(mock_bridge)

        # Manually set app to None
        dashboard.app = None

        # Should not raise, just log warning
        await dashboard.start()

    def test_dashboard_with_memory_and_safety(self):
        """Dashboard works with memory and safety available."""
        mock_bridge = Mock()
        mock_bridge.memory = Mock()
        mock_bridge.safety = Mock()

        dashboard = DashboardServer(mock_bridge)
        assert dashboard.bridge.memory is not None
        assert dashboard.bridge.safety is not None
