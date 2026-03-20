"""Additional tests for metrics module."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.metrics import (
    MetricsCollector,
    MetricsServer,
    MetricsSnapshot,
)


class TestMetricsRecordingAdditional:
    """Additional metrics recording tests."""

    def test_record_response_time(self):
        """Test recording response time."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter"):
                with patch("agent_ros_bridge.metrics.Gauge"):
                    with patch("agent_ros_bridge.metrics.Histogram") as mock_hist:
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()
                            collector.record_response_time(0.5)
                            mock_hist.return_value.observe.assert_called_with(0.5)

    def test_update_system_metrics_with_psutil(self):
        """Test updating system metrics."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            with patch.dict("sys.modules", {"psutil": MagicMock()}):
                import sys

                mock_psutil = MagicMock()
                mock_psutil.cpu_percent.return_value = 50.0
                mock_psutil.virtual_memory.return_value.used = 1024 * 1024 * 100  # 100MB
                sys.modules["psutil"] = mock_psutil

                collector = MetricsCollector()
                collector.update_system_metrics()

                assert collector._gauges.get("cpu_percent") == 50.0
                assert collector._gauges.get("memory_mb") == 100.0


class TestMetricsServerAdditional:
    """Additional metrics server tests."""

    @pytest.mark.asyncio
    async def test_start_custom_server(self):
        """Test starting custom server without prometheus."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            with patch("aiohttp.web.AppRunner") as mock_runner:
                with patch("aiohttp.web.TCPSite") as mock_site:
                    mock_runner_instance = AsyncMock()
                    mock_runner.return_value = mock_runner_instance
                    mock_site_instance = AsyncMock()
                    mock_site.return_value = mock_site_instance

                    server = MetricsServer(port=8080)
                    await server.start()

                    assert server.running is True

    def test_stop_server(self):
        """Test stopping server."""
        server = MetricsServer()
        server.running = True
        server.stop()
        assert server.running is False


class TestMetricsSnapshotAdditional:
    """Additional snapshot tests."""

    def test_snapshot_with_all_fields(self):
        """Test snapshot with all fields populated."""
        snapshot = MetricsSnapshot(
            timestamp=1234567890.0,
            robots_total=10,
            robots_online=8,
            tasks_completed=100,
            tasks_failed=5,
            messages_sent=1000,
            messages_received=950,
            active_connections=25,
            cpu_percent=45.5,
            memory_mb=512.0,
        )
        assert snapshot.robots_total == 10
        assert snapshot.tasks_completed == 100
        assert snapshot.cpu_percent == 45.5
