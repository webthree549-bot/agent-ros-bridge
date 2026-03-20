"""Additional tests for metrics module - Part 3 (Fixed)."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, Mock, call, patch

import pytest

from agent_ros_bridge.metrics import (
    MetricsCollector,
    MetricsServer,
    MetricsSnapshot,
    get_metrics,
)


class TestMetricsCollectorWithPrometheus:
    """Test metrics collector with Prometheus available."""

    def test_init_with_prometheus_available(self):
        """Test initialization when Prometheus is available."""
        # Mock the prometheus_client module
        mock_prometheus = MagicMock()
        mock_prometheus.CollectorRegistry = MagicMock
        mock_prometheus.Counter = MagicMock
        mock_prometheus.Gauge = MagicMock
        mock_prometheus.Histogram = MagicMock
        mock_prometheus.Info = MagicMock

        with patch.dict("sys.modules", {"prometheus_client": mock_prometheus}):
            with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
                collector = MetricsCollector()
                assert collector._initialized is True

    def test_record_methods_with_prometheus(self):
        """Test recording methods with Prometheus."""
        mock_prometheus = MagicMock()
        mock_counter = MagicMock()
        mock_gauge = MagicMock()
        mock_hist = MagicMock()
        mock_info = MagicMock()

        mock_prometheus.Counter = Mock(return_value=mock_counter)
        mock_prometheus.Gauge = Mock(return_value=mock_gauge)
        mock_prometheus.Histogram = Mock(return_value=mock_hist)
        mock_prometheus.Info = Mock(return_value=mock_info)
        mock_prometheus.CollectorRegistry = Mock

        with patch.dict("sys.modules", {"prometheus_client": mock_prometheus}):
            with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
                collector = MetricsCollector()

                # Test recording
                collector.record_message_sent("websocket", 1024)
                collector.record_message_received("mqtt", 512)
                collector.record_task_completed("navigate", 5.5)
                collector.record_task_failed("manipulate")
                collector.record_connection_opened("grpc")
                collector.record_response_time(0.1)

                # All should succeed without error
                assert True


class TestMetricsServerStart:
    """Test metrics server start."""

    @pytest.mark.asyncio
    async def test_start_with_prometheus(self):
        """Test starting server with Prometheus."""
        mock_prometheus = MagicMock()
        mock_prometheus.start_http_server = Mock(return_value=MagicMock())
        mock_prometheus.CollectorRegistry = Mock
        mock_prometheus.Counter = Mock
        mock_prometheus.Gauge = Mock
        mock_prometheus.Histogram = Mock
        mock_prometheus.Info = Mock

        with patch.dict("sys.modules", {"prometheus_client": mock_prometheus}):
            with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
                collector = MetricsCollector()
                server = MetricsServer(port=9090, collector=collector)

                await server.start()

                assert server.running is True
                mock_prometheus.start_http_server.assert_called_once()


class TestMetricsSnapshotCoverage:
    """Test metrics snapshot for coverage."""

    def test_snapshot_all_fields(self):
        """Test snapshot with all fields."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()

            # Set all metrics
            collector.set_robots_total(10)
            collector.set_robots_online(8)
            collector.set_active_connections(5, "websocket")
            collector.set_task_queue_size(3)
            collector.record_message_sent("ws", 100)
            collector.record_message_received("ws", 100)
            collector.record_task_completed("nav", 1.0)
            collector.record_task_failed("nav")
            collector.record_connection_opened("ws")

            snapshot = collector.get_snapshot()

            assert snapshot.robots_total == 10
            assert snapshot.robots_online == 8
            assert snapshot.messages_sent == 1
            assert snapshot.messages_received == 1
            assert snapshot.tasks_completed == 1
            assert snapshot.tasks_failed == 1
            assert snapshot.active_connections == 5


class TestGetMetricsMultiple:
    """Test get_metrics with multiple calls."""

    def test_get_metrics_multiple_calls_same_instance(self):
        """Test that multiple calls return same instance."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            # Reset global
            import agent_ros_bridge.metrics as metrics_module

            original = metrics_module._global_metrics
            metrics_module._global_metrics = None

            try:
                m1 = get_metrics()
                m2 = get_metrics()
                m3 = get_metrics()

                # All should be same object
                assert m1 is m2
                assert m2 is m3
            finally:
                metrics_module._global_metrics = original
