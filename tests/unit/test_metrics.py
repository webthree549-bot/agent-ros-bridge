"""Tests for metrics module."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

from agent_ros_bridge.metrics import (
    MetricsCollector,
    MetricsServer,
    MetricsSnapshot,
    get_metrics,
)


class TestMetricsSnapshot:
    """Test MetricsSnapshot dataclass."""

    def test_snapshot_creation(self):
        """Test creating a snapshot."""
        snapshot = MetricsSnapshot(
            timestamp=1234567890.0,
            robots_total=5,
            robots_online=3,
        )
        assert snapshot.timestamp == 1234567890.0
        assert snapshot.robots_total == 5
        assert snapshot.robots_online == 3

    def test_snapshot_defaults(self):
        """Test snapshot defaults."""
        snapshot = MetricsSnapshot(timestamp=0.0)
        assert snapshot.robots_total == 0
        assert snapshot.tasks_completed == 0


class TestMetricsCollectorInitialization:
    """Test metrics collector initialization."""

    def test_init_default(self):
        """Test default initialization."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            assert collector.namespace == "agent_ros_bridge"
            assert collector._initialized is False

    def test_init_custom_namespace(self):
        """Test custom namespace."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector(namespace="custom")
            assert collector.namespace == "custom"

    def test_init_with_prometheus(self):
        """Test initialization with prometheus."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.CollectorRegistry"):
                with patch("agent_ros_bridge.metrics.Counter"):
                    with patch("agent_ros_bridge.metrics.Gauge"):
                        with patch("agent_ros_bridge.metrics.Histogram"):
                            with patch("agent_ros_bridge.metrics.Info"):
                                collector = MetricsCollector()
                                assert collector._initialized is True


class TestMetricsRecording:
    """Test recording metrics."""

    def test_record_message_sent(self):
        """Test recording message sent."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.record_message_sent("websocket", 100)
            assert collector._counters["messages_sent"] == 1

    def test_record_message_received(self):
        """Test recording message received."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.record_message_received("websocket", 100)
            assert collector._counters["messages_received"] == 1

    def test_record_task_completed(self):
        """Test recording task completion."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.record_task_completed("navigate", 5.0)
            assert collector._counters["tasks_completed"] == 1

    def test_record_task_failed(self):
        """Test recording task failure."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.record_task_failed("navigate")
            assert collector._counters["tasks_failed"] == 1

    def test_record_connection_opened(self):
        """Test recording connection."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.record_connection_opened("websocket")
            assert collector._counters["connections_total"] == 1

    def test_set_robots_online(self):
        """Test setting robots online."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_online(5)
            assert collector._gauges["robots_online"] == 5

    def test_set_robots_total(self):
        """Test setting total robots."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_total(10)
            assert collector._gauges["robots_total"] == 10

    def test_set_active_connections(self):
        """Test setting active connections."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_active_connections(3, "websocket")
            assert collector._gauges["active_connections"] == 3

    def test_set_task_queue_size(self):
        """Test setting task queue size."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_task_queue_size(5)
            assert collector._gauges["task_queue_size"] == 5


class TestMetricsSnapshot:
    """Test getting metrics snapshot."""

    def test_get_snapshot(self):
        """Test getting snapshot."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_online(3)
            collector.set_robots_total(5)
            collector.record_message_sent()

            snapshot = collector.get_snapshot()
            assert isinstance(snapshot, MetricsSnapshot)
            assert snapshot.robots_online == 3
            assert snapshot.robots_total == 5
            assert snapshot.messages_sent == 1


class TestMetricsText:
    """Test metrics text output."""

    def test_get_metrics_text_no_prometheus(self):
        """Test text output without prometheus."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_online(3)

            text = collector.get_metrics_text()
            assert "robots_online 3" in text
            assert "Agent ROS Bridge Metrics" in text


class TestMetricsServer:
    """Test metrics server."""

    def test_init_default(self):
        """Test default initialization."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            server = MetricsServer()
            assert server.port == 9090
            assert server.collector is not None
            assert server.running is False

    def test_init_custom_port(self):
        """Test custom port."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            server = MetricsServer(port=8080)
            assert server.port == 8080

    def test_init_with_collector(self):
        """Test with custom collector."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            server = MetricsServer(collector=collector)
            assert server.collector is collector


class TestGetMetrics:
    """Test get_metrics convenience function."""

    def test_get_metrics_singleton(self):
        """Test that get_metrics returns singleton."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            # Reset global
            import agent_ros_bridge.metrics as metrics_module

            metrics_module._global_metrics = None

            m1 = get_metrics()
            m2 = get_metrics()
            assert m1 is m2
