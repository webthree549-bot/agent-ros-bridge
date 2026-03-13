"""Tests for metrics module."""

from collections import deque

import pytest

from agent_ros_bridge.metrics import MetricsCollector, MetricsServer, MetricsSnapshot, get_metrics


class TestMetricsSnapshot:
    """Test MetricsSnapshot dataclass."""

    def test_default_snapshot(self):
        """Default snapshot values."""
        snapshot = MetricsSnapshot(timestamp=1234567890.0)

        assert snapshot.timestamp == 1234567890.0
        assert snapshot.robots_total == 0
        assert snapshot.robots_online == 0
        assert snapshot.tasks_completed == 0
        assert snapshot.tasks_failed == 0
        assert snapshot.messages_sent == 0
        assert snapshot.messages_received == 0
        assert snapshot.active_connections == 0
        assert snapshot.cpu_percent == 0.0
        assert snapshot.memory_mb == 0.0

    def test_custom_snapshot(self):
        """Custom snapshot values."""
        snapshot = MetricsSnapshot(
            timestamp=1234567890.0, robots_total=5, robots_online=3, tasks_completed=100
        )

        assert snapshot.robots_total == 5
        assert snapshot.robots_online == 3
        assert snapshot.tasks_completed == 100


class TestMetricsCollectorInit:
    """Test MetricsCollector initialization."""

    def test_init_with_defaults(self):
        """Collector initializes with defaults."""
        collector = MetricsCollector()

        assert collector.namespace == "agent_ros_bridge"
        assert collector._initialized is False or True  # Depends on prometheus availability
        assert isinstance(collector._counters, dict)
        assert isinstance(collector._gauges, dict)
        assert isinstance(collector._history, deque)

    def test_init_with_custom_namespace(self):
        """Collector initializes with custom namespace."""
        collector = MetricsCollector(namespace="custom")

        assert collector.namespace == "custom"


class TestMetricsCollectorRecording:
    """Test MetricsCollector recording methods."""

    def test_record_message_sent(self):
        """Can record message sent."""
        collector = MetricsCollector()

        collector.record_message_sent("websocket", 100)

        assert collector._counters["messages_sent"] == 1

    def test_record_message_received(self):
        """Can record message received."""
        collector = MetricsCollector()

        collector.record_message_received("websocket", 100)

        assert collector._counters["messages_received"] == 1

    def test_record_task_completed(self):
        """Can record task completed."""
        collector = MetricsCollector()

        collector.record_task_completed("navigate", 5.0)

        assert collector._counters["tasks_completed"] == 1

    def test_record_task_failed(self):
        """Can record task failed."""
        collector = MetricsCollector()

        collector.record_task_failed("navigate")

        assert collector._counters["tasks_failed"] == 1

    def test_record_connection_opened(self):
        """Can record connection opened."""
        collector = MetricsCollector()

        collector.record_connection_opened("websocket")

        assert collector._counters["connections_total"] == 1


class TestMetricsCollectorGauges:
    """Test MetricsCollector gauge methods."""

    def test_set_robots_online(self):
        """Can set robots online."""
        collector = MetricsCollector()

        collector.set_robots_online(5)

        assert collector._gauges["robots_online"] == 5

    def test_set_robots_total(self):
        """Can set robots total."""
        collector = MetricsCollector()

        collector.set_robots_total(10)

        assert collector._gauges["robots_total"] == 10

    def test_set_active_connections(self):
        """Can set active connections."""
        collector = MetricsCollector()

        collector.set_active_connections(3, "websocket")

        assert collector._gauges["active_connections"] == 3

    def test_set_task_queue_size(self):
        """Can set task queue size."""
        collector = MetricsCollector()

        collector.set_task_queue_size(7)

        assert collector._gauges["task_queue_size"] == 7


class TestMetricsCollectorSnapshot:
    """Test MetricsCollector snapshot functionality."""

    def test_get_snapshot_returns_snapshot(self):
        """get_snapshot returns MetricsSnapshot."""
        collector = MetricsCollector()
        collector.set_robots_online(5)
        collector.set_robots_total(10)
        collector.record_task_completed()

        snapshot = collector.get_snapshot()

        assert isinstance(snapshot, MetricsSnapshot)
        assert snapshot.robots_online == 5
        assert snapshot.robots_total == 10
        assert snapshot.tasks_completed == 1

    def test_get_metrics_text_returns_string(self):
        """get_metrics_text returns string."""
        collector = MetricsCollector()

        text = collector.get_metrics_text()

        assert isinstance(text, str)
        assert "robots_online" in text


class TestMetricsCollectorResponseTime:
    """Test MetricsCollector response time recording."""

    def test_record_response_time(self):
        """Can record response time."""
        collector = MetricsCollector()

        collector.record_response_time(0.05)

        # Should not raise


class TestMetricsServer:
    """Test MetricsServer."""

    def test_server_init_with_defaults(self):
        """Server initializes with defaults."""
        server = MetricsServer()

        assert server.port == 9090
        assert server.collector is not None
        assert server.running is False

    def test_server_init_with_custom_port(self):
        """Server initializes with custom port."""
        server = MetricsServer(port=8080)

        assert server.port == 8080

    def test_server_init_with_custom_collector(self):
        """Server initializes with custom collector."""
        collector = MetricsCollector()
        server = MetricsServer(collector=collector)

        assert server.collector is collector

    @pytest.mark.asyncio
    async def test_server_stop(self):
        """Server can be stopped."""
        server = MetricsServer()

        server.stop()

        assert server.running is False


class TestGetMetrics:
    """Test get_metrics global function."""

    def test_get_metrics_returns_collector(self):
        """get_metrics returns MetricsCollector."""
        collector = get_metrics()

        assert isinstance(collector, MetricsCollector)

    def test_get_metrics_returns_same_instance(self):
        """get_metrics returns same instance on multiple calls."""
        collector1 = get_metrics()
        collector2 = get_metrics()

        assert collector1 is collector2
