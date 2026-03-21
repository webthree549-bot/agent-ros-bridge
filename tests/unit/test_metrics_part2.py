"""Additional tests for metrics module - Part 2."""

import asyncio
from unittest.mock import AsyncMock, MagicMock, Mock, call, patch

import pytest

from agent_ros_bridge.metrics import (
    PROMETHEUS_AVAILABLE,
    MetricsCollector,
    MetricsServer,
    MetricsSnapshot,
    get_metrics,
)

# Skip all tests in this file if prometheus_client is not available
pytestmark = pytest.mark.skipif(not PROMETHEUS_AVAILABLE, reason="prometheus_client not available")


class TestMetricsCollectorPrometheus:
    """Test metrics collector with Prometheus."""

    def test_init_creates_all_metrics(self):
        """Test that all metrics are created."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.CollectorRegistry"):
                with patch("agent_ros_bridge.metrics.Counter") as mock_counter:
                    with patch("agent_ros_bridge.metrics.Gauge") as mock_gauge:
                        with patch("agent_ros_bridge.metrics.Histogram") as mock_hist:
                            with patch("agent_ros_bridge.metrics.Info") as mock_info:
                                collector = MetricsCollector()

                                # Should create multiple counters
                                assert mock_counter.call_count >= 4
                                # Should create multiple gauges
                                assert mock_gauge.call_count >= 5
                                # Should create histograms
                                assert mock_hist.call_count >= 3
                                # Should create info
                                assert mock_info.call_count == 1


class TestMetricsCollectorRecordingDetailed:
    """Detailed recording tests."""

    def test_record_message_sent_with_size(self):
        """Test recording message with size."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter"):
                with patch("agent_ros_bridge.metrics.Gauge"):
                    with patch("agent_ros_bridge.metrics.Histogram") as mock_hist:
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()
                            collector.record_message_sent("websocket", 1024)

                            # Should observe message size
                            mock_hist.return_value.observe.assert_called_with(1024)

    def test_record_message_received_with_size(self):
        """Test recording received message with size."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter"):
                with patch("agent_ros_bridge.metrics.Gauge"):
                    with patch("agent_ros_bridge.metrics.Histogram") as mock_hist:
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()
                            collector.record_message_received("mqtt", 512)

                            mock_hist.return_value.observe.assert_called_with(512)

    def test_record_task_completed_with_duration(self):
        """Test recording task with duration."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter"):
                with patch("agent_ros_bridge.metrics.Gauge"):
                    with patch("agent_ros_bridge.metrics.Histogram") as mock_hist:
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()
                            collector.record_task_completed("navigate", 5.5)

                            mock_hist.return_value.observe.assert_called_with(5.5)

    def test_record_task_failed(self):
        """Test recording failed task."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter") as mock_counter:
                with patch("agent_ros_bridge.metrics.Gauge"):
                    with patch("agent_ros_bridge.metrics.Histogram"):
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()
                            collector.record_task_failed("manipulate")

                            # Should increment failed counter
                            mock_counter.return_value.labels.assert_called_with(status="failed")

    def test_set_robots_online_multiple(self):
        """Test setting robots online multiple times."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.Counter"):
                with patch("agent_ros_bridge.metrics.Gauge") as mock_gauge:
                    with patch("agent_ros_bridge.metrics.Histogram"):
                        with patch("agent_ros_bridge.metrics.Info"):
                            collector = MetricsCollector()

                            collector.set_robots_online(5)
                            collector.set_robots_online(3)
                            collector.set_robots_online(8)

                            # Should call set each time
                            assert mock_gauge.return_value.set.call_count == 3


class TestMetricsSnapshotDetailed:
    """Detailed snapshot tests."""

    def test_snapshot_from_collector(self):
        """Test getting snapshot from collector."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_total(10)
            collector.set_robots_online(8)
            collector.record_message_sent()
            collector.record_message_received()
            collector.record_task_completed()

            snapshot = collector.get_snapshot()

            assert snapshot.robots_total == 10
            assert snapshot.robots_online == 8
            assert snapshot.messages_sent == 1
            assert snapshot.messages_received == 1
            assert snapshot.tasks_completed == 1


class TestMetricsTextFormat:
    """Test metrics text format output."""

    def test_get_metrics_text_with_prometheus(self):
        """Test text format with Prometheus."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", True):
            with patch("agent_ros_bridge.metrics.generate_latest") as mock_generate:
                with patch("agent_ros_bridge.metrics.CollectorRegistry"):
                    with patch("agent_ros_bridge.metrics.Counter"):
                        with patch("agent_ros_bridge.metrics.Gauge"):
                            with patch("agent_ros_bridge.metrics.Histogram"):
                                with patch("agent_ros_bridge.metrics.Info"):
                                    mock_generate.return_value = b"# HELP test\ntest 1\n"
                                    collector = MetricsCollector()

                                    text = collector.get_metrics_text()

                                    assert "# HELP test" in text
                                    assert "test 1" in text

    def test_get_metrics_text_without_prometheus(self):
        """Test text format without Prometheus."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            collector = MetricsCollector()
            collector.set_robots_online(5)
            collector.set_robots_total(10)

            text = collector.get_metrics_text()

            assert "Agent ROS Bridge Metrics" in text
            assert "robots_online 5" in text
            assert "robots_total 10" in text


class TestGetMetricsSingleton:
    """Test get_metrics singleton behavior."""

    def test_get_metrics_creates_instance(self):
        """Test that get_metrics creates instance."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            # Reset global
            import agent_ros_bridge.metrics as metrics_module

            original = metrics_module._global_metrics
            metrics_module._global_metrics = None

            try:
                metrics = get_metrics()
                assert metrics is not None
                assert isinstance(metrics, MetricsCollector)
            finally:
                metrics_module._global_metrics = original

    def test_get_metrics_returns_same_instance(self):
        """Test that get_metrics returns same instance."""
        with patch("agent_ros_bridge.metrics.PROMETHEUS_AVAILABLE", False):
            # Reset global
            import agent_ros_bridge.metrics as metrics_module

            original = metrics_module._global_metrics
            metrics_module._global_metrics = None

            try:
                m1 = get_metrics()
                m2 = get_metrics()
                m3 = get_metrics()

                assert m1 is m2 is m3
            finally:
                metrics_module._global_metrics = original
