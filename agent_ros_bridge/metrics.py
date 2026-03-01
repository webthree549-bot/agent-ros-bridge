"""Metrics and observability for Agent ROS Bridge.

Exports Prometheus metrics and supports OpenTelemetry tracing.

Example:
    from agent_ros_bridge.metrics import MetricsExporter

    metrics = MetricsExporter(bridge)
    await metrics.start(port=9090)

    # Metrics available at http://localhost:9090/metrics
"""

import asyncio
import logging
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

try:
    from prometheus_client import (
        CONTENT_TYPE_LATEST,
        Counter,
        Gauge,
        Histogram,
        Info,
        generate_latest,
        start_http_server,
    )

    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class MetricConfig:
    """Configuration for metrics collection."""

    enabled: bool = True
    port: int = 9090
    host: str = "0.0.0.0"
    namespace: str = "agent_ros_bridge"


class MetricsCollector:
    """Collect and export metrics for the ROS Bridge.

    Tracks:
    - Action execution counts and latency
    - Connected agents
    - Topic data age
    - Transport connections
    - Error rates
    """

    def __init__(self, bridge, config: Optional[MetricConfig] = None):
        """Initialize metrics collector.

        Args:
            bridge: ROSBridge instance
            config: Metrics configuration
        """
        if not PROMETHEUS_AVAILABLE:
            raise ImportError(
                "Prometheus metrics require prometheus_client: pip install prometheus-client"
            )

        self.bridge = bridge
        self.config = config or MetricConfig()
        self._server = None
        self._running = False

        # Initialize metrics
        self._init_metrics()

    def _init_metrics(self):
        """Initialize Prometheus metrics."""
        ns = self.config.namespace

        # Action metrics
        self.action_counter = Counter(
            f"{ns}_actions_total", "Total actions executed", ["action", "status"]
        )

        self.action_latency = Histogram(
            f"{ns}_action_latency_seconds",
            "Action execution latency",
            ["action"],
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0],
        )

        # Connection metrics
        self.connected_agents = Gauge(f"{ns}_connected_agents", "Number of connected agents")

        self.agent_sessions_total = Counter(
            f"{ns}_agent_sessions_total", "Total agent sessions created"
        )

        # Transport metrics
        self.transport_connections = Gauge(
            f"{ns}_transport_connections", "Number of transport connections", ["transport"]
        )

        # Topic metrics
        self.topic_data_age = Gauge(
            f"{ns}_topic_data_age_seconds", "Age of last topic data", ["topic"]
        )

        # Error metrics
        self.errors_total = Counter(f"{ns}_errors_total", "Total errors", ["type"])

        # System info
        self.info = Info(f"{ns}_info", "Bridge information")
        self.info.info({"version": "0.4.0", "ros_version": str(self.bridge.ros_version)})

    async def start(self):
        """Start metrics HTTP server."""
        from prometheus_client import start_http_server

        self._server = start_http_server(self.config.port, self.config.host)
        self._running = True

        # Start collection loop
        asyncio.create_task(self._collect_loop())

        logger.info(
            f"Metrics server started on http://{self.config.host}:{self.config.port}/metrics"
        )

    async def stop(self):
        """Stop metrics server."""
        self._running = False
        if self._server:
            self._server.shutdown()
        logger.info("Metrics server stopped")

    async def _collect_loop(self):
        """Background metrics collection loop."""
        while self._running:
            try:
                # Update connection counts
                self.connected_agents.set(len(self.bridge._sessions))

                # Update transport connections
                for transport in self.bridge.transport_manager.get_transports():
                    if hasattr(transport, "get_client_count"):
                        count = transport.get_client_count()
                        self.transport_connections.labels(transport=type(transport).__name__).set(
                            count
                        )

                await asyncio.sleep(10)  # Collect every 10 seconds
            except Exception as e:
                logger.error(f"Metrics collection error: {e}")
                await asyncio.sleep(10)

    def record_action(self, action: str, status: str, latency: float):
        """Record action execution metrics.

        Args:
            action: Action name
            status: "success" or "error"
            latency: Execution time in seconds
        """
        self.action_counter.labels(action=action, status=status).inc()
        self.action_latency.labels(action=action).observe(latency)

    def record_error(self, error_type: str):
        """Record an error.

        Args:
            error_type: Type of error (transport, action, etc.)
        """
        self.errors_total.labels(type=error_type).inc()

    def record_session_created(self):
        """Record a new session."""
        self.agent_sessions_total.inc()

    def update_topic_age(self, topic: str, age_seconds: float):
        """Update topic data age.

        Args:
            topic: Topic name
            age_seconds: Age of last data
        """
        self.topic_data_age.labels(topic=topic).set(age_seconds)


class MetricsMiddleware:
    """Middleware to automatically collect metrics from bridge operations."""

    def __init__(self, collector: MetricsCollector):
        self.collector = collector

    @asynccontextmanager
    async def timed_action(self, action: str):
        """Context manager to time action execution.

        Usage:
            async with middleware.timed_action("navigate"):
                await bridge.call_action("navigate", x=5, y=3)
        """
        start = time.time()
        try:
            yield
            latency = time.time() - start
            self.collector.record_action(action, "success", latency)
        except Exception:
            latency = time.time() - start
            self.collector.record_action(action, "error", latency)
            self.collector.record_error("action")
            raise


class HealthChecker:
    """Health check endpoint for monitoring systems."""

    def __init__(self, bridge):
        self.bridge = bridge
        self._checks: Dict[str, Callable] = {}

    def register_check(self, name: str, check_func: Callable):
        """Register a health check.

        Args:
            name: Check name
            check_func: Async function returning (healthy: bool, message: str)
        """
        self._checks[name] = check_func

    async def check_health(self) -> Dict[str, Any]:
        """Run all health checks.

        Returns:
            Health status report
        """
        results = {}
        healthy = True

        for name, check in self._checks.items():
            try:
                is_healthy, message = await check()
                results[name] = {
                    "status": "healthy" if is_healthy else "unhealthy",
                    "message": message,
                }
                if not is_healthy:
                    healthy = False
            except Exception as e:
                results[name] = {"status": "error", "message": str(e)}
                healthy = False

        return {
            "status": "healthy" if healthy else "unhealthy",
            "checks": results,
            "timestamp": time.time(),
        }

    async def check_ready(self) -> bool:
        """Check if bridge is ready to accept connections.

        Returns:
            True if ready
        """
        # Check if transports are running
        for transport in self.bridge.transport_manager.get_transports():
            if hasattr(transport, "_running") and not transport._running:
                return False

        return True


# Convenience functions
def create_metrics(bridge, **kwargs) -> MetricsCollector:
    """Create a metrics collector.

    Args:
        bridge: ROSBridge instance
        **kwargs: Config options

    Returns:
        MetricsCollector instance
    """
    config = MetricConfig(**kwargs)
    return MetricsCollector(bridge, config)


async def start_metrics_server(bridge, port: int = 9090):
    """Quick start metrics server.

    Args:
        bridge: ROSBridge instance
        port: Port to listen on

    Returns:
        Running MetricsCollector
    """
    metrics = create_metrics(bridge, port=port)
    await metrics.start()
    return metrics


__all__ = [
    "MetricsCollector",
    "MetricsMiddleware",
    "HealthChecker",
    "MetricConfig",
    "create_metrics",
    "start_metrics_server",
]
