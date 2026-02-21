#!/usr/bin/env python3
"""Prometheus metrics exporter for Agent ROS Bridge

Provides metrics for monitoring robot fleets, bridge performance,
and system health. Compatible with Prometheus + Grafana.

Metrics exposed:
- Robot status and availability
- Task execution statistics
- Message throughput
- Connection counts
- System resources

Usage:
    from agent_ros_bridge.metrics import MetricsServer
    
    metrics = MetricsServer(port=9090)
    await metrics.start()
    
    # Record metrics
    metrics.record_message_sent("websocket")
    metrics.record_task_completed(duration=5.2)
"""

import asyncio
import logging
import time
from typing import Dict, Optional
from dataclasses import dataclass, field
from collections import deque
from datetime import datetime

try:
    from prometheus_client import Counter, Histogram, Gauge, Info, start_http_server, CollectorRegistry
    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False

logger = logging.getLogger("metrics")


@dataclass
class MetricsSnapshot:
    """Snapshot of current metrics"""
    timestamp: float
    robots_total: int = 0
    robots_online: int = 0
    tasks_completed: int = 0
    tasks_failed: int = 0
    messages_sent: int = 0
    messages_received: int = 0
    active_connections: int = 0
    cpu_percent: float = 0.0
    memory_mb: float = 0.0


class MetricsCollector:
    """Collects and exposes metrics for Prometheus"""
    
    def __init__(self, namespace: str = "agent_ros_bridge"):
        self.namespace = namespace
        self.registry = CollectorRegistry() if PROMETHEUS_AVAILABLE else None
        self._initialized = False
        
        # Initialize Prometheus metrics
        if PROMETHEUS_AVAILABLE:
            self._init_metrics()
        
        # Internal counters (for non-Prometheus mode)
        self._counters = {
            "messages_sent": 0,
            "messages_received": 0,
            "tasks_completed": 0,
            "tasks_failed": 0,
            "connections_total": 0,
        }
        
        self._gauges = {
            "robots_online": 0,
            "robots_total": 0,
            "active_connections": 0,
            "task_queue_size": 0,
        }
        
        # History for time-series data
        self._history = deque(maxlen=1000)
        self._start_time = time.time()
    
    def _init_metrics(self):
        """Initialize Prometheus metric objects"""
        ns = self.namespace
        
        # Info
        self.info = Info(f"{ns}_build", "Build information", registry=self.registry)
        self.info.info({"version": "0.1.0", "platform": "python"})
        
        # Counters
        self.messages_sent = Counter(
            f"{ns}_messages_sent_total",
            "Total messages sent",
            ["transport"],
            registry=self.registry
        )
        self.messages_received = Counter(
            f"{ns}_messages_received_total",
            "Total messages received",
            ["transport"],
            registry=self.registry
        )
        self.tasks_completed = Counter(
            f"{ns}_tasks_completed_total",
            "Total tasks completed",
            ["status"],
            registry=self.registry
        )
        self.connections_total = Counter(
            f"{ns}_connections_total",
            "Total connections established",
            ["transport"],
            registry=self.registry
        )
        
        # Gauges
        self.robots_online = Gauge(
            f"{ns}_robots_online",
            "Number of robots currently online",
            registry=self.registry
        )
        self.robots_total = Gauge(
            f"{ns}_robots_total",
            "Total number of registered robots",
            registry=self.registry
        )
        self.active_connections = Gauge(
            f"{ns}_active_connections",
            "Number of active connections",
            ["transport"],
            registry=self.registry
        )
        self.task_queue_size = Gauge(
            f"{ns}_task_queue_size",
            "Number of tasks in queue",
            registry=self.registry
        )
        self.uptime_seconds = Gauge(
            f"{ns}_uptime_seconds",
            "Bridge uptime in seconds",
            registry=self.registry
        )
        
        # Histograms
        self.task_duration = Histogram(
            f"{ns}_task_duration_seconds",
            "Task execution duration",
            ["task_type"],
            buckets=[0.1, 0.5, 1.0, 2.5, 5.0, 10.0, 30.0, 60.0],
            registry=self.registry
        )
        self.message_size = Histogram(
            f"{ns}_message_size_bytes",
            "Message size distribution",
            buckets=[100, 500, 1000, 5000, 10000, 50000],
            registry=self.registry
        )
        self.response_time = Histogram(
            f"{ns}_response_time_seconds",
            "Response time for commands",
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0],
            registry=self.registry
        )
        
        self._initialized = True
    
    # Recording methods
    def record_message_sent(self, transport: str = "websocket", size_bytes: int = 0):
        """Record message sent"""
        self._counters["messages_sent"] += 1
        if PROMETHEUS_AVAILABLE:
            self.messages_sent.labels(transport=transport).inc()
            if size_bytes > 0:
                self.message_size.observe(size_bytes)
    
    def record_message_received(self, transport: str = "websocket", size_bytes: int = 0):
        """Record message received"""
        self._counters["messages_received"] += 1
        if PROMETHEUS_AVAILABLE:
            self.messages_received.labels(transport=transport).inc()
            if size_bytes > 0:
                self.message_size.observe(size_bytes)
    
    def record_task_completed(self, task_type: str = "generic", duration_sec: float = 0.0):
        """Record task completion"""
        self._counters["tasks_completed"] += 1
        if PROMETHEUS_AVAILABLE:
            self.tasks_completed.labels(status="success").inc()
            if duration_sec > 0:
                self.task_duration.labels(task_type=task_type).observe(duration_sec)
    
    def record_task_failed(self, task_type: str = "generic"):
        """Record task failure"""
        self._counters["tasks_failed"] += 1
        if PROMETHEUS_AVAILABLE:
            self.tasks_completed.labels(status="failed").inc()
    
    def record_connection_opened(self, transport: str = "websocket"):
        """Record new connection"""
        self._counters["connections_total"] += 1
        if PROMETHEUS_AVAILABLE:
            self.connections_total.labels(transport=transport).inc()
    
    def set_robots_online(self, count: int):
        """Set number of online robots"""
        self._gauges["robots_online"] = count
        if PROMETHEUS_AVAILABLE:
            self.robots_online.set(count)
    
    def set_robots_total(self, count: int):
        """Set total number of robots"""
        self._gauges["robots_total"] = count
        if PROMETHEUS_AVAILABLE:
            self.robots_total.set(count)
    
    def set_active_connections(self, count: int, transport: str = "websocket"):
        """Set active connections"""
        self._gauges["active_connections"] = count
        if PROMETHEUS_AVAILABLE:
            self.active_connections.labels(transport=transport).set(count)
    
    def set_task_queue_size(self, count: int):
        """Set task queue size"""
        self._gauges["task_queue_size"] = count
        if PROMETHEUS_AVAILABLE:
            self.task_queue_size.set(count)
    
    def record_response_time(self, duration_sec: float):
        """Record command response time"""
        if PROMETHEUS_AVAILABLE:
            self.response_time.observe(duration_sec)
    
    def update_system_metrics(self):
        """Update system resource metrics"""
        try:
            import psutil
            cpu = psutil.cpu_percent()
            memory = psutil.virtual_memory()
            
            self._gauges["cpu_percent"] = cpu
            self._gauges["memory_mb"] = memory.used / 1024 / 1024
            
            if PROMETHEUS_AVAILABLE:
                self.uptime_seconds.set(time.time() - self._start_time)
        except ImportError:
            pass
    
    def get_snapshot(self) -> MetricsSnapshot:
        """Get current metrics snapshot"""
        self.update_system_metrics()
        
        return MetricsSnapshot(
            timestamp=time.time(),
            robots_total=self._gauges.get("robots_total", 0),
            robots_online=self._gauges.get("robots_online", 0),
            tasks_completed=self._counters["tasks_completed"],
            tasks_failed=self._counters["tasks_failed"],
            messages_sent=self._counters["messages_sent"],
            messages_received=self._counters["messages_received"],
            active_connections=self._gauges.get("active_connections", 0),
            cpu_percent=self._gauges.get("cpu_percent", 0),
            memory_mb=self._gauges.get("memory_mb", 0)
        )
    
    def get_metrics_text(self) -> str:
        """Get metrics in Prometheus text format"""
        if PROMETHEUS_AVAILABLE:
            from prometheus_client import generate_latest
            return generate_latest(self.registry).decode('utf-8')
        else:
            # Simple text format fallback
            snapshot = self.get_snapshot()
            return f"""# Agent ROS Bridge Metrics
robots_online {snapshot.robots_online}
robots_total {snapshot.robots_total}
tasks_completed {snapshot.tasks_completed}
tasks_failed {snapshot.tasks_failed}
messages_sent {snapshot.messages_sent}
messages_received {snapshot.messages_received}
active_connections {snapshot.active_connections}
"""


class MetricsServer:
    """HTTP server for exposing metrics"""
    
    def __init__(self, port: int = 9090, collector: Optional[MetricsCollector] = None):
        self.port = port
        self.collector = collector or MetricsCollector()
        self.server = None
        self.running = False
    
    async def start(self):
        """Start metrics server"""
        if PROMETHEUS_AVAILABLE:
            # Use prometheus_client's built-in server
            start_http_server(self.port, registry=self.collector.registry)
            logger.info(f"📊 Prometheus metrics server started on port {self.port}")
        else:
            # Start custom HTTP server
            await self._start_custom_server()
        
        self.running = True
    
    async def _start_custom_server(self):
        """Start custom metrics HTTP server (no prometheus_client)"""
        from aiohttp import web
        
        async def metrics_handler(request):
            text = self.collector.get_metrics_text()
            return web.Response(text=text, content_type="text/plain")
        
        async def health_handler(request):
            return web.Response(text="OK")
        
        app = web.Application()
        app.router.add_get("/metrics", metrics_handler)
        app.router.add_get("/health", health_handler)
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self.port)
        await site.start()
        
        logger.info(f"📊 Custom metrics server started on port {self.port}")
    
    def stop(self):
        """Stop metrics server"""
        self.running = False


# Global metrics instance
_global_metrics: Optional[MetricsCollector] = None


def get_metrics() -> MetricsCollector:
    """Get global metrics collector"""
    global _global_metrics
    if _global_metrics is None:
        _global_metrics = MetricsCollector()
    return _global_metrics
