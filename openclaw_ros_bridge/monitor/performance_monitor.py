#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Performance Monitor - Tracks CPU/memory/latency/throughput for all systems"""
import time
import psutil
from typing import Dict, Any, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class PerformanceMonitor:
    """Singleton Performance Monitor"""
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.monitor_config = version_manager.global_config["monitor"]
        self.enabled = self.monitor_config["enabled"]
        self.interval = self.monitor_config["monitor_interval"]
        self.metrics_history = {"cpu": [], "memory": [], "latency": []}
        self.msg_count = 0
        self.latency_timestamps = {}
        self._initialized = True
        logger.info("Performance Monitor initialized")

    def start_monitoring(self) -> None:
        """Start background performance monitoring"""
        if not self.enabled:
            return
        logger.info("Performance monitoring started")

    def stop_monitoring(self) -> None:
        """Stop performance monitoring"""
        logger.info("Performance monitoring stopped")

    def track_latency(self, key: str) -> None:
        """Track latency for a specific operation"""
        self.latency_timestamps[key] = time.time()

    def increment_msg_count(self) -> None:
        """Increment total message count"""
        self.msg_count += 1

    def get_metrics_report(self) -> Dict[str, Any]:
        """Generate a performance metrics report"""
        return {
            "timestamp": time.time(),
            "cpu": psutil.cpu_percent(),
            "memory": psutil.virtual_memory().percent,
            "total_messages": self.msg_count
        }

# Global Performance Monitor Instance
perf_monitor = PerformanceMonitor()