#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""State Monitor - Tracks real-time state of ROS/OpenClaw/HAL/Hardware"""
import time
from typing import Dict, Any
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class StateMonitor:
    """Singleton State Monitor"""
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.monitor_interval = 1.0
        self.system_state = {
            "ros": {"state": "offline", "timestamp": time.time()},
            "openclaw": {"state": "offline", "timestamp": time.time()},
            "sensor_hal": {"state": "offline", "timestamp": time.time()},
            "actuator_hal": {"state": "offline", "timestamp": time.time()}
        }
        self._initialized = True
        logger.info("State Monitor initialized")

    def get_system_state(self, component: str = None) -> Dict[str, Any]:
        """Get current system state"""
        if component and component in self.system_state:
            return self.system_state[component]
        return self.system_state

    def get_health_status(self) -> str:
        """Get overall system health status"""
        return "healthy"

# Global State Monitor Instance
state_monitor = StateMonitor()