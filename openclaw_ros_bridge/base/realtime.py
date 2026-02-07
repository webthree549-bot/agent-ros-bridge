#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real-Time Scheduler - Linux SCHED_FIFO priority & CPU affinity for ROS nodes"""
import os
import psutil
import ctypes
import ctypes.util
import platform
from typing import List, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.base.utils import convert_to_bool, get_cpu_cores

logger = get_logger(__name__)

# Linux kernel constants (for SCHED_FIFO)
SCHED_FIFO = 1
libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)

class RealTimeScheduler:
    """Real-Time Scheduler for Linux - Set SCHED_FIFO priority and CPU affinity"""
    def __init__(self):
        self.pid = os.getpid()
        self.process = psutil.Process(self.pid)
        self.supported = platform.system() == "Linux"
        if not self.supported:
            logger.warn("Real-time scheduling only supported on Linux - disabling")

    def set_realtime_priority(self, priority: int = 90) -> bool:
        """
        Set process to Linux SCHED_FIFO real-time priority (requires root/sudo)
        
        Args:
            priority: SCHED_FIFO priority (0-99, higher = more priority)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.supported:
            return False
        if not (0 <= priority <= 99):
            logger.error(f"Invalid real-time priority: {priority} (must be 0-99)")
            return False

        # Set scheduling policy
        class SchedParam(ctypes.Structure):
            _fields_ = [("sched_priority", ctypes.c_int)]

        param = SchedParam(priority)
        result = libc.sched_setscheduler(
            self.pid,
            SCHED_FIFO,
            ctypes.pointer(param)
        )

        if result == 0:
            logger.info(f"Set real-time SCHED_FIFO priority: {priority} (PID: {self.pid})")
            return True
        else:
            errno = ctypes.get_errno()
            logger.error(
                f"Failed to set real-time priority (errno: {errno}) - run with sudo/root"
            )
            return False

    def set_cpu_affinity(self, cores: Optional[List[int]] = None) -> bool:
        """
        Set CPU affinity for the process (bind to specific cores)
        
        Args:
            cores: List of CPU core IDs to bind to (0-indexed)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.supported:
            return False
        cores = cores or get_cpu_cores()
        try:
            self.process.cpu_affinity(cores)
            logger.info(f"Set CPU affinity to cores: {cores} (PID: {self.pid})")
            return True
        except Exception as e:
            logger.error(f"Failed to set CPU affinity: {str(e)}")
            return False

    def configure(self, config: dict) -> bool:
        """
        Configure real-time scheduling from a config dict (version_manager)
        
        Args:
            config: Real-time config dict (enabled, priority, cpu_affinity)
        
        Returns:
            True if all configs applied successfully, False otherwise
        """
        if not self.supported:
            return False
        if not convert_to_bool(config.get("enabled", False)):
            logger.info("Real-time scheduling disabled in config")
            return True

        success = True
        # Set CPU affinity first
        cores = config.get("cpu_affinity", get_cpu_cores())
        success &= self.set_cpu_affinity(cores)
        # Set real-time priority
        priority = config.get("priority", 90)
        success &= self.set_realtime_priority(priority)
        return success

# Singleton instance
rt_scheduler = RealTimeScheduler()