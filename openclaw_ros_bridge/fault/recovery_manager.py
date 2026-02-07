#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Recovery Manager - Singleton coordinator for all fault recovery"""
from typing import Dict, Any
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager
from openclaw_ros_bridge.fault.recovery_strategies import (
    ROSRecoveryStrategy,
    OpenClawRecoveryStrategy,
    HALRecoveryStrategy
)

logger = get_logger(__name__)

class RecoveryManager:
    """Singleton Recovery Manager - Coordinates fault recovery across all systems"""
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.fault_config = version_manager.fault_config
        self.recovery_enabled = self.fault_config["global"]["recovery_enabled"]
        self.max_attempts = self.fault_config["global"]["max_recovery_attempts"]
        self._running = False

        # Initialize recovery strategies
        self.ros_strategy = ROSRecoveryStrategy()
        self.oc_strategy = OpenClawRecoveryStrategy()
        self.hal_strategy = HALRecoveryStrategy()

        self._initialized = True
        logger.info("Recovery Manager initialized")

    def recover(self, fault_type: str, target: str = None) -> bool:
        """Execute recovery for a specific fault type"""
        if not self.recovery_enabled:
            logger.warn("Recovery disabled in config")
            return False

        logger.info(f"Executing recovery for fault: {fault_type}")
        if fault_type == "ros_disconnect":
            return self.ros_strategy.recover()
        elif fault_type == "openclaw_disconnect":
            return self.oc_strategy.recover()
        elif fault_type == "hal_disconnect":
            return self.hal_strategy.recover()
        return False

# Singleton instance
recovery_manager = RecoveryManager()