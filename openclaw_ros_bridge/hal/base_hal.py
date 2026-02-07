#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Base HAL Class - Abstract parent for all HAL modules (sensors/actuators)"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

class BaseHAL(ABC):
    """Abstract Base HAL Class - Defines mandatory API for all HAL modules"""
    def __init__(self, hal_type: str):
        self.hal_type = hal_type  # sensor/actuator
        self.mock_mode = False
        self.hal_config = {}
        self.hardware_model = self._get_hardware_model()
        self.initialized = False

    def _get_hardware_model(self) -> str:
        """Get detected/configured hardware model from VersionManager"""
        if self.mock_mode:
            return "mock_hardware"
        return "auto"

    @abstractmethod
    def init_hardware(self) -> bool:
        """Initialize hardware (mandatory)"""
        pass

    @abstractmethod
    def read(self, **kwargs) -> Dict[str, Any]:
        """Read data from hardware (sensors) / Read state (actuators) (mandatory)"""
        pass

    @abstractmethod
    def write(self, data: Dict[str, Any], **kwargs) -> bool:
        """Write data/commands to hardware (actuators) (mandatory for actuators)"""
        pass

    @abstractmethod
    def safe_state(self) -> bool:
        """Set hardware to safe state (mandatory)"""
        pass

    def destroy(self) -> None:
        """Cleanup hardware resources (optional - override if needed)"""
        self.initialized = False
