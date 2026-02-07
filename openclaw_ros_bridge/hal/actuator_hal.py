#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Actuator HAL - Unified interface for all actuators (manipulator/motor/relay)"""
import time
from typing import Dict, Any, Optional
from openclaw_ros_bridge.hal.base_hal import BaseHAL

class ActuatorHAL(BaseHAL):
    """Actuator HAL - Implements BaseHAL for manipulators/motors/relays"""
    def __init__(self):
        super().__init__(hal_type="actuator")
        self.max_speed = 255
        self.max_force = 100
        self.current_state = {"state": "idle", "value": 0, "timestamp": time.time()}
        self.valid_actuators = ["robotiq_2f_85", "dynamixel_xl430", "l298n", "tb6612", "sr501"]

    def init_hardware(self) -> bool:
        """Initialize actuator hardware (auto-detect from config)"""
        if self.mock_mode:
            self.initialized = True
            return True
        self.initialized = True
        self.safe_state()
        return True

    def read(self, **kwargs) -> Dict[str, Any]:
        """Read current actuator state (position/speed/force)"""
        if not self.initialized:
            return {"timestamp": time.time(), "state": "error", "value": 0}
        if self.mock_mode:
            self.current_state["timestamp"] = time.time()
            return self.current_state
        self.current_state["timestamp"] = time.time()
        return self.current_state

    def write(self, data: Dict[str, Any], **kwargs) -> bool:
        """Write command to actuator (position/speed/force/state)"""
        if not self.initialized:
            return False
        if self.mock_mode:
            self.current_state = {**self.current_state, **data, "timestamp": time.time()}
            return True
        if "state" not in data or "value" not in data:
            return False
        data["value"] = self._enforce_limits(data["value"])
        self.current_state = {**self.current_state, **data, "timestamp": time.time()}
        return True

    def _enforce_limits(self, value):
        """Enforce hardware speed/force limits (clamp value)"""
        if isinstance(value, dict):
            return value
        if self.hardware_model in ["robotiq_2f_85", "dynamixel_xl430"]:
            return max(0, min(value, self.max_force))
        return max(0, min(value, self.max_speed))

    def safe_state(self) -> bool:
        """Set actuator to safe state (idle/stop/0 position)"""
        if not self.initialized:
            return False
        safe_data = {"state": "idle", "value": 0}
        self.write(safe_data)
        return True

# Global Actuator HAL Instance
actuator_hal = ActuatorHAL()
