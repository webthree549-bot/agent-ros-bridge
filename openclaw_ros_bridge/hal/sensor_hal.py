#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sensor HAL - Unified interface for all sensors (temp/humidity/camera)"""
import time
import random
from typing import Dict, Any, Optional
from openclaw_ros_bridge.hal.base_hal import BaseHAL

class SensorHAL(BaseHAL):
    """Sensor HAL - Implements BaseHAL for temperature/humidity/camera sensors"""
    def __init__(self):
        super().__init__(hal_type="sensor")
        self.sample_freq = 1.0
        self.last_read = 0.0
        self.valid_sensors = ["dht22", "bme280", "ds18b20", "usb_cam", "raspi_cam", "jetson_cam"]

    def init_hardware(self) -> bool:
        """Initialize sensor hardware (auto-detect from config)"""
        if self.mock_mode:
            self.initialized = True
            return True
        self.initialized = True
        return True

    def read(self, sensor_type: Optional[str] = "env", **kwargs) -> Dict[str, Any]:
        """Read data from sensor (rate-limited by sample frequency)"""
        if not self.initialized:
            return self._empty_read()
        # Rate limiting
        if time.time() - self.last_read < 1.0 / self.sample_freq:
            return self._empty_read()
        # Mock mode read
        if self.mock_mode:
            return self._mock_read(sensor_type)
        # Physical sensor read (stub)
        self.last_read = time.time()
        return self._mock_read(sensor_type)

    def _mock_read(self, sensor_type: str) -> Dict[str, Any]:
        """Generate mock sensor data (no physical hardware)"""
        base = {"timestamp": time.time(), "model": "mock_hardware", "mock_mode": True}
        if sensor_type == "env":
            return {**base, "temperature": 25.0, "humidity": 50.0}
        return {**base, "resolution": "640x480", "fps": 30, "frame_count": 0}

    def _empty_read(self) -> Dict[str, Any]:
        """Return empty sensor data (error/rate-limited)"""
        return {"timestamp": time.time(), "temperature": 0.0, "humidity": 0.0, "error": True}

    def write(self, data: Dict[str, Any], **kwargs) -> bool:
        """Write config to sensor (e.g., sample frequency/resolution)"""
        if not self.initialized:
            return False
        if self.mock_mode:
            return True
        try:
            if "sample_freq" in data:
                self.sample_freq = float(data["sample_freq"])
            return True
        except Exception:
            return False

    def safe_state(self) -> bool:
        """Set sensor to safe state (stop sampling/streaming)"""
        if not self.initialized:
            return False
        if self.mock_mode:
            return True
        self.sample_freq = 0.1
        return True

# Global Sensor HAL Instance
sensor_hal = SensorHAL()
