#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Base Plugin Class - Mandatory abstract API for all business plugins"""
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, Any, Optional
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager
from openclaw_ros_bridge.communication import get_ros_communicator, openclaw_comm
from openclaw_ros_bridge.converter import data_converter
from openclaw_ros_bridge.hal import sensor_hal, actuator_hal

class PluginStatus(Enum):
    UNINITIALIZED = "uninitialized"
    INITIALIZED = "initialized"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"
    STOPPED = "stopped"

class BasePlugin(ABC):
    """Abstract Base Plugin - Core parent class for all business plugins"""
    def __init__(self, plugin_name: str, config_path: Optional[str] = None):
        self.plugin_name = plugin_name
        self.config_path = config_path
        self.mock_mode = version_manager.MOCK_MODE
        self.ros_type = version_manager.ROS_TYPE
        self.oc_version = version_manager.OC_VER

        self.ros_comm = get_ros_communicator()
        self.oc_comm = openclaw_comm
        self.converter = data_converter
        self.sensor_hal = sensor_hal
        self.actuator_hal = actuator_hal

        self.status = PluginStatus.UNINITIALIZED
        self.plugin_config: Dict[str, Any] = {}
        self.logger = get_logger(f"plugin_{self.plugin_name}")
        self.logger.info(f"Base Plugin initialized - Name: {self.plugin_name}")

    def init_core(self) -> bool:
        """Initialize core framework dependencies"""
        if self.status != PluginStatus.UNINITIALIZED:
            return True
        try:
            if not self.ros_comm.is_initialized:
                self.ros_comm._init_node()
            if not self.oc_comm.is_connected:
                self.oc_comm.connect()
            self.sensor_hal.init_hardware()
            self.actuator_hal.init_hardware()
            if self.config_path:
                self._load_plugin_config()
            self.set_status(PluginStatus.INITIALIZED)
            return True
        except Exception as e:
            self.logger.error(f"Core init failed: {str(e)}")
            self.set_status(PluginStatus.ERROR)
            return False

    def _load_plugin_config(self) -> None:
        """Load plugin-specific YAML config"""
        from openclaw_ros_bridge.base.config_loader import config_loader
        self.plugin_config = config_loader.load_yaml(self.config_path)

    def set_status(self, new_status: PluginStatus) -> None:
        """Update plugin status"""
        self.status = new_status
        self.logger.info(f"Plugin status changed to: {new_status.value}")

    def get_status(self) -> str:
        """Get current plugin status"""
        return self.status.value

    def graceful_stop(self) -> None:
        """Graceful plugin shutdown"""
        self.sensor_hal.safe_state()
        self.actuator_hal.safe_state()
        self.set_status(PluginStatus.STOPPED)
        self.logger.info("Plugin gracefully stopped")

    @abstractmethod
    def init_plugin(self) -> bool:
        """Plugin-specific initialization"""
        pass

    @abstractmethod
    def run(self) -> None:
        """Plugin main loop"""
        pass

    @abstractmethod
    def handle_ros_msg(self, ros_msg: Any, topic_name: str) -> None:
        """ROS message handler"""
        pass

    @abstractmethod
    def handle_oc_msg(self, oc_json: str, msg_type: str) -> None:
        """OpenClaw message handler"""
        pass