#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Version Manager - Singleton core for ROS1/ROS2/OpenClaw auto-detection & config"""
import os
import yaml
from typing import Dict, Optional, Literal, Any
from openclaw_ros_bridge.base.config_loader import config_loader
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.base.utils import convert_to_bool, validate_path

# Type Hints
ROS_Type = Literal["ros1", "ros2"]
ROS_Distro = Literal["noetic", "humble", "jazzy"]
OpenClaw_Ver = Literal["v1", "v2"]
HAL_Hardware = Literal["auto", "dht22", "bme280", "robotiq_2f_85", "l298n"]

logger = get_logger(__name__)

class VersionManager:
    """Singleton Version Manager - ONLY entry point for version-specific configs"""
    _instance: Optional["VersionManager"] = None
    _initialized: bool = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        # Project Root & Config Paths
        self.ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        self.CONFIG_DIR = os.path.join(self.ROOT_DIR, "config")
        validate_path(self.CONFIG_DIR, create=True)

        # Load All Core Configs (centralized YAML)
        self._load_core_configs()

        # Env > Auto-Detect > Default Version Resolution
        self.OC_VER: OpenClaw_Ver = self._resolve_openclaw_version()
        self.ROS_DISTRO: ROS_Distro = self._resolve_ros_distro()
        self.ROS_TYPE: ROS_Type = self._resolve_ros_type()
        self.HAL_HARDWARE: HAL_Hardware = self._resolve_hal_hardware()
        self.MOCK_MODE = convert_to_bool(os.getenv("MOCK_MODE", "false"))
        self.MIXED_DEPLOYMENT = convert_to_bool(os.getenv("MIXED_DEPLOYMENT", "false"))

        # Validate Versions
        self._validate_versions()

        # Load Version-Specific Configs
        self.ros_config = self._get_ros_config()
        self.oc_config = self.openclaw_config["openclaw_versions"][self.OC_VER]
        self.hal_sensor_config = self._get_hal_config("sensors")
        self.hal_actuator_config = self._get_hal_config("actuators")

        # Initialize
        self._initialized = True

        # Initialization Log
        logger.info(f"Version Manager Initialized - ROS: {self.ROS_TYPE.upper()} ({self.ROS_DISTRO}) | OpenClaw: {self.OC_VER} | HAL: {self.HAL_HARDWARE} | Mock Mode: {self.MOCK_MODE}")
        if self.MIXED_DEPLOYMENT:
            logger.info(f"Mixed ROS1/ROS2 Deployment Enabled")

    def _load_core_configs(self) -> None:
        """Load all core YAML configs from the config directory"""
        self.global_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "global_config.yaml"))
        self.ros1_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "ros1_config.yaml"))
        self.ros2_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "ros2_config.yaml"))
        self.openclaw_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "openclaw_config.yaml"))
        self.hal_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "hal_config.yaml"))
        self.fault_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "fault_config.yaml"))
        self.debug_config = config_loader.load_yaml(os.path.join(self.CONFIG_DIR, "debug_config.yaml"))

    def _resolve_ros_distro(self) -> ROS_Distro:
        """Resolve ROS distro (ENV > ROS_DISTRO env > default: humble)"""
        env_distro = os.getenv("ROS_DISTRO", "").lower()
        manual_distro = os.getenv("ROS_DISTRO_MANUAL", "").lower()
        distro = manual_distro or env_distro or "humble"
        return distro if distro in ["noetic", "humble", "jazzy"] else "humble"

    def _resolve_ros_type(self) -> ROS_Type:
        """Resolve ROS type (ENV > distro mapping > default: ros2)"""
        manual_type = os.getenv("ROS_TYPE", "").lower()
        if manual_type in ["ros1", "ros2"]:
            return manual_type
        return "ros1" if self.ROS_DISTRO == "noetic" else "ros2"

    def _resolve_openclaw_version(self) -> OpenClaw_Ver:
        """Resolve OpenClaw version (ENV > default from config)"""
        env_oc = os.getenv("OPENCLAW_VERSION", "").lower()
        default_oc = self.openclaw_config.get("default_version", "v2")
        return env_oc if env_oc in ["v1", "v2"] else default_oc

    def _resolve_hal_hardware(self) -> HAL_Hardware:
        """Resolve HAL hardware (ENV > default: auto)"""
        env_hal = os.getenv("HAL_HARDWARE", "").lower()
        supported_hal = ["auto", "dht22", "bme280", "robotiq_2f_85", "l298n"]
        return env_hal if env_hal in supported_hal else "auto"

    def _validate_versions(self) -> None:
        """Validate all resolved versions are supported"""
        ros1_supported = ["noetic"]
        ros2_supported = ["humble", "jazzy"]
        if self.ROS_TYPE == "ros1" and self.ROS_DISTRO not in ros1_supported:
            raise ValueError(f"ROS1 only supports {ros1_supported} - got {self.ROS_DISTRO}")
        if self.ROS_TYPE == "ros2" and self.ROS_DISTRO not in ros2_supported:
            raise ValueError(f"ROS2 only supports {ros2_supported} - got {self.ROS_DISTRO}")
        logger.debug("All versions validated successfully")

    def _get_ros_config(self) -> Dict[str, Any]:
        """Get ROS1/ROS2 distro-specific config"""
        if self.ROS_TYPE == "ros1":
            return self.ros1_config["ros1_versions"][self.ROS_DISTRO]
        return self.ros2_config["ros2_versions"][self.ROS_DISTRO]

    def _get_hal_config(self, hal_type: Literal["sensors", "actuators"]) -> Dict[str, Any]:
        """Get HAL sensor/actuator config (auto-detect if HAL_HARDWARE=auto)"""
        if self.MOCK_MODE:
            logger.info("Mock mode enabled - returning empty HAL config")
            return {}
        hal_config = self.hal_config[hal_type]
        if self.HAL_HARDWARE == "auto":
            return {k: v for k, v in hal_config.items() if k != "supported_models"}
        for group, config in hal_config.items():
            if self.HAL_HARDWARE in config.get("supported_models", []):
                return config[self.HAL_HARDWARE]
        logger.warn(f"HAL hardware {self.HAL_HARDWARE} not found - using default")
        return hal_config[list(hal_config.keys())[0]][hal_config[list(hal_config.keys())[0]]["default_model"]]

    def get_ros_param(self, param_name: str, default: Any = None) -> Any:
        """Get ROS1/ROS2 param (distro-specific > global > default)"""
        if param_name in self.ros_config:
            return self.ros_config[param_name]
        global_config = self.ros1_config["global"] if self.ROS_TYPE == "ros1" else self.ros2_config["global"]
        return global_config.get(param_name, default)

    def get_oc_param(self, param_name: str, default: Any = None) -> Any:
        """Get OpenClaw param (version-specific > global > default)"""
        if param_name in self.oc_config:
            return self.oc_config[param_name]
        return self.openclaw_config["global"].get(param_name, default)

    def get_hal_param(self, param_name: str, hal_type: Literal["sensors", "actuators"], default: Any = None) -> Any:
        """Get HAL param (sensor/actuator > global > default)"""
        hal_config = self.hal_sensor_config if hal_type == "sensors" else self.hal_actuator_config
        if param_name in hal_config:
            return hal_config[param_name]
        return self.hal_config["global"].get(param_name, default)

    def get_ros_env_path(self) -> str:
        """Get ROS1/ROS2 environment setup script path"""
        return self.get_ros_param("env_path", "/opt/ros/humble/setup.bash")

    def get_oc_tcp_config(self) -> Dict[str, Any]:
        """Get unified OpenClaw TCP config for communication"""
        return {
            "host": self.get_oc_param("tcp_host", "127.0.0.1"),
            "port": self.get_oc_param("tcp_port", 9999),
            "buffer_size": self.get_oc_param("recv_buffer_size", 8192),
            "send_timeout": self.get_oc_param("send_timeout", 10),
            "recv_timeout": self.get_oc_param("recv_timeout", 10),
            "heartbeat_interval": self.get_oc_param("heartbeat_interval", 3),
            "reconnect_attempts": self.get_oc_param("reconnect_attempts", 15)
        }

    def get_ros_build_config(self) -> Dict[str, Any]:
        """Get ROS1/ROS2 build system config (Catkin/Colcon)"""
        if self.ROS_TYPE == "ros1":
            return {
                "build_cmd": self.get_ros_param("build_cmd", "catkin_make"),
                "ws_dir": self.get_ros_param("catkin_ws", "catkin_ws"),
                "src_dir": self.get_ros_param("catkin_src", "catkin_ws/src")
            }
        return {
            "build_cmd": "colcon build --symlink-install",
            "ws_dir": f"build_{self.ROS_DISTRO}",
            "install_dir": f"install_{self.ROS_DISTRO}",
            "log_dir": f"log_{self.ROS_DISTRO}"
        }

# Singleton Instance (used across the entire framework)
version_manager = VersionManager()

def main() -> None:
    """Main entry point - Version Manager CLI (verify versions/config)"""
    from openclaw_ros_bridge.base.utils import get_system_info
    sys_info = get_system_info()
    logger.info("=== OpenClaw-ROS Bridge Version Manager ===")
    for key, value in sys_info.items():
        logger.info(f"{key.upper()}: {value}")
    logger.info("============================================")

if __name__ == "__main__":
    main()