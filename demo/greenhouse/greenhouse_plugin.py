#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greenhouse Demo Plugin - Production-grade agricultural robotics demo"""
import time
from openclaw_ros_bridge.plugin_base.base_plugin import BasePlugin, PluginStatus

class GreenhousePlugin(BasePlugin):
    """Greenhouse Demo Plugin"""
    def __init__(self, config_path: str):
        super().__init__(plugin_name="greenhouse", config_path=config_path)
        self.current_env = {"temperature": 0.0, "humidity": 0.0}
        self.current_actuators = {"fan": False, "valve": False}

    def init_plugin(self) -> bool:
        """Plugin-specific initialization"""
        if self.status != PluginStatus.INITIALIZED:
            return False
        try:
            self.logger.info("Greenhouse plugin initialized")
            return True
        except Exception as e:
            self.logger.error(f"Greenhouse plugin init failed: {str(e)}")
            self.set_status(PluginStatus.ERROR)
            return False

    def run(self) -> None:
        """Plugin main loop"""
        if self.status != PluginStatus.INITIALIZED:
            return
        self.set_status(PluginStatus.RUNNING)
        self.logger.info("Greenhouse plugin running")
        while self.status == PluginStatus.RUNNING:
            self._read_sensor_hal()
            self._auto_control_actuators()
            time.sleep(1.0)

    def handle_ros_msg(self, ros_msg, topic_name: str) -> None:
        """ROS message handler"""
        pass

    def handle_oc_msg(self, oc_json: str, msg_type: str) -> None:
        """OpenClaw message handler"""
        pass

    def _read_sensor_hal(self) -> None:
        """Read environmental sensor data"""
        sensor_data = self.sensor_hal.read(sensor_type="env")
        self.current_env["temperature"] = sensor_data.get("temperature", 0.0)
        self.current_env["humidity"] = sensor_data.get("humidity", 0.0)

    def _auto_control_actuators(self) -> None:
        """Auto-control fan/valve based on thresholds"""
        self.current_actuators["fan"] = self.current_env["temperature"] > 28.0
        self.current_actuators["valve"] = self.current_env["humidity"] < 40.0

def main():
    """Main entry point"""
    import os
    import sys
    PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_path = os.path.join(PROJECT_ROOT, "demo/greenhouse/gh_config.yaml")
    plugin = GreenhousePlugin(config_path=config_path)
    if plugin.init_core() and plugin.init_plugin():
        plugin.run()
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()