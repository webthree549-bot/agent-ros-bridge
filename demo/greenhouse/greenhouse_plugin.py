#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greenhouse Demo Plugin - Application-specific handlers for greenhouse control

This is a DEMO application showing how to build on top of the generic
OpenClaw ROS Bridge. It implements greenhouse-specific commands like:
- read_sensor: Read temperature/humidity
- control_fan: Turn fan on/off
- control_valve: Open/close water valve

To use: Load this plugin with the TCP server to enable greenhouse commands.
"""
from typing import Dict, Any
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.hal import sensor_hal, actuator_hal

logger = get_logger(__name__)


class GreenhousePlugin:
    """Greenhouse Demo Plugin - Registers greenhouse-specific commands"""
    
    def __init__(self):
        self.name = "greenhouse"
        self.version = "1.0.0"
        self.description = "Demo greenhouse control plugin"
    
    def register(self, server) -> None:
        """Register command handlers with the TCP server
        
        Args:
            server: The OpenClawTCPServer instance
        """
        server.register_handler("read_sensor", self.handle_read_sensor)
        server.register_handler("write_actuator", self.handle_write_actuator)
        server.register_handler("get_greenhouse_status", self.handle_greenhouse_status)
        
        # Initialize HAL
        sensor_hal.init_hardware()
        actuator_hal.init_hardware()
        
        logger.info(f"Greenhouse plugin v{self.version} registered")
    
    def handle_read_sensor(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Handle read_sensor command
        
        Args:
            cmd: Command dict with 'sensor' key (e.g., 'env' for environment)
        
        Returns:
            Response dict with sensor data
        """
        sensor_type = cmd.get('sensor', 'env')
        
        try:
            data = sensor_hal.read(sensor_type)
            return {'status': 'ok', 'data': data}
        except Exception as e:
            logger.error(f"Sensor read failed: {e}")
            return {'status': 'error', 'message': f'Sensor read failed: {str(e)}'}
    
    def handle_write_actuator(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Handle write_actuator command
        
        Args:
            cmd: Command dict with 'actuator' and 'value' keys
        
        Returns:
            Response dict confirming the action
        """
        actuator = cmd.get('actuator')
        value = cmd.get('value')
        
        if not actuator:
            return {'status': 'error', 'message': 'Missing actuator parameter'}
        
        try:
            actuator_hal.write({actuator: value})
            return {
                'status': 'ok', 
                'message': f'{actuator} set to {value}',
                'actuator': actuator,
                'value': value
            }
        except Exception as e:
            logger.error(f"Actuator write failed: {e}")
            return {'status': 'error', 'message': f'Actuator write failed: {str(e)}'}
    
    def handle_greenhouse_status(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Get greenhouse-specific status
        
        Returns:
            Response dict with greenhouse system status
        """
        try:
            sensor_data = sensor_hal.read("env")
            actuator_data = actuator_hal.read()
            
            return {
                'status': 'ok',
                'plugin': self.name,
                'version': self.version,
                'sensors': sensor_data,
                'actuators': actuator_data
            }
        except Exception as e:
            logger.error(f"Status read failed: {e}")
            return {'status': 'error', 'message': f'Status read failed: {str(e)}'}


# Global plugin instance
greenhouse_plugin = GreenhousePlugin()


def register_with_server(server) -> None:
    """Convenience function to register plugin with server
    
    Usage:
        from openclaw_ros_bridge.communication.openclaw_tcp_server import openclaw_server
        from demo.greenhouse.greenhouse_plugin import register_with_server
        
        register_with_server(openclaw_server)
        openclaw_server.start()
    """
    greenhouse_plugin.register(server)
