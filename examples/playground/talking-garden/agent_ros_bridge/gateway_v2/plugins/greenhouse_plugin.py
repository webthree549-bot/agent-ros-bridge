#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Greenhouse Control Plugin for OpenClaw Gateway

Example plugin showing how to build application-specific functionality
on top of the universal gateway.
"""

import asyncio
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass

from agent_ros_bridge.gateway_v2.core import (
    Plugin, Message, Identity, Command, Telemetry, Event, Header
)
from agent_ros_bridge.gateway_v2.core import Bridge

logger = logging.getLogger("plugin.greenhouse")


@dataclass
class GreenhouseState:
    """Greenhouse state tracking"""
    temperature: float = 25.0
    humidity: float = 50.0
    light_level: float = 1000.0
    fan_on: bool = False
    valve_open: bool = False
    auto_mode: bool = False
    target_temp: float = 25.0
    target_humidity: float = 50.0


class GreenhousePlugin(Plugin):
    """Greenhouse control plugin"""
    
    name = "greenhouse"
    version = "2.0.0"
    
    def __init__(self):
        self.greenhouses: Dict[str, GreenhouseState] = {}
        self.gateway: Optional[Bridge] = None
        self._control_task: Optional[asyncio.Task] = None
    
    async def initialize(self, gateway: Bridge) -> None:
        """Initialize plugin"""
        self.gateway = gateway
        
        # Create example greenhouse
        self.greenhouses["demo"] = GreenhouseState()
        
        # Start control loop if auto mode is enabled
        self._control_task = asyncio.create_task(self._control_loop())
        
        logger.info(f"Greenhouse plugin v{self.version} initialized")
    
    async def shutdown(self) -> None:
        """Shutdown plugin"""
        if self._control_task:
            self._control_task.cancel()
            try:
                await self._control_task
            except asyncio.CancelledError:
                pass
        
        logger.info("Greenhouse plugin shutdown")
    
    async def handle_message(self, message: Message, identity: Identity) -> Optional[Message]:
        """Handle greenhouse-specific commands"""
        if not message.command:
            return None
        
        cmd = message.command
        
        # Only handle greenhouse.* commands
        if not cmd.action.startswith("greenhouse."):
            return None
        
        action = cmd.action.replace("greenhouse.", "")
        params = cmd.parameters
        greenhouse_id = params.get("id", "demo")
        
        # Ensure greenhouse exists
        if greenhouse_id not in self.greenhouses:
            return Message(
                header=Header(correlation_id=message.header.message_id),
                event=Event(
                    event_type="error",
                    severity="error",
                    data={"message": f"Greenhouse {greenhouse_id} not found"}
                )
            )
        
        state = self.greenhouses[greenhouse_id]
        
        # Handle commands
        if action == "status":
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "set_target":
            state.target_temp = params.get("temperature", state.target_temp)
            state.target_humidity = params.get("humidity", state.target_humidity)
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "fan":
            state.fan_on = params.get("on", not state.fan_on)
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "valve":
            state.valve_open = params.get("open", not state.valve_open)
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "auto":
            state.auto_mode = params.get("enabled", not state.auto_mode)
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "read_sensors":
            # In real implementation, this would read from actual sensors
            # For demo, simulate sensor readings
            state.temperature = 25.0 + (1 if state.fan_on else 3)
            state.humidity = 50.0 + (5 if state.valve_open else 0)
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "simulate":
            # Simulate environmental changes
            scenario = params.get("scenario", "normal")
            if scenario == "hot_day":
                state.temperature = 32.0
                state.humidity = 30.0
            elif scenario == "cool_morning":
                state.temperature = 18.0
                state.humidity = 65.0
            elif scenario == "rainy":
                state.temperature = 20.0
                state.humidity = 80.0
            
            return self._make_status_response(message, state, greenhouse_id)
        
        elif action == "list":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="/greenhouse/list",
                    data={
                        "greenhouses": list(self.greenhouses.keys())
                    }
                )
            )
        
        return None
    
    def _make_status_response(self, trigger_message: Message, 
                              state: GreenhouseState, 
                              greenhouse_id: str) -> Message:
        """Create status response message"""
        return Message(
            header=Header(correlation_id=trigger_message.header.message_id),
            telemetry=Telemetry(
                topic=f"/greenhouse/{greenhouse_id}/status",
                data={
                    "id": greenhouse_id,
                    "temperature": state.temperature,
                    "humidity": state.humidity,
                    "light_level": state.light_level,
                    "fan_on": state.fan_on,
                    "valve_open": state.valve_open,
                    "auto_mode": state.auto_mode,
                    "target": {
                        "temperature": state.target_temp,
                        "humidity": state.target_humidity
                    }
                }
            )
        )
    
    async def _control_loop(self) -> None:
        """Automatic control loop"""
        while True:
            try:
                for greenhouse_id, state in self.greenhouses.items():
                    if state.auto_mode:
                        # Simple bang-bang control
                        if state.temperature > state.target_temp + 2:
                            state.fan_on = True
                        elif state.temperature < state.target_temp - 1:
                            state.fan_on = False
                        
                        if state.humidity < state.target_humidity - 5:
                            state.valve_open = True
                        elif state.humidity > state.target_humidity + 2:
                            state.valve_open = False
                        
                        logger.debug(f"Auto control for {greenhouse_id}: "
                                   f"fan={state.fan_on}, valve={state.valve_open}")
                
                await asyncio.sleep(5)  # Control loop interval
                
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                await asyncio.sleep(5)


# Example usage
async def example():
    """Example greenhouse plugin usage"""
    from agent_ros_bridge.gateway_v2.core import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    
    # Create gateway
    gateway = Bridge()
    
    # Register WebSocket transport
    ws_transport = WebSocketTransport({
        "host": "0.0.0.0",
        "port": 8765
    })
    gateway.transport_manager.register(ws_transport)
    
    # Load greenhouse plugin
    greenhouse_plugin = GreenhousePlugin()
    await gateway.plugin_manager.load_plugin(greenhouse_plugin)
    
    # Start gateway
    async with gateway.run():
        logger.info("Gateway with greenhouse plugin running on ws://localhost:8765")
        logger.info("Example commands:")
        logger.info('  {"command": {"action": "greenhouse.status"}}')
        logger.info('  {"command": {"action": "greenhouse.read_sensors", "parameters": {"id": "demo"}}}')
        logger.info('  {"command": {"action": "greenhouse.fan", "parameters": {"on": true}}}')
        logger.info('  {"command": {"action": "greenhouse.auto", "parameters": {"enabled": true}}}')
        
        # Run forever
        await asyncio.Future()


if __name__ == "__main__":
    import asyncio
    asyncio.run(example())
