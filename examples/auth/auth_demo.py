#!/usr/bin/env python3
"""Simulated Robot Bridge with Authentication Demo

This demonstrates JWT authentication on the simulated bridge.
"""

import asyncio
import logging
from datetime import datetime

from agent_ros_bridge import Bridge, Message, Header, Command, Telemetry, Event, Identity, Plugin
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("simulated_bridge")


class SimulatedRobotPlugin(Plugin):
    """Plugin that simulates a robot for testing"""
    name = "simulated_robot"
    version = "1.0.0"
    
    def __init__(self):
        self.topics = [
            "/cmd_vel", "/odom", "/scan", "/joint_states", "/tf", "/battery_state"
        ]
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.battery = 85.0
        self.speed = 0.0
    
    async def initialize(self, gateway) -> None:
        logger.info("Simulated robot plugin initialized")
    
    async def shutdown(self) -> None:
        logger.info("Simulated robot plugin shutdown")
    
    async def handle_message(self, message: Message, identity: Identity) -> Message:
        if not message.command:
            return None
        
        cmd = message.command
        logger.info(f"📩 Command from {identity.name} (roles: {identity.roles}): {cmd.action}")
        
        if cmd.action == "list_robots":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="robots",
                    data={"robots": [{"id": "turtlebot_01", "name": "TurtleBot4 Simulated", 
                                      "type": "ros2_simulated", "connected": True, "battery": self.battery}]}
                )
            )
        
        elif cmd.action == "get_topics":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="topics",
                    data={"topics": self.topics}
                )
            )
        
        elif cmd.action == "move":
            direction = cmd.parameters.get("direction", "forward")
            distance = cmd.parameters.get("distance", 0.0)
            logger.info(f"🤖 Move {direction} {distance}m")
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="result",
                    data={"moving": True, "direction": direction, "distance": distance}
                )
            )
        
        return Message(
            header=Header(correlation_id=message.header.message_id),
            event=Event(
                event_type="unknown_command",
                severity="warning",
                data={"action": cmd.action}
            )
        )


async def main():
    bridge = Bridge()
    
    # Create WebSocket transport with auth enabled
    ws_transport = WebSocketTransport({
        'port': 8768,  # Different port to avoid conflicts
        'auth': {
            'enabled': True,
            'jwt_secret': '8GpxD7Zp6L6bGl_bHbzLmNNaEpyKE3Sh1xEzO-dJfAo',
            'api_keys': {
                'ak_test123': {
                    'user_id': 'test_api_user',
                    'roles': ['operator']
                }
            }
        }
    })
    bridge.transport_manager.register(ws_transport)
    
    # Register simulated robot plugin
    simulated_plugin = SimulatedRobotPlugin()
    await bridge.plugin_manager.load_plugin(simulated_plugin)
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print("🎭 SIMULATED ROBOT BRIDGE (WITH AUTH)")
    print("=" * 60)
    print("WebSocket: ws://localhost:8768")
    print("")
    print("⚠️  AUTHENTICATION REQUIRED")
    print("")
    print("Connect with token:")
    print('  wscat -c "ws://localhost:8768?token=eyJhbGciOiJIUzI1NiIs..."')
    print("")
    print("Or generate a new token:")
    print('  python scripts/generate_token.py --secret "8GpxD7Zp6L6bGl_bHbzLmNNaEpyKE3Sh1xEzO-dJfAo"')
    print("")
    print("Test API key:")
    print('  Header: X-API-Key: ak_test123')
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print('\n⏹️  Stopping...')
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
