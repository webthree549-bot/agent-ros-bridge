#!/usr/bin/env python3
"""Agent ROS Bridge - Production WebSocket to ROS2 Gateway

Requires ROS2 to be installed and running.
For testing without ROS2, use demo/mock_bridge.py
"""

import asyncio
import logging

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")


async def main():
    bridge = Bridge()
    
    # Register WebSocket transport
    bridge.transport_manager.register(WebSocketTransport({'port': 8765}))
    
    # Register ROS2 connector
    bridge.connector_registry.register(ROS2Connector({'auto_discover': True}))
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print("🤖 Agent ROS Bridge Running")
    print("=" * 60)
    print("WebSocket: ws://localhost:8765")
    print("")
    print("Requires ROS2 to be installed and running.")
    print("Test commands:")
    print('  {"command": {"action": "list_robots"}}')
    print('  {"command": {"action": "get_topics"}}')
    print('  {"command": {"action": "publish", "parameters": {"topic": "/cmd_vel", "data": {"linear": {"x": 0.5}}}}}}')
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print('\n⏹️  Stopping bridge...')
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
