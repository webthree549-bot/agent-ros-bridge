#!/usr/bin/env python3
"""Dual ROS Bridge - Run both ROS1 and ROS2 simultaneously

This script sets up a bridge that connects to both ROS1 and ROS2
environments simultaneously, allowing control of mixed robot fleets.

Requirements:
- Ubuntu 20.04 or 22.04
- ROS1 Noetic installed
- ROS2 Humble (or Foxy) installed
- ros1_bridge (for ROS1-ROS2 communication)

Usage:
    # Terminal 1: Start ROS1 roscore
    source /opt/ros/noetic/setup.bash
    roscore
    
    # Terminal 2: Start ROS1-ROS2 bridge
    source /opt/ros/noetic/setup.bash
    source /opt/ros/humble/setup.bash
    ros2 run ros1_bridge dynamic_bridge
    
    # Terminal 3: Start Agent ROS Bridge with dual config
    python run_bridge_dual_ros.py
"""

import asyncio
import logging
import os
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dual_ros_bridge")


async def main():
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
    
    bridge = Bridge()
    
    # Register WebSocket transport
    bridge.transport_manager.register(WebSocketTransport({'port': 8765}))
    
    # Connect to ROS1
    logger.info("Connecting to ROS1...")
    try:
        ros1_connector = ROS1Connector({
            'endpoint_id': 'ros1_local',
            'host': 'localhost',
            'auto_discover': True
        })
        bridge.connector_registry.register(ros1_connector)
        logger.info("✅ ROS1 connected")
        ros1_connected = True
    except Exception as e:
        logger.error(f"❌ ROS1 connection failed: {e}")
        ros1_connected = False
    
    # Connect to ROS2
    logger.info("Connecting to ROS2...")
    try:
        ros2_connector = ROS2Connector({
            'endpoint_id': 'ros2_local',
            'host': 'localhost',
            'domain_id': int(os.environ.get('ROS_DOMAIN_ID', '0')),
            'auto_discover': True
        })
        bridge.connector_registry.register(ros2_connector)
        logger.info("✅ ROS2 connected")
        ros2_connected = True
    except Exception as e:
        logger.error(f"❌ ROS2 connection failed: {e}")
        ros2_connected = False
    
    if not ros1_connected and not ros2_connected:
        logger.error("❌ No ROS connections available!")
        sys.exit(1)
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print("🤖 DUAL ROS BRIDGE (ROS1 + ROS2)")
    print("=" * 60)
    print("WebSocket: ws://localhost:8765")
    print("")
    print("Connected:")
    if ros1_connected:
        print("  ✅ ROS1 Noetic")
    if ros2_connected:
        print(f"  ✅ ROS2 (Domain ID: {os.environ.get('ROS_DOMAIN_ID', '0')})")
    print("")
    print("Both environments accessible via single WebSocket endpoint!")
    print("")
    print("Query specific ROS version:")
    print('  {"command": {"action": "list_robots", "parameters": {"ros_version": "ros1"}}}')
    print('  {"command": {"action": "list_robots", "parameters": {"ros_version": "ros2"}}}')
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print('\n⏹️  Stopping dual bridge...')
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
