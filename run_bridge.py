#!/usr/bin/env python3
"""Agent ROS Bridge - Unified ROS1/ROS2 Gateway

Auto-detects available ROS version and uses appropriate connector.
For demo/testing without ROS, use demo/mock_bridge.py
"""

import asyncio
import logging
import os
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")


def detect_ros_version():
    """Detect which ROS version is available"""
    ros_distro = os.environ.get("ROS_DISTRO", "").lower()
    
    # Check environment variable first
    if ros_distro in ["noetic"]:
        return "ros1"
    elif ros_distro in ["humble", "jazzy", "iron", "galactic", "foxy"]:
        return "ros2"
    
    # Try importing ROS1
    try:
        import rospy
        return "ros1"
    except ImportError:
        pass
    
    # Try importing ROS2
    try:
        import rclpy
        return "ros2"
    except ImportError:
        pass
    
    return None


async def main():
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    
    bridge = Bridge()
    
    # Register WebSocket transport
    bridge.transport_manager.register(WebSocketTransport({'port': 8765}))
    
    # Auto-detect ROS version
    ros_version = detect_ros_version()
    
    if ros_version == "ros1":
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
        bridge.connector_registry.register(ROS1Connector({'auto_discover': True}))
        ros_display = "ROS1 Noetic"
        logger.info("✅ Using ROS1 connector")
        
    elif ros_version == "ros2":
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
        bridge.connector_registry.register(ROS2Connector({'auto_discover': True}))
        ros_display = "ROS2"
        logger.info("✅ Using ROS2 connector")
        
    else:
        logger.error("❌ No ROS detected!")
        logger.error("   Install ROS1: apt-get install python3-rospy")
        logger.error("   Install ROS2: apt-get install python3-rclpy")
        logger.error("   Or run demo: python demo/mock_bridge.py")
        sys.exit(1)
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print(f"🤖 Agent ROS Bridge Running [{ros_display}]")
    print("=" * 60)
    print("WebSocket: ws://localhost:8765")
    print("")
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
