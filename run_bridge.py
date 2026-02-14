#!/usr/bin/env python3
"""Agent ROS Bridge - Unified ROS1/ROS2 Gateway with Multi-ROS Support

Supports:
- Auto-detect local ROS
- Connect to multiple remote ROS endpoints
- Mixed ROS1/ROS2 environments
- Configuration via YAML

For demo/testing without ROS, use demo/mock_bridge.py
"""

import asyncio
import logging
import os
import sys
import yaml
from typing import List, Dict, Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")


def detect_ros_version():
    """Detect which ROS version is available locally"""
    ros_distro = os.environ.get("ROS_DISTRO", "").lower()
    
    if ros_distro in ["noetic"]:
        return "ros1"
    elif ros_distro in ["humble", "jazzy", "iron", "galactic", "foxy"]:
        return "ros2"
    
    try:
        import rospy
        return "ros1"
    except ImportError:
        pass
    
    try:
        import rclpy
        return "ros2"
    except ImportError:
        pass
    
    return None


def load_config(config_path: str = None) -> Dict[str, Any]:
    """Load configuration from YAML or use defaults"""
    default_config = {
        "bridge": {
            "name": "agent_ros_bridge",
            "transports": {
                "websocket": {"port": 8765}
            },
            "connectors": {
                "ros": {
                    "auto_detect": True,
                    "endpoints": []
                }
            }
        }
    }
    
    if config_path and os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            # Merge with defaults
            default_config["bridge"].update(config.get("bridge", {}))
    
    return default_config


async def connect_ros_endpoint(bridge, endpoint_config: Dict[str, Any]):
    """Connect to a single ROS endpoint"""
    ros_type = endpoint_config.get("ros_type", "ros2")
    endpoint_id = endpoint_config.get("id", f"ros_{ros_type}_{id(endpoint_config)}")
    host = endpoint_config.get("host", "localhost")
    
    logger.info(f"Connecting to {ros_type.upper()} endpoint: {endpoint_id} @ {host}")
    
    if ros_type == "ros1":
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
        connector = ROS1Connector({
            'endpoint_id': endpoint_id,
            'host': host,
            'port': endpoint_config.get('port'),
            'auto_discover': endpoint_config.get('auto_discover', True),
            **endpoint_config
        })
    else:  # ros2
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
        connector = ROS2Connector({
            'endpoint_id': endpoint_id,
            'host': host,
            'domain_id': endpoint_config.get('domain_id', 0),
            'auto_discover': endpoint_config.get('auto_discover', True),
            **endpoint_config
        })
    
    bridge.connector_registry.register(connector)
    return connector


async def main():
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    
    # Load config
    config_path = os.environ.get("BRIDGE_CONFIG", "config/bridge.yaml")
    config = load_config(config_path)
    bridge_config = config.get("bridge", {})
    
    bridge = Bridge()
    
    # Register WebSocket transport
    ws_config = bridge_config.get("transports", {}).get("websocket", {"port": 8765})
    bridge.transport_manager.register(WebSocketTransport(ws_config))
    
    connectors_config = bridge_config.get("connectors", {}).get("ros", {})
    endpoints = connectors_config.get("endpoints", [])
    auto_detect = connectors_config.get("auto_detect", True)
    
    connected_endpoints = []
    
    # Connect to configured remote endpoints
    for endpoint in endpoints:
        try:
            connector = await connect_ros_endpoint(bridge, endpoint)
            connected_endpoints.append({
                "id": endpoint.get("id"),
                "type": endpoint.get("ros_type", "ros2"),
                "host": endpoint.get("host", "localhost")
            })
        except Exception as e:
            logger.error(f"Failed to connect to endpoint {endpoint.get('id')}: {e}")
    
    # Auto-detect local ROS if enabled and no endpoints configured
    if auto_detect and not connected_endpoints:
        ros_version = detect_ros_version()
        
        if ros_version == "ros1":
            from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
            bridge.connector_registry.register(ROS1Connector({'auto_discover': True}))
            connected_endpoints.append({"id": "local", "type": "ros1", "host": "localhost"})
            logger.info("✅ Auto-connected to local ROS1")
            
        elif ros_version == "ros2":
            from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
            bridge.connector_registry.register(ROS2Connector({'auto_discover': True}))
            connected_endpoints.append({"id": "local", "type": "ros2", "host": "localhost"})
            logger.info("✅ Auto-connected to local ROS2")
            
        else:
            logger.error("❌ No ROS detected!")
            logger.error("   Install ROS1: apt-get install python3-rospy")
            logger.error("   Install ROS2: apt-get install python3-rclpy")
            logger.error("   Or run demo: python demo/mock_bridge.py")
            logger.error("   Or configure remote endpoints in config/bridge.yaml")
            sys.exit(1)
    
    # Start bridge
    await bridge.start()
    
    # Display status
    print("=" * 60)
    print("🤖 Agent ROS Bridge Running")
    print("=" * 60)
    print(f"WebSocket: ws://localhost:{ws_config.get('port', 8765)}")
    print("")
    print("Connected ROS Endpoints:")
    for ep in connected_endpoints:
        print(f"  • {ep['id']} ({ep['type'].upper()}) @ {ep['host']}")
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
