#!/usr/bin/env python3
"""ROS2 Real Bridge Example

This example demonstrates actual ROS2 connectivity:
1. Connect to ROS2 (requires rclpy and a running ROS2 system)
2. Discover available topics
3. Subscribe to a topic
4. Publish commands

Usage:
    # Terminal 1: Start ROS2 system (or turtlesim for testing)
    ros2 run turtlesim turtlesim_node

    # Terminal 2: Run this bridge
    python examples/ros2_real_bridge.py

    # Terminal 3: Test with ros2 CLI
    ros2 topic echo /turtle1/cmd_vel
    ros2 topic pub /turtle1/pose geometry_msgs/Pose '{position: {x: 5.0, y: 5.0}}'
"""

import asyncio
import json
import os
import sys

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.core import Command
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport


async def main():
    """Run ROS2 real bridge example."""
    print("=" * 60)
    print("🤖 ROS2 Real Bridge Example")
    print("=" * 60)

    # Set JWT secret
    os.environ["JWT_SECRET"] = "ros2-bridge-secret-key"

    # Create bridge
    bridge = Bridge()

    # Add WebSocket transport for AI agents to connect
    ws_transport = WebSocketTransport({"port": 8766, "host": "localhost"})
    bridge.transport_manager.register(ws_transport)

    try:
        # Start bridge
        await bridge.start()
        print("\n✅ Bridge started on ws://localhost:8766")

        # Import and use ROS2 connector
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import (
            ROS2_AVAILABLE,
            ROS2Connector,
        )

        if not ROS2_AVAILABLE:
            print("\n❌ ROS2 not available. Install rclpy:")
            print("   sudo apt install ros-${ROS_DISTRO}-rclpy")
            print("   pip install rclpy (if not using apt)")
            return

        # Create ROS2 connector
        ros2 = ROS2Connector(domain_id=0)
        bridge.connector_registry.register(ros2)
        print("✅ ROS2 connector registered")

        # Discover ROS2 systems
        print("\n🔍 Discovering ROS2 systems...")
        endpoints = await bridge.connector_registry.discover_all()
        print(f"   Found {len(endpoints)} ROS2 endpoint(s)")
        for ep in endpoints:
            print(f"   - {ep.name}: {ep.uri}")
            print(f"     Capabilities: {ep.capabilities}")
            print(f"     Metadata: {ep.metadata}")

        # Connect to ROS2
        print("\n🔗 Connecting to ROS2 domain 0...")
        robot = await ros2.connect("ros2://0/")
        print(f"✅ Connected to robot: {robot.name}")

        # Get available topics
        print("\n📡 Available ROS2 Topics:")
        topics = robot._cmd_get_topics()
        for topic in topics[:10]:  # Show first 10
            print(f"   - {topic['name']}: {topic['types']}")
        if len(topics) > 10:
            print(f"   ... and {len(topics) - 10} more")

        # Get nodes
        print("\n🔧 ROS2 Nodes:")
        nodes = robot._cmd_get_nodes()
        for node in nodes[:5]:
            print(f"   - {node}")
        if len(nodes) > 5:
            print(f"   ... and {len(nodes) - 5} more")

        # Test tool discovery
        print("\n🔍 Testing Tool Discovery...")
        discovery = bridge.get_tool_discovery()
        tools = discovery.discover_all()
        print(f"   Discovered {len(tools)} tools:")
        for tool in tools[:5]:
            danger = "⚠️" if tool.dangerous else "  "
            print(f"   {danger} {tool.name} ({tool.action_type}): {tool.ros_type}")

        # Test publishing (if turtlesim is running)
        print("\n📝 Testing Publish (if turtlesim is running)...")
        print("   Sending Twist command to /turtle1/cmd_vel")

        result = await robot.execute(
            Command(
                action="publish",
                parameters={
                    "topic": "/turtle1/cmd_vel",
                    "type": "geometry_msgs/Twist",
                    "data": {
                        "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.5},
                    },
                },
            )
        )
        print(f"   Result: {result}")

        # Test subscribing
        print("\n👂 Testing Subscribe (if /turtle1/pose exists)...")
        print("   Subscribing to /turtle1/pose for 5 seconds...")

        # Check if topic exists
        topic_exists = any(t["name"] == "/turtle1/pose" for t in topics)
        if topic_exists:
            count = 0
            async for telemetry in robot.subscribe("/turtle1/pose"):
                if count == 0:
                    print(f"   Received: {json.dumps(telemetry.data, indent=2)[:200]}...")
                count += 1
                if count >= 5:
                    break
            print(f"   ✅ Received {count} messages")
        else:
            print("   ⚠️  /turtle1/pose not available (start turtlesim to test)")

        # Export tools to MCP format
        print("\n🔧 Exporting to MCP format...")
        mcp_tools = discovery.to_mcp_tools()
        if mcp_tools:
            print(f"   Exported {len(mcp_tools)} tools to MCP format")
            print(f"   Example: {json.dumps(mcp_tools[0], indent=2)[:200]}...")

        # Keep bridge running
        print("\n⏳ Bridge running. Press Ctrl+C to stop...")
        while bridge.running:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\n\n🛑 Stopping bridge...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await bridge.stop()
        print("✅ Bridge stopped")


if __name__ == "__main__":
    asyncio.run(main())
