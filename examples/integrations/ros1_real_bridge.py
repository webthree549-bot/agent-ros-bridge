#!/usr/bin/env python3
"""ROS1 Real Bridge Example.

This example demonstrates:
1. Creating a bridge with ROS1 connector
2. Discovering ROS1 topics
3. Connecting to a ROS1 robot
4. Publishing messages
5. Subscribing to topics
6. Tool discovery for AI integration

Usage:
    # Terminal 1: Start roscore
    $ roscore

    # Terminal 2: Start turtlesim
    $ rosrun turtlesim turtlesim_node

    # Terminal 3: Run this example
    $ python examples/ros1_real_bridge.py
"""

import asyncio
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ros1_example")

# Mock mode for testing without ROS1
MOCK_MODE = True


async def main():
    """Run ROS1 bridge example."""
    logger.info("=" * 60)
    logger.info("Agent ROS Bridge - ROS1 Example")
    logger.info("=" * 60)

    try:
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector
        from agent_ros_bridge.gateway_v2.core import Bridge, Command
    except ImportError as e:
        logger.error(f"Failed to import agent_ros_bridge: {e}")
        return

    # Create bridge
    bridge = Bridge()
    logger.info("✓ Bridge created")

    # Register ROS1 connector
    ros1 = ROS1Connector()
    bridge.connector_registry.register(ros1)
    logger.info("✓ ROS1 connector registered")

    # Discover robots
    logger.info("\n🔍 Discovering ROS1 systems...")
    try:
        endpoints = await bridge.connector_registry.discover_all()
        logger.info(f"Found {len(endpoints)} ROS1 endpoints:")
        for ep in endpoints:
            logger.info(f"  - {ep.name}: {ep.uri}")
            logger.info(f"    Capabilities: {ep.capabilities}")
    except Exception as e:
        logger.warning(f"Discovery failed (expected if ROS not running): {e}")
        endpoints = []

    # Try to connect to ROS1
    logger.info("\n🔌 Connecting to ROS1...")
    try:
        if endpoints:
            robot = await bridge.connect_robot(endpoints[0].uri)
        else:
            # Try default connection
            robot = await ros1.connect("ros1:///")

        if not robot:
            logger.error("Failed to connect to ROS1")
            return

        logger.info(f"✓ Connected to robot: {robot.name}")
        logger.info(f"  ID: {robot.robot_id}")
        logger.info(f"  Type: {robot.connector_type}")
        logger.info(f"  Capabilities: {robot.capabilities}")

        # Get topics
        logger.info("\n📡 Getting available topics...")
        topics = await robot.execute(Command(action="get_topics", parameters={}))
        logger.info(f"Found {len(topics)} topics:")
        for topic in topics[:10]:  # Show first 10
            logger.info(f"  - {topic.get('name', 'unknown')}: {topic.get('type', 'unknown')}")

        # Get services
        logger.info("\n🔧 Getting available services...")
        services = await robot.execute(Command(action="get_services", parameters={}))
        logger.info(f"Found {len(services)} services:")
        for svc in services[:5]:  # Show first 5
            logger.info(f"  - {svc.get('name', 'unknown')}")

        # Publish to a topic (example)
        logger.info("\n📤 Publishing example message...")
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
        logger.info(f"Publish result: {result}")

        # Subscribe to a topic
        logger.info("\n📥 Subscribing to /turtle1/pose...")
        try:
            msg_count = 0
            async for telemetry in robot.subscribe("/turtle1/pose", msg_type="turtlesim/Pose"):
                logger.info(
                    f"  Received: x={telemetry.data.get('x', 0):.2f}, y={telemetry.data.get('y', 0):.2f}"
                )
                msg_count += 1
                if msg_count >= 3:
                    break
            logger.info(f"✓ Received {msg_count} messages")
        except Exception as e:
            logger.warning(f"Subscribe failed: {e}")

        # Tool discovery
        logger.info("\n🤖 Discovering AI tools...")
        discovery = bridge.get_tool_discovery()
        tools = discovery.discover_all()
        logger.info(f"Found {len(tools)} AI tools:")
        for tool in tools[:10]:
            danger_flag = " ⚠️" if tool.dangerous else ""
            logger.info(f"  - {tool.name}: {tool.description[:50]}...{danger_flag}")

        # Export to MCP format
        logger.info("\n📋 MCP Tools format:")
        mcp_tools = discovery.to_mcp_tools(tools[:3])
        for tool in mcp_tools:
            logger.info(f"  - {tool['name']}: {tool['description'][:40]}...")

        # Disconnect
        logger.info("\n🔌 Disconnecting...")
        await robot.disconnect()
        logger.info("✓ Disconnected")

    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback

        traceback.print_exc()

    logger.info("\n" + "=" * 60)
    logger.info("Example completed!")
    logger.info("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
