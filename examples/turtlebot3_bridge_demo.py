#!/usr/bin/env python3
"""
TurtleBot3 Bridge Demo

Demonstrates how to use agent-ros-bridge to control the TurtleBot3
simulation running in the Docker container.

Prerequisites:
- Docker container 'ros2_humble' running with TurtleBot3 simulation
- agent-ros-bridge installed

Usage:
    python turtlebot3_bridge_demo.py
"""

import asyncio
import sys
from datetime import datetime

# Add the workspace to path if running from within it
sys.path.insert(0, '/workspace')

try:
    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector, ROS2Robot
    from agent_ros_bridge.gateway_v2.core import Command
except ImportError:
    print("❌ agent_ros_bridge not found. Make sure it's installed.")
    sys.exit(1)


class TurtleBot3BridgeDemo:
    """Demo class showing how to control TurtleBot3 via the bridge."""

    def __init__(self):
        self.connector = None
        self.robot = None

    async def connect(self):
        """Connect to the TurtleBot3 in the Docker container."""
        print("🤖 Connecting to TurtleBot3 simulation...")

        # Create the ROS2 connector
        self.connector = ROS2Connector()

        # Connect to the robot via the Docker container's ROS2 environment
        # The container exposes the ROS2 topics via shared network
        robot = await self.connector.connect_robot(
            robot_id="tb3_001",
            name="TurtleBot3 Burger",
            uri="ros2://localhost:11311"  # ROS2 master URI
        )

        if robot:
            self.robot = robot
            print(f"✅ Connected to {robot.name}")
            print(f"   Robot ID: {robot.robot_id}")
            print(f"   Capabilities: {robot.capabilities}")
            return True
        else:
            print("❌ Failed to connect to robot")
            return False

    async def demo_get_topics(self):
        """Demo: List available ROS topics."""
        print("\n📡 Available ROS Topics:")
        print("-" * 50)

        result = await self.robot.execute(Command(
            action="get_topics",
            parameters={}
        ))

        if result:
            for topic in result[:10]:  # Show first 10
                print(f"  • {topic['name']} [{topic['type']}]")

        return result

    async def demo_read_sensors(self):
        """Demo: Read sensor data from the robot."""
        print("\n📊 Reading Sensor Data:")
        print("-" * 50)

        # Subscribe to odometry for 3 seconds
        print("\n1. Odometry (position tracking):")
        count = 0
        async for telemetry in self.robot.subscribe(
            "/odom",
            msg_type="nav_msgs/Odometry"
        ):
            pose = telemetry.data.get('pose', {}).get('pose', {})
            position = pose.get('position', {})
            print(f"   Position: x={position.get('x', 0):.2f}, "
                  f"y={position.get('y', 0):.2f}")

            count += 1
            if count >= 3:
                break

        # Subscribe to laser scan once
        print("\n2. Laser Scan (obstacle detection):")
        async for telemetry in self.robot.subscribe(
            "/scan",
            msg_type="sensor_msgs/LaserScan"
        ):
            ranges = telemetry.data.get('ranges', [])
            if ranges:
                min_distance = min(r for r in ranges if r > 0)
                print(f"   Closest obstacle: {min_distance:.2f} meters")
            break

        # Subscribe to IMU once
        print("\n3. IMU (orientation):")
        async for telemetry in self.robot.subscribe(
            "/imu",
            msg_type="sensor_msgs/Imu"
        ):
            orientation = telemetry.data.get('orientation', {})
            print(f"   Orientation: x={orientation.get('x', 0):.2f}, "
                  f"y={orientation.get('y', 0):.2f}")
            break

    async def demo_move_robot(self):
        """Demo: Move the robot using velocity commands."""
        print("\n🚀 Moving Robot:")
        print("-" * 50)

        # Move forward
        print("\n1. Moving forward...")
        await self.robot.execute(Command(
            action="publish",
            parameters={
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }
        ))
        await asyncio.sleep(2)

        # Stop
        print("2. Stopping...")
        await self.robot.execute(Command(
            action="publish",
            parameters={
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }
        ))

        # Rotate
        print("3. Rotating...")
        await self.robot.execute(Command(
            action="publish",
            parameters={
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
                }
            }
        ))
        await asyncio.sleep(2)

        # Stop again
        print("4. Stopping...")
        await self.robot.execute(Command(
            action="publish",
            parameters={
                "topic": "/cmd_vel",
                "msg_type": "geometry_msgs/Twist",
                "message": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }
        ))

        print("✅ Movement complete")

    async def demo_continuous_monitoring(self):
        """Demo: Continuously monitor robot state."""
        print("\n📈 Continuous Monitoring (5 seconds):")
        print("-" * 50)

        start_time = datetime.now()

        async for telemetry in self.robot.subscribe(
            "/odom",
            msg_type="nav_msgs/Odometry"
        ):
            pose = telemetry.data.get('pose', {}).get('pose', {})
            position = pose.get('position', {})

            elapsed = (datetime.now() - start_time).total_seconds()
            print(f"   t={elapsed:.1f}s: x={position.get('x', 0):.3f}, "
                  f"y={position.get('y', 0):.3f}")

            if elapsed >= 5.0:
                break

    async def disconnect(self):
        """Disconnect from the robot."""
        if self.robot:
            await self.robot.disconnect()
            print("\n👋 Disconnected from robot")


async def main():
    """Main demo function."""
    print("=" * 60)
    print("🐢 TurtleBot3 Bridge Demo")
    print("=" * 60)
    print()
    print("This demo shows how to use agent-ros-bridge to:")
    print("  1. Connect to the TurtleBot3 simulation")
    print("  2. List available ROS topics")
    print("  3. Read sensor data (odometry, laser, IMU)")
    print("  4. Send movement commands")
    print("  5. Monitor robot state continuously")
    print()

    demo = TurtleBot3BridgeDemo()

    try:
        # Connect to robot
        if not await demo.connect():
            return

        # Run demos
        await demo.demo_get_topics()
        await demo.demo_read_sensors()
        await demo.demo_move_robot()
        await demo.demo_continuous_monitoring()

    except KeyboardInterrupt:
        print("\n\n⚠️ Demo interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await demo.disconnect()

    print()
    print("=" * 60)
    print("✅ Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
