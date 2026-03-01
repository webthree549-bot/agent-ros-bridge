#!/usr/bin/env python3
"""Arm Robot Demo - UR5/xArm/Franka Control

Demonstrates arm robot manipulation with pick-and-place operations.

Usage:
    python demo/arm_demo.py --arm-type ur --ros2
    python demo/arm_demo.py --arm-type xarm --ros2
"""

import argparse
import asyncio
import logging

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.plugins.arm_robot import ArmRobotPlugin, CartesianPose, move_ur_to_home

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("arm_demo")


async def demo_ur5_pick_place(arm: ArmRobotPlugin):
    """Demonstrate UR5 pick and place"""
    print("\n🦾 UR5 Pick and Place Demo")
    print("=" * 50)

    # 1. Get current state
    print("\n1. Getting current state...")
    state = await arm.handle_command("arm.get_state", {})
    print(f"   State: {state.get('state')}")
    print(f"   Joints: {state.get('joints')}")

    # 2. Move to home
    print("\n2. Moving to home position...")
    result = await move_ur_to_home(arm)
    print(f"   Result: {result}")

    # 3. Open gripper
    print("\n3. Opening gripper...")
    result = await arm.handle_command("arm.gripper", {"position": 0.0})
    print(f"   Result: {result}")

    # 4. Move to pick position
    print("\n4. Moving to pick position...")
    pick_joints = [0.5, -1.0, 0.5, -1.5, -0.5, 0.0]
    result = await arm.handle_command("arm.move_joints", {"joints": pick_joints})
    print(f"   Result: {result}")

    # 5. Close gripper (pick object)
    print("\n5. Closing gripper (picking object)...")
    result = await arm.handle_command("arm.gripper", {"position": 0.8})
    print(f"   Result: {result}")

    # 6. Move to place position
    print("\n6. Moving to place position...")
    place_joints = [-0.5, -1.0, 0.5, -1.5, -0.5, 0.0]
    result = await arm.handle_command("arm.move_joints", {"joints": place_joints})
    print(f"   Result: {result}")

    # 7. Open gripper (place object)
    print("\n7. Opening gripper (placing object)...")
    result = await arm.handle_command("arm.gripper", {"position": 0.0})
    print(f"   Result: {result}")

    # 8. Return home
    print("\n8. Returning to home...")
    result = await move_ur_to_home(arm)
    print(f"   Result: {result}")

    print("\n✅ Pick and place complete!")


async def demo_cartesian_moves(arm: ArmRobotPlugin):
    """Demonstrate cartesian space moves"""
    print("\n📍 Cartesian Movement Demo")
    print("=" * 50)

    poses = [
        CartesianPose(0.5, 0.0, 0.3),  # Front
        CartesianPose(0.5, 0.2, 0.3),  # Front-left
        CartesianPose(0.5, -0.2, 0.3),  # Front-right
        CartesianPose(0.4, 0.0, 0.4),  # Up
        CartesianPose(0.4, 0.0, 0.2),  # Down
    ]

    for i, pose in enumerate(poses):
        print(f"\n{i + 1}. Moving to ({pose.x}, {pose.y}, {pose.z})...")
        result = await arm.controller.move_cartesian(pose)
        print(f"   Result: {result}")
        await asyncio.sleep(1)


async def interactive_control(arm: ArmRobotPlugin):
    """Interactive arm control via WebSocket"""
    print("\n🎮 Interactive Arm Control")
    print("=" * 50)
    print("WebSocket: ws://localhost:8772")
    print("\nAvailable commands:")
    print('  {"command": {"action": "arm.get_state"}}')
    print(
        '  {"command": {"action": "arm.move_joints", "parameters": {"joints": [0, -1.57, 0, -1.57, 0, 0]}}}'
    )
    print('  {"command": {"action": "arm.gripper", "parameters": {"position": 0.5}}}')
    print('  {"command": {"action": "arm.stop"}}}')


async def main():
    parser = argparse.ArgumentParser(description="Arm Robot Demo")
    parser.add_argument(
        "--arm-type", choices=["ur", "xarm", "franka"], default="ur", help="Arm robot type"
    )
    parser.add_argument(
        "--ros-version", choices=["ros1", "ros2"], default="ros2", help="ROS version"
    )
    parser.add_argument("--namespace", default="", help="Robot namespace")
    parser.add_argument(
        "--demo",
        choices=["pick_place", "cartesian", "interactive"],
        default="interactive",
        help="Demo type",
    )
    args = parser.parse_args()

    # Create bridge
    bridge = Bridge()

    # Add WebSocket transport
    ws_transport = WebSocketTransport({"port": 8772})
    bridge.transport_manager.register(ws_transport)

    # Create arm plugin
    print(f"🦾 Initializing {args.arm_type.upper()} robot ({args.ros_version})...")
    arm = ArmRobotPlugin(
        arm_type=args.arm_type, ros_version=args.ros_version, namespace=args.namespace
    )

    # Initialize
    success = await arm.initialize(bridge)
    if not success:
        print("⚠️  Could not connect to real robot, running in demo mode")

    # Start bridge
    await bridge.start()

    print("=" * 50)
    print(f"🦾 {args.arm_type.upper()} Arm Robot Demo")
    print("=" * 50)
    print(f"Type: {args.arm_type}")
    print(f"ROS: {args.ros_version}")
    print("WebSocket: ws://localhost:8772")
    print("=" * 50)

    # Run demo
    try:
        if args.demo == "pick_place":
            await demo_ur5_pick_place(arm)
        elif args.demo == "cartesian":
            await demo_cartesian_moves(arm)
        else:
            await interactive_control(arm)

            # Keep running for interactive control
            print("\n⏳ Waiting for commands... (Press Ctrl+C to stop)")
            while True:
                await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping...")
    finally:
        await arm.shutdown()
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
