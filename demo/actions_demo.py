#!/usr/bin/env python3
"""ROS Actions Demo - Navigation and Motion Planning

Demonstrates ROS Actions for long-running tasks like:
- Navigation (navigate_to_pose)
- Motion planning (follow_joint_trajectory)
- Manipulation (pick/place with feedback)

Usage:
    python demo/actions_demo.py --action navigate
    python demo/actions_demo.py --action manipulate
"""

import asyncio
import argparse
import logging

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.actions import create_action_client, ActionFeedback, ActionResult

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("actions_demo")


async def demo_navigation():
    """Demo: Navigate to pose action"""
    print("\n🧭 NAVIGATION ACTION DEMO")
    print("=" * 60)
    
    # Create action client for navigation
    client = create_action_client(
        action_name="navigate_to_pose",
        action_type="nav2_msgs/action/NavigateToPose",
        ros_version="ros2"
    )
    
    # Connect to action server
    connected = await client.connect()
    if not connected:
        print("⚠️  Using mock action client (no real ROS)")
    
    # Register feedback callback
    def on_feedback(feedback: ActionFeedback):
        data = feedback.feedback_data
        if "distance_remaining" in data:
            print(f"  📍 Distance: {data['distance_remaining']:.2f}m, "
                  f"Speed: {data.get('current_speed', 0):.2f}m/s")
    
    def on_result(result: ActionResult):
        status = "✅" if result.success else "❌"
        print(f"\n{status} Navigation {result.status.name}")
        print(f"   Time: {result.execution_time_sec:.1f}s")
        if result.error_message:
            print(f"   Error: {result.error_message}")
    
    client.register_feedback_callback(on_feedback)
    client.register_result_callback(on_result)
    
    # Navigation goals
    goals = [
        {"pose": {"x": 5.0, "y": 3.0, "theta": 1.57}},
        {"pose": {"x": 2.0, "y": 2.0, "theta": 0.0}},
        {"pose": {"x": 0.0, "y": 0.0, "theta": 0.0}},  # Return home
    ]
    
    for i, goal_data in enumerate(goals, 1):
        print(f"\n{i}. Navigating to ({goal_data['pose']['x']}, {goal_data['pose']['y']})...")
        
        result = await client.send_goal(goal_data, timeout_sec=30.0)
        
        if not result.success:
            print(f"   ⚠️  Navigation failed: {result.error_message}")
        
        await asyncio.sleep(1)
    
    await client.disconnect()
    print("\n✅ Navigation demo complete!")


async def demo_manipulation():
    """Demo: Follow joint trajectory action"""
    print("\n🦾 MANIPULATION ACTION DEMO")
    print("=" * 60)
    
    client = create_action_client(
        action_name="follow_joint_trajectory",
        action_type="control_msgs/action/FollowJointTrajectory",
        ros_version="ros2"
    )
    
    await client.connect()
    
    def on_feedback(feedback: ActionFeedback):
        data = feedback.feedback_data
        if "progress" in data:
            print(f"  🔧 Progress: {data['progress']*100:.0f}%")
    
    client.register_feedback_callback(on_feedback)
    
    # Joint trajectory goal
    goal_data = {
        "trajectory": {
            "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            "points": [
                {"positions": [0, -1.57, 0, -1.57, 0, 0], "time_from_start": 2.0},
                {"positions": [0.5, -1.0, 0.5, -1.5, -0.5, 0], "time_from_start": 4.0},
                {"positions": [0, -1.57, 0, -1.57, 0, 0], "time_from_start": 6.0},
            ]
        }
    }
    
    print("\nExecuting joint trajectory...")
    result = await client.send_goal(goal_data, timeout_sec=15.0)
    
    print(f"\n{'✅' if result.success else '❌'} Trajectory {result.status.name}")
    
    await client.disconnect()


async def demo_cancel():
    """Demo: Cancel ongoing action"""
    print("\n🛑 CANCEL ACTION DEMO")
    print("=" * 60)
    
    client = create_action_client(
        action_name="navigate_to_pose",
        action_type="nav2_msgs/action/NavigateToPose"
    )
    
    await client.connect()
    
    print("\nStarting long navigation...")
    
    # Start goal in background
    goal_task = asyncio.create_task(
        client.send_goal({"pose": {"x": 100.0, "y": 100.0}}, timeout_sec=60.0)
    )
    
    # Let it run for 3 seconds
    await asyncio.sleep(3)
    
    print("\n⏹️  Cancelling goal...")
    cancelled = await client.cancel_goal()
    
    if cancelled:
        print("✅ Goal cancelled successfully")
    else:
        print("⚠️  Could not cancel (may already be complete)")
    
    # Wait for result
    result = await goal_task
    print(f"Final status: {result.status.name}")
    
    await client.disconnect()


async def interactive_mode():
    """Interactive action control via WebSocket"""
    print("\n🎮 INTERACTIVE ACTION CONTROL")
    print("=" * 60)
    print("WebSocket: ws://localhost:8773")
    print("\nAvailable commands:")
    print('  {"command": {"action": "actions.navigate", "parameters": {"x": 5.0, "y": 3.0}}}')
    print('  {"command": {"action": "actions.cancel"}}}')
    print('  {"command": {"action": "actions.status"}}}')


async def main():
    parser = argparse.ArgumentParser(description="ROS Actions Demo")
    parser.add_argument("--action", choices=["navigate", "manipulate", "cancel", "interactive"],
                       default="interactive", help="Demo type")
    parser.add_argument("--ros-version", default="ros2", help="ROS version")
    args = parser.parse_args()
    
    # Create bridge
    bridge = Bridge()
    ws_transport = WebSocketTransport({'port': 8773})
    bridge.transport_manager.register(ws_transport)
    await bridge.start()
    
    print("=" * 60)
    print("⚡ ROS ACTIONS DEMO")
    print("=" * 60)
    print("ROS Actions provide:")
    print("  • Goal → Feedback → Result lifecycle")
    print("  • Long-running task support")
    print("  • Cancel/preempt operations")
    print("  • Progress updates")
    print("=" * 60)
    
    try:
        if args.action == "navigate":
            await demo_navigation()
        elif args.action == "manipulate":
            await demo_manipulation()
        elif args.action == "cancel":
            await demo_cancel()
        else:
            await interactive_mode()
            print("\n⏳ Waiting... (Press Ctrl+C to stop)")
            while True:
                await asyncio.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping...")
    finally:
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
