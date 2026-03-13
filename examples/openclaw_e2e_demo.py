#!/usr/bin/env python3
"""
OpenClaw E2E Demo for Agent ROS Bridge

This script demonstrates how OpenClaw would interact with Agent ROS Bridge
to control a robot in Gazebo simulation.

Prerequisites:
    - ROS2 Humble installed
    - Gazebo Ignition installed
    - TurtleBot3 packages: sudo apt install ros-humble-turtlebot3*
    - Agent ROS Bridge installed: pip install agent-ros-bridge

Usage:
    # Terminal 1: Start Gazebo
    ros2 launch turtlebot3_gazebo empty_world.launch.py

    # Terminal 2: Start Agent ROS Bridge with MCP
    python -m agent_ros_bridge --mcp-server

    # Terminal 3: Run this demo
    python examples/openclaw_e2e_demo.py
"""

import sys


def check_prerequisites():
    """Check if required components are available."""
    try:
        import rclpy

        print("✅ ROS2 Python bindings available")
    except ImportError:
        print("❌ ROS2 Python bindings not found")
        print("   Install: sudo apt install ros-humble-rclpy")
        return False

    try:
        import agent_ros_bridge

        print(f"✅ Agent ROS Bridge v{agent_ros_bridge.__version__} available")
    except ImportError:
        print("❌ Agent ROS Bridge not found")
        print("   Install: pip install agent-ros-bridge")
        return False

    return True


def demo_mcp_tool_call():
    """
    Demo: OpenClaw agent calls ROS tools via MCP.

    This simulates what OpenClaw would do when it wants to control a robot.
    """
    print("\n" + "=" * 60)
    print("DEMO 1: MCP Tool Call (Agent Runtime Mode)")
    print("=" * 60)

    print("\nScenario: OpenClaw agent receives command from user")
    print("User: 'Navigate to the kitchen'")

    print("\nOpenClaw agent reasoning:")
    print("  1. Parse intent: NAVIGATE to location 'kitchen'")
    print("  2. Recall: kitchen is at map coordinates (3.5, 2.0)")
    print("  3. Call MCP tool: ros_navigate_to_pose")

    # Simulate MCP tool call
    print("\nMCP Tool Call:")
    print("  tool: ros_navigate_to_pose")
    print("  arguments:")
    print("    pose:")
    print("      position: {x: 3.5, y: 2.0, z: 0.0}")
    print("      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}")

    print("\nAgent ROS Bridge:")
    print("  1. Receives MCP tool call")
    print("  2. Validates safety constraints")
    print("  3. Sends Nav2 action goal")

    print("\nRobot in Gazebo:")
    print("  1. Receives navigation goal")
    print("  2. Plans path using Nav2")
    print("  3. Executes movement")
    print("  4. Reports success")

    print("\nOpenClaw agent:")
    print("  1. Receives success response")
    print("  2. Updates memory: 'Kitchen navigation successful'")
    print("  3. Responds to user: 'I've arrived at the kitchen'")

    print("\n✅ Demo 1 complete")


def demo_standalone_api():
    """
    Demo: Direct Python API without external agent.

    This shows how to use Agent ROS Bridge as a standalone library.
    """
    print("\n" + "=" * 60)
    print("DEMO 2: Standalone Python API")
    print("=" * 60)

    print("\nScenario: Python script controls robot directly")

    code = """
from agent_ros_bridge import RobotController, NavigationGoal

# Connect to robot
robot = Robot(ros_master="localhost:11311")

# Define navigation goal
goal = NavigationGoal(
    x=3.5,
    y=2.0,
    theta=0.0,
    frame_id="map"
)

# Execute navigation
result = robot.navigate_to(goal)

if result.success:
    print("Robot reached destination!")
    print(f"Execution time: {result.execution_time:.2f}s")
else:
    print(f"Navigation failed: {result.error_message}")
"""

    print("\nExample code:")
    print(code)

    print("\nWhat this would do:")
    print("  1. Connect to ROS master")
    print("  2. Discover available navigation capabilities")
    print("  3. Send goal to Nav2 action server")
    print("  4. Monitor execution progress")
    print("  5. Return result with timing and status")

    print("\n⚠️  Note: This API is planned but not fully implemented yet")
    print("   Current: Use low-level ROS interface directly")
    print("   Future: High-level Robot class as shown above")

    print("\n✅ Demo 2 complete")


def demo_agent_learning():
    """
    Demo: Agent learns from interactions.

    Shows how the agent improves over time.
    """
    print("\n" + "=" * 60)
    print("DEMO 3: Agent Learning Loop")
    print("=" * 60)

    print("\nFirst interaction:")
    print("  User: 'Go to the place where we eat'")
    print("  Agent: 'Do you mean the kitchen or the dining room?'")
    print("  User: 'The kitchen'")
    print("  Agent: [Navigates to kitchen]")
    print("  Agent: [Learns: 'place where we eat' = 'kitchen']")

    print("\nSecond interaction:")
    print("  User: 'Go to the place where we eat'")
    print("  Agent: [Recalls: 'place where we eat' = 'kitchen']")
    print("  Agent: [Navigates to kitchen directly]")
    print("  Agent: 'Going to the kitchen'")

    print("\nThird interaction (failure case):")
    print("  User: 'Go to the garden'")
    print("  Agent: [Navigates but path is blocked]")
    print("  Agent: 'I couldn't reach the garden - path blocked'")
    print("  Agent: [Learns: garden route blocked, try alternative]")

    print("\nKey capabilities demonstrated:")
    print("  ✅ Context-aware understanding")
    print("  ✅ Memory of user preferences")
    print("  ✅ Learning from outcomes")
    print("  ✅ Graceful failure handling")

    print("\n✅ Demo 3 complete")


def demo_safety_validation():
    """
    Demo: Safety validation prevents dangerous commands.
    """
    print("\n" + "=" * 60)
    print("DEMO 4: Safety Validation")
    print("=" * 60)

    print("\nScenario 1: Safe command")
    print("  User: 'Move forward 1 meter at 0.5 m/s'")
    print("  Safety check: ✅ Velocity 0.5 m/s < limit 1.0 m/s")
    print("  Safety check: ✅ Distance 1m in clear path")
    print("  Result: ✅ Command executed")

    print("\nScenario 2: Unsafe velocity")
    print("  User: 'Move forward at 5 m/s'")
    print("  Safety check: ❌ Velocity 5.0 m/s > limit 1.0 m/s")
    print("  Result: ❌ Command rejected")
    print("  Agent: 'That speed is unsafe. I'll use maximum safe speed of 1 m/s.'")

    print("\nScenario 3: Obstacle detected")
    print("  User: 'Move forward 3 meters'")
    print("  Safety check: ⚠️ Obstacle detected at 2 meters")
    print("  Result: ⚠️ Command modified - stops at 1.9 meters")
    print("  Agent: 'I stopped early because there's an obstacle ahead.'")

    print("\n✅ Demo 4 complete")


def main():
    """Run all demos."""
    print("\n" + "=" * 60)
    print("AGENT ROS BRIDGE - OpenClaw E2E Demo")
    print("=" * 60)

    # Check prerequisites
    print("\nChecking prerequisites...")
    if not check_prerequisites():
        print("\n❌ Prerequisites not met. Please install required components.")
        print("\nFor simulation only (no ROS):")
        print("  pip install agent-ros-bridge")
        print("\nFor full E2E with Gazebo:")
        print("  1. Install ROS2 Humble")
        print("  2. Install Gazebo Ignition")
        print("  3. Install TurtleBot3: sudo apt install ros-humble-turtlebot3*")
        sys.exit(1)

    # Run demos
    try:
        demo_mcp_tool_call()
        demo_standalone_api()
        demo_agent_learning()
        demo_safety_validation()

        print("\n" + "=" * 60)
        print("ALL DEMOS COMPLETE")
        print("=" * 60)
        print("\nNext steps:")
        print("  1. Start Gazebo: ros2 launch turtlebot3_gazebo empty_world.launch.py")
        print("  2. Start bridge: python -m agent_ros_bridge --mcp-server")
        print("  3. Connect OpenClaw and try: 'Navigate to position (1, 2)'")
        print("\nFor real E2E testing, run:")
        print("  pytest tests/e2e/test_openclaw_integration.py -v")

    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()
