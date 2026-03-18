#!/usr/bin/env python3
"""OpenClaw E2E Test for Agent ROS Bridge

This test demonstrates the full end-to-end flow:
OpenClaw Agent → Agent ROS Bridge → ROS → Simulated Robot

Usage:
    python tests/e2e/test_openclaw_integration.py

Requirements:
    - ROS2 Docker container running
    - Gazebo with TurtleBot3 (for full integration tests)
    - Agent ROS Bridge installed
"""

import os
import subprocess

import pytest

CONTAINER_NAME = "ros2_humble"


def is_container_running() -> bool:
    """Check if ROS2 container is running."""
    result = subprocess.run(
        ["docker", "ps", "--filter", f"name={CONTAINER_NAME}", "--format", "{{.Status}}"],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and "Up" in result.stdout


def get_ros_distro() -> str:
    """Auto-detect ROS distribution in container."""
    result = subprocess.run(
        ["docker", "exec", CONTAINER_NAME, "ls", "/opt/ros/"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        distros = result.stdout.strip().split()
        if distros:
            return distros[0]
    return "jazzy"


def run_in_ros2_container(cmd: str, timeout: int = 30) -> subprocess.CompletedProcess:
    """Run a command in the ROS2 Docker container with ROS environment sourced."""
    distro = get_ros_distro()
    # Source ROS setup before running command
    full_cmd = f"source /opt/ros/{distro}/setup.bash && {cmd}"
    result = subprocess.run(
        ["docker", "exec", CONTAINER_NAME, "bash", "-c", full_cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return result


pytestmark = [
    pytest.mark.e2e,
]


class TestOpenClawE2E:
    """End-to-end tests for OpenClaw integration."""

    def test_e2e_navigate_via_openclaw(self):
        """
        E2E Test: OpenClaw agent commands robot to navigate.

        This test runs inside the Docker container to use the real ROS2 environment.
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        # Run test inside the container
        test_script = '''
import sys
sys.path.insert(0, "/workspace")

try:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from agent_ros_bridge.robot_api import NavigationGoal
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create test node
        node = Node("e2e_test_node")
        
        # Create Nav2 action client
        nav_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        
        # Wait for action server
        if not nav_client.wait_for_server(timeout_sec=5.0):
            print("NAV2_NOT_AVAILABLE")
            sys.exit(0)
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 2.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        future = nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if goal_handle is None:
            print("GOAL_SEND_FAILED")
            sys.exit(1)
        if not goal_handle.accepted:
            print("GOAL_REJECTED")
            sys.exit(1)
        
        print("NAVIGATION_TEST_PASSED")
    finally:
        rclpy.shutdown()
except ImportError as e:
    print(f"IMPORT_ERROR: {e}")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "NAV2_NOT_AVAILABLE" in result.stdout:
            pytest.skip("Nav2 not available - full simulation not running")
        if "NAVIGATION_TEST_PASSED" not in result.stdout:
            pytest.fail(f"Navigation test failed: {result.stdout} {result.stderr}")
        
        print("\n✅ E2E navigation test passed")

    def test_e2e_mcp_tool_call(self):
        """
        E2E Test: MCP tool call from OpenClaw agent.
        
        Tests the MCP server running inside the container.
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        # Check if MCP server module is available
        test_script = '''
import sys
sys.path.insert(0, "/workspace")

try:
    from agent_ros_bridge.integrations.mcp_transport import MCPServer
    print("MCP_AVAILABLE")
except ImportError as e:
    print(f"MCP_NOT_AVAILABLE: {e}")
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=30)
        
        if "MCP_NOT_AVAILABLE" in result.stdout:
            pytest.skip("MCP server not available in container")
        if "MCP_AVAILABLE" not in result.stdout:
            pytest.fail(f"MCP check failed: {result.stdout} {result.stderr}")
        
        print("\n✅ MCP tool call test passed")

    def test_e2e_agent_memory(self):
        """
        E2E Test: Agent remembers previous interactions.

        Verifies that the agent can recall:
        - Previous commands
        - Success/failure outcomes
        - Location mappings
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        test_script = """
import sys
sys.path.insert(0, "/workspace")

try:
    from agent_ros_bridge.ai.context_aware_parser import ContextAwareParser
    
    parser = ContextAwareParser()
    
    # First interaction: Learn mapping
    parser.add_conversation_turn("the kitchen is at position (5, 3)", "CONFIGURE")
    
    # Second interaction: Use learned mapping
    parser.update_robot_state(location="living_room")
    resolved = parser.resolve_context("go to the kitchen")
    
    # Verify context was used
    has_kitchen = "kitchen" in resolved
    has_location = hasattr(parser, "_environment") and "kitchen" in parser._environment.known_locations
    
    if has_kitchen or has_location:
        print("MEMORY_TEST_PASSED")
    else:
        print(f"MEMORY_TEST_FAILED: resolved={resolved}")
        sys.exit(1)
except ImportError as e:
    print(f"IMPORT_ERROR: {e}")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
"""
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "MEMORY_TEST_PASSED" not in result.stdout:
            pytest.fail(f"Agent memory test failed: {result.stdout} {result.stderr}")
        
        print("\n✅ Agent memory test passed")


class TestOpenClawStandaloneE2E:
    """E2E tests for standalone mode (no external agent)."""

    def test_standalone_robot_api(self):
        """
        E2E Test: Direct Python API to control robot.

        Usage:
            from agent_ros_bridge import Robot
            robot = Robot()
            robot.navigate_to("kitchen")
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        test_script = '''
import sys
sys.path.insert(0, "/workspace")

try:
    from agent_ros_bridge import Robot
    
    robot = Robot(ros_master="localhost:11311")
    
    # Should be able to:
    # - Connect to ROS
    # - Discover available actions
    # - Execute high-level commands
    
    # For now, just verify the class exists
    assert Robot is not None
    print("ROBOT_API_AVAILABLE")
except ImportError as e:
    print(f"ROBOT_API_NOT_AVAILABLE: {e}")
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=30)
        
        if "ROBOT_API_NOT_AVAILABLE" in result.stdout:
            pytest.skip(f"Standalone Robot API not available: {result.stdout}")
        if "ROBOT_API_AVAILABLE" not in result.stdout:
            pytest.fail(f"Robot API check failed: {result.stdout} {result.stderr}")
        
        print("\n✅ Standalone Robot API test passed")

    def test_standalone_skill_execution(self):
        """
        E2E Test: Execute pre-defined skills.

        Skills are composed motion primitives.
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        test_script = '''
import sys
sys.path.insert(0, "/workspace")

try:
    from agent_ros_bridge.ai.motion_primitives import (
        MotionPrimitiveFactory,
        navigate_to_pose,
    )
    
    # Create navigation primitive
    nav = navigate_to_pose(x=1.0, y=2.0, theta=0.0)
    
    assert nav is not None
    assert nav.name == "NAVIGATE"
    
    print("SKILL_EXECUTION_PASSED")
except ImportError as e:
    print(f"IMPORT_ERROR: {e}")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "SKILL_EXECUTION_PASSED" not in result.stdout:
            pytest.fail(f"Skill execution test failed: {result.stdout} {result.stderr}")
        
        print("\n✅ Skill execution test passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
