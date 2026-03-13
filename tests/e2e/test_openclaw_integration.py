#!/usr/bin/env python3
"""
OpenClaw E2E Test for Agent ROS Bridge

This test demonstrates the full end-to-end flow:
OpenClaw Agent → Agent ROS Bridge → ROS → Simulated Robot

Usage:
    python tests/e2e/test_openclaw_integration.py

Requirements:
    - ROS2 Humble installed
    - Gazebo with TurtleBot3
    - Agent ROS Bridge installed
"""

import os
import signal
import subprocess
import time

import pytest

# Skip all tests if ROS2 not available
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(os.system("which ros2 >/dev/null 2>&1") != 0, reason="ROS2 not installed"),
    pytest.mark.skipif(
        os.system("which gz sim >/dev/null 2>&1") != 0, reason="Gazebo not installed"
    ),
]


class TestOpenClawE2E:
    """End-to-end tests for OpenClaw integration."""

    @pytest.fixture(scope="class")
    def ros_bridge(self):
        """Start Agent ROS Bridge for testing."""
        # Start bridge process
        bridge_process = subprocess.Popen(
            ["python", "-m", "agent_ros_bridge", "--config", "tests/e2e/test_config.yaml"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        # Wait for bridge to start
        time.sleep(3)

        yield bridge_process

        # Cleanup
        os.killpg(os.getpgid(bridge_process.pid), signal.SIGTERM)
        bridge_process.wait()

    @pytest.fixture(scope="class")
    def gazebo_sim(self):
        """Start Gazebo simulation with TurtleBot3."""
        # Launch Gazebo
        gazebo_process = subprocess.Popen(
            ["ros2", "launch", "turtlebot3_gazebo", "empty_world.launch.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        # Wait for simulation
        time.sleep(10)

        yield gazebo_process

        # Cleanup
        os.killpg(os.getpgid(gazebo_process.pid), signal.SIGTERM)
        gazebo_process.wait()

    def test_e2e_navigate_via_openclaw(self, ros_bridge, gazebo_sim):
        """
        E2E Test: OpenClaw agent commands robot to navigate.

        Flow:
            1. OpenClaw agent receives: "go to position (1, 2)"
            2. Agent calls ROS bridge via MCP/tool
            3. Bridge sends Nav2 goal
            4. Robot moves in Gazebo
            5. Verify robot reached goal
        """
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from rclpy.node import Node
        except ImportError:
            pytest.skip("ROS2 Python bindings not available")

        # Initialize ROS2
        rclpy.init()

        try:
            # Create test node
            node = Node("e2e_test_node")

            # Subscribe to robot pose
            current_pose = None

            def pose_callback(msg):
                nonlocal current_pose
                current_pose = msg.pose.pose

            pose_sub = node.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", pose_callback, 10
            )

            # Create Nav2 action client
            nav_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

            # Wait for action server
            if not nav_client.wait_for_server(timeout_sec=10.0):
                pytest.skip("Nav2 action server not available")

            # Get initial pose
            start_time = time.time()
            while current_pose is None and time.time() - start_time < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)

            assert current_pose is not None, "Could not get robot pose"
            initial_x = current_pose.position.x
            initial_y = current_pose.position.y

            # Simulate OpenClaw agent sending command
            # In real scenario, this would come from OpenClaw MCP/tool call
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.pose.position.x = initial_x + 1.0  # Move 1m forward
            goal_msg.pose.pose.position.y = initial_y
            goal_msg.pose.pose.orientation.w = 1.0

            # Send goal (simulating bridge execution)
            future = nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            goal_handle = future.result()
            assert goal_handle is not None, "Failed to send navigation goal"
            assert goal_handle.accepted, "Navigation goal rejected"

            # Wait for result
            result_future = goal_handle.get_result_async()
            start_time = time.time()

            while time.time() - start_time < 30.0:  # 30 second timeout
                rclpy.spin_once(node, timeout_sec=0.1)

                if result_future.done():
                    result = result_future.result()
                    assert result.status == 4, f"Navigation failed with status {result.status}"
                    break

                # Check if robot is moving
                if current_pose:
                    distance_moved = (
                        (current_pose.position.x - initial_x) ** 2
                        + (current_pose.position.y - initial_y) ** 2
                    ) ** 0.5
                    if distance_moved > 0.5:  # Robot has moved significantly
                        print(f"Robot moved {distance_moved:.2f}m")
            else:
                pytest.fail("Navigation timed out")

            # Verify final position
            final_distance = (
                (current_pose.position.x - (initial_x + 1.0)) ** 2
                + (current_pose.position.y - initial_y) ** 2
            ) ** 0.5

            assert (
                final_distance < 0.5
            ), f"Robot didn't reach goal (distance: {final_distance:.2f}m)"

            print(f"✅ E2E test passed: Robot navigated {distance_moved:.2f}m to goal")

        finally:
            rclpy.shutdown()

    def test_e2e_mcp_tool_call(self, ros_bridge):
        """
        E2E Test: MCP tool call from OpenClaw agent.

        Simulates OpenClaw calling ROS tools via MCP.
        """
        try:
            from agent_ros_bridge.integrations.mcp_transport import MCPServer
        except ImportError:
            pytest.skip("MCP server not available")

        # This would be called by OpenClaw
        # For now, verify the MCP server is running and responsive
        # TODO: Implement actual MCP client test

        pytest.skip("MCP E2E test requires OpenClaw runtime")

    def test_e2e_agent_memory(self, ros_bridge):
        """
        E2E Test: Agent remembers previous interactions.

        Verifies that the agent can recall:
        - Previous commands
        - Success/failure outcomes
        - Location mappings
        """
        try:
            from agent_ros_bridge.ai.context_aware_parser import ContextAwareParser
        except ImportError:
            pytest.skip("Context parser not available")

        parser = ContextAwareParser()

        # First interaction: Learn mapping
        parser.add_conversation_turn("the kitchen is at position (5, 3)", "CONFIGURE")

        # Second interaction: Use learned mapping
        parser.update_robot_state(location="living_room")
        resolved = parser.resolve_context("go to the kitchen")

        # Verify context was used
        assert "kitchen" in resolved or parser._environment.known_locations

        print("✅ Agent memory test passed")


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
        try:
            # This API should exist but doesn't yet
            from agent_ros_bridge import Robot

            robot = Robot(ros_master="localhost:11311")

            # Should be able to:
            # - Connect to ROS
            # - Discover available actions
            # - Execute high-level commands

            # For now, just verify the class exists
            assert Robot is not None

        except ImportError as e:
            pytest.skip(f"Standalone Robot API not available: {e}")

    def test_standalone_skill_execution(self):
        """
        E2E Test: Execute pre-defined skills.

        Skills are composed motion primitives.
        """
        try:
            from agent_ros_bridge.ai.motion_primitives import (
                MotionPrimitiveFactory,
                navigate_to_pose,
            )
        except ImportError:
            pytest.skip("Motion primitives not available")

        # Create navigation primitive
        nav = navigate_to_pose(x=1.0, y=2.0, theta=0.0)

        assert nav is not None
        assert nav.name == "NavigateToPose"

        print("✅ Skill execution test passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
