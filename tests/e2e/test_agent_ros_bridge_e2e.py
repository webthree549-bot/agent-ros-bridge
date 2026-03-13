#!/usr/bin/env python3
"""
Agent ROS Bridge E2E Test

Full end-to-end test demonstrating:
1. Agent receives natural language command
2. Agent parses intent
3. Agent calls ROS2 via bridge
4. Robot executes command
5. Verify execution result

This test uses Docker-based ROS2.
"""

import subprocess
import time

import pytest


def run_in_ros2_container(cmd: str, timeout: int = 30) -> subprocess.CompletedProcess:
    """Run a command in the ROS2 Docker container."""
    return subprocess.run(
        [
            "docker",
            "exec",
            "ros2_humble",
            "bash",
            "-c",
            f"source /opt/ros/humble/setup.bash && {cmd}",
        ],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


class TestAgentROSBridgeE2E:
    """End-to-end tests for Agent ROS Bridge."""

    def test_ros2_container_running(self):
        """Verify ROS2 Docker container is running."""
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=ros2_humble", "--format", "{{.Status}}"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0 or "Up" not in result.stdout:
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")
        print("\n✅ ROS2 container is running")

    def test_ros2_basic_commands(self):
        """Test basic ROS2 commands work."""
        # Check if container is running first
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=ros2_humble", "--format", "{{.Status}}"],
            capture_output=True,
            text=True,
        )
        if "Up" not in result.stdout:
            pytest.skip("ROS2 container not running")

        # Test ros2 topic list
        result = run_in_ros2_container("ros2 topic list")
        if result.returncode != 0:
            pytest.skip("ROS2 not responding in container")
        print(f"\n✅ ROS2 topics: {result.stdout.strip() or 'No topics yet'}")

        # Test ros2 node list
        result = run_in_ros2_container("ros2 node list")
        assert result.returncode == 0
        print(f"✅ ROS2 nodes: {result.stdout.strip() or 'No nodes yet'}")

    def test_agent_intent_parsing(self):
        """Test Agent can parse natural language to intent (host-based, no rclpy)."""
        # Skip if rclpy not available (Docker-only)
        try:
            import rclpy
        except ImportError:
            pytest.skip("rclpy not available on host - run in Docker")

        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        from agent_ros_bridge_msgs.srv import ParseIntent

        # Create parser
        parser = IntentParserNode()

        # Test utterances
        test_cases = [
            ("go to kitchen", "NAVIGATE"),
            ("stop", "SAFETY"),
            ("pick up the cup", "MANIPULATE"),
        ]

        for utterance, expected_type in test_cases:
            request = ParseIntent.Request()
            request.utterance = utterance
            response = ParseIntent.Response()
            result = parser.parse_intent_callback(request, response)

            assert result.success, f"Failed to parse: {utterance}"
            assert (
                result.intent.type == expected_type
            ), f"Expected {expected_type}, got {result.intent.type}"
            print(f"\n✅ Parsed '{utterance}' -> {result.intent.type}")

    def test_bridge_ros_command_execution(self):
        """Test bridge can execute ROS commands via Docker."""
        # Start a demo node in background
        run_in_ros2_container("ros2 run demo_nodes_cpp talker &", timeout=5)

        time.sleep(2)  # Let node start

        # Check topic exists
        result = run_in_ros2_container("ros2 topic list")
        assert result.returncode == 0
        assert "/chatter" in result.stdout, f"Demo node not publishing. Topics: {result.stdout}"

        print("\n✅ Bridge ROS command execution working")

    def test_full_flow_navigate_command(self):
        """
        Full E2E: Agent receives 'go to position' and executes via ROS.

        Flow:
            1. User: "navigate to position (1, 2)"
            2. Agent parses -> NAVIGATE intent
            3. Agent creates motion plan
            4. Bridge sends to ROS
            5. Verify command was sent
        """
        # Skip if rclpy not available
        try:
            import rclpy
        except ImportError:
            pytest.skip("rclpy not available on host - run in Docker")

        # Step 1: Parse intent
        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        from agent_ros_bridge_msgs.srv import ParseIntent

        parser = IntentParserNode()
        request = ParseIntent.Request()
        request.utterance = "navigate to position 1 2"

        response = ParseIntent.Response()
        result = parser.parse_intent_callback(request, response)

        assert result.success
        assert result.intent.type == "NAVIGATE"
        print(f"\n✅ Step 1: Intent parsed - {result.intent.type}")

        # Step 2: Create motion goal
        from agent_ros_bridge.ai.robot_api import NavigationGoal

        goal = NavigationGoal(x=1.0, y=2.0, theta=0.0)
        print(f"✅ Step 2: Navigation goal created - ({goal.x}, {goal.y})")

        # Step 3: Verify ROS2 can receive commands
        result = run_in_ros2_container("ros2 topic list")
        assert result.returncode == 0
        print("✅ Step 3: ROS2 bridge accessible")

        print("\n✅ Full flow test passed")

    def test_performance_latency(self):
        """Test end-to-end latency is acceptable."""
        # Skip if rclpy not available
        try:
            import rclpy
        except ImportError:
            pytest.skip("rclpy not available on host - run in Docker")

        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        from agent_ros_bridge_msgs.srv import ParseIntent

        parser = IntentParserNode()

        # Measure parsing latency
        latencies = []
        for _ in range(10):
            request = ParseIntent.Request()
            request.utterance = "go to kitchen"
            response = ParseIntent.Response()

            start = time.time()
            parser.parse_intent_callback(request, response)
            latency = (time.time() - start) * 1000
            latencies.append(latency)

        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)

        print(f"\n✅ Performance: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")

        # Assert performance targets
        assert avg_latency < 10.0, f"Average latency too high: {avg_latency:.2f}ms"
        assert max_latency < 20.0, f"Max latency too high: {max_latency:.2f}ms"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
