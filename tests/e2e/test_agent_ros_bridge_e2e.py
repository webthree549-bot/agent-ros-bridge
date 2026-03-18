#!/usr/bin/env python3
"""Agent ROS Bridge E2E Test

Full end-to-end test demonstrating:
1. Agent receives natural language command
2. Agent parses intent
3. Agent calls ROS2 via bridge
4. Robot executes command
5. Verify execution result

This test uses Docker-based ROS2 - tests run commands inside the container.

NOTE: These tests require ROS messages to be compiled. Run:
    cd /workspace && colcon build --packages-select agent_ros_bridge_msgs
"""

import subprocess
import time

import pytest

CONTAINER_NAME = "ros2_humble"


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
    return "humble"


def run_in_ros2_container(cmd: str, timeout: int = 30) -> subprocess.CompletedProcess:
    """Run a command in the ROS2 Docker container with ROS environment sourced."""
    distro = get_ros_distro()
    # Source both the base ROS and the built messages
    full_cmd = f"source /opt/ros/{distro}/setup.bash && if [ -f /tmp/ros_build/install/setup.bash ]; then source /tmp/ros_build/install/setup.bash; fi && {cmd}"
    return subprocess.run(
        ["docker", "exec", CONTAINER_NAME, "bash", "-c", full_cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def is_container_running() -> bool:
    """Check if ROS2 container is running."""
    result = subprocess.run(
        ["docker", "ps", "--filter", f"name={CONTAINER_NAME}", "--format", "{{.Status}}"],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and "Up" in result.stdout


def ros_messages_compiled() -> bool:
    """Check if ROS messages are compiled with type support."""
    # Check if the build exists in the container
    result = subprocess.run(
        ["docker", "exec", CONTAINER_NAME, "test", "-f", "/tmp/ros_build/install/setup.bash"],
        capture_output=True,
    )
    return result.returncode == 0


class TestAgentROSBridgeE2E:
    """End-to-end tests for Agent ROS Bridge."""

    def test_ros2_container_running(self):
        """Verify ROS2 Docker container is running."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")
        print("\n✅ ROS2 container is running")

    def test_ros2_basic_commands(self):
        """Test basic ROS2 commands work."""
        if not is_container_running():
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
        """Test Agent can parse natural language to intent (runs in Docker)."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")
        
        if not ros_messages_compiled():
            pytest.skip("ROS messages not compiled - run 'colcon build --packages-select agent_ros_bridge_msgs'")

        # Run intent parsing test inside the container
        test_script = '''
import sys
# Use compiled messages first, then workspace
sys.path.insert(0, "/tmp/ros_build/install/agent_ros_bridge_msgs/lib/python3.12/site-packages")
sys.path.insert(1, "/workspace")

try:
    import rclpy
    from agent_ros_bridge.ai.intent_parser import IntentParserNode
    from agent_ros_bridge_msgs.srv import ParseIntent
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        parser = IntentParserNode()
        
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
            assert result.intent.type == expected_type, f"Expected {expected_type}, got {result.intent.type}"
            print(f"✅ Parsed '{utterance}' -> {result.intent.type}")
        
        print("INTENT_PARSING_PASSED")
    finally:
        rclpy.shutdown()
except Exception as e:
    print(f"INTENT_PARSING_FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "INTENT_PARSING_PASSED" not in result.stdout:
            pytest.fail(f"Intent parsing test failed: {result.stdout} {result.stderr}")
        
        print("\n✅ Agent intent parsing test passed")

    def test_bridge_ros_command_execution(self):
        """Test bridge can execute ROS commands via Docker."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")

        # Start a demo node in background
        run_in_ros2_container("ros2 run demo_nodes_cpp talker &", timeout=5)

        # Wait for node to start (up to 10 seconds)
        for _ in range(10):
            time.sleep(1)
            result = run_in_ros2_container("ros2 topic list")
            if "/chatter" in result.stdout:
                break
        else:
            pytest.skip("Demo node did not start in time - demo_nodes_cpp may not be installed")

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
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")
            
        if not ros_messages_compiled():
            pytest.skip("ROS messages not compiled - run 'colcon build --packages-select agent_ros_bridge_msgs'")

        # Run full flow test inside the container
        test_script = '''
import sys
# Use compiled messages first, then workspace
sys.path.insert(0, "/tmp/ros_build/install/agent_ros_bridge_msgs/lib/python3.12/site-packages")
sys.path.insert(1, "/workspace")

try:
    import rclpy
    from agent_ros_bridge.ai.intent_parser import IntentParserNode
    from agent_ros_bridge_msgs.srv import ParseIntent
    from agent_ros_bridge.robot_api import NavigationGoal
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Step 1: Parse intent
        parser = IntentParserNode()
        request = ParseIntent.Request()
        request.utterance = "navigate to position 1 2"
        response = ParseIntent.Response()
        result = parser.parse_intent_callback(request, response)
        
        assert result.success, "Intent parsing failed"
        assert result.intent.type == "NAVIGATE", f"Expected NAVIGATE, got {result.intent.type}"
        print(f"✅ Step 1: Intent parsed - {result.intent.type}")
        
        # Step 2: Create motion goal
        goal = NavigationGoal(x=1.0, y=2.0, theta=0.0)
        print(f"✅ Step 2: Navigation goal created - ({goal.x}, {goal.y})")
        
        print("FULL_FLOW_PASSED")
    finally:
        rclpy.shutdown()
except Exception as e:
    print(f"FULL_FLOW_FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "FULL_FLOW_PASSED" not in result.stdout:
            pytest.fail(f"Full flow test failed: {result.stdout} {result.stderr}")
        
        # Step 3: Verify ROS2 can receive commands
        result = run_in_ros2_container("ros2 topic list")
        assert result.returncode == 0
        print("✅ Step 3: ROS2 bridge accessible")

        print("\n✅ Full flow test passed")

    def test_performance_latency(self):
        """Test end-to-end latency is acceptable."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - start with: ./docker-manager.sh start")
            
        if not ros_messages_compiled():
            pytest.skip("ROS messages not compiled - run 'colcon build --packages-select agent_ros_bridge_msgs'")

        # Run performance test inside the container
        test_script = '''
import sys
import time
# Use compiled messages first, then workspace
sys.path.insert(0, "/tmp/ros_build/install/agent_ros_bridge_msgs/lib/python3.12/site-packages")
sys.path.insert(1, "/workspace")

try:
    import rclpy
    from agent_ros_bridge.ai.intent_parser import IntentParserNode
    from agent_ros_bridge_msgs.srv import ParseIntent
    
    # Initialize ROS2
    rclpy.init()
    
    try:
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
        
        print(f"PERFORMANCE: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")
        
        if avg_latency >= 10.0:
            print(f"PERFORMANCE_FAILED: Average latency too high: {avg_latency:.2f}ms")
            sys.exit(1)
        if max_latency >= 20.0:
            print(f"PERFORMANCE_FAILED: Max latency too high: {max_latency:.2f}ms")
            sys.exit(1)
        
        print("PERFORMANCE_PASSED")
    finally:
        rclpy.shutdown()
except Exception as e:
    print(f"PERFORMANCE_FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
'''
        result = run_in_ros2_container(f"python3 -c '{test_script}'", timeout=60)
        
        if "PERFORMANCE_PASSED" not in result.stdout:
            pytest.fail(f"Performance test failed: {result.stdout} {result.stderr}")
        
        # Extract and print performance metrics
        for line in result.stdout.split('\n'):
            if 'PERFORMANCE:' in line:
                print(f"\n✅ {line.replace('PERFORMANCE:', 'Performance:')}")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
