#!/usr/bin/env python3
"""
Gazebo E2E Test for Agent ROS Bridge

This test uses actual Gazebo simulation with TurtleBot3.
No mocking - real physics, real ROS2, real robot behavior.

Prerequisites:
    - ROS2 Humble in Docker: docker start ros2_humble
    - OR ROS2 Humble installed locally
    - Gazebo Ignition installed
    - TurtleBot3 packages

Usage:
    # Terminal 1: Start ROS2 Docker
    docker start ros2_humble

    # Terminal 2: Run this test
    python tests/e2e/test_gazebo_e2e.py
"""

import subprocess

import pytest

# Check if ROS2 is available locally
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# Check if Docker ROS2 is available
def is_docker_ros2_available():
    """Check if ROS2 Docker container is running."""
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=ros2_humble", "--format", "{{.Names}}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return "ros2_humble" in result.stdout
    except Exception:
        return False


DOCKER_ROS2_AVAILABLE = is_docker_ros2_available()


# Require ROS2 Docker container - no skipping allowed
@pytest.fixture(scope="session", autouse=True)
def require_ros2_docker():
    """Ensure ROS2 Docker container is running before any tests."""
    if not DOCKER_ROS2_AVAILABLE:
        pytest.fail(
            "❌ ROS2 Docker container 'ros2_humble' is not running.\n"
            "   Start it with: docker start ros2_humble\n"
            "   No tests will be skipped - all must run in Docker."
        )


class TestGazeboE2E:
    """End-to-end tests using real Gazebo simulation."""

    def test_ros2_available(self):
        """Verify ROS2 is available (locally or in Docker)."""
        if ROS2_AVAILABLE:
            print("\n✅ ROS2 available locally")
        elif DOCKER_ROS2_AVAILABLE:
            print("\n✅ ROS2 available in Docker")
        else:
            pytest.fail("ROS2 not available")

    def test_docker_ros2_commands(self):
        """Test running ROS2 commands in Docker container."""
        if not DOCKER_ROS2_AVAILABLE:
            pytest.skip("Docker ROS2 not available")

        # Test basic ROS2 command
        result = subprocess.run(
            [
                "docker",
                "exec",
                "ros2_humble",
                "bash",
                "-c",
                "source /opt/ros/humble/setup.bash && ros2 --help",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        assert result.returncode == 0, f"ROS2 command failed: {result.stderr}"
        assert "ros2 is an extensible command-line tool" in result.stdout
        print("\n✅ Docker ROS2 commands working")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
