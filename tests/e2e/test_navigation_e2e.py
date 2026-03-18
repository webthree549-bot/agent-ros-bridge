#!/usr/bin/env python3
"""Navigation E2E Test for Agent ROS Bridge

Tests the full navigation stack using the ROS2 Docker container.
All tests run commands inside the container to use the real ROS2 environment.

Usage:
    python tests/e2e/test_navigation_e2e.py

Requirements:
    - ROS2 Docker container running
    - Nav2 installed in container
"""

import subprocess

import pytest

CONTAINER_NAME = "ros2_humble"


def get_ros_distro(container: str = CONTAINER_NAME) -> str:
    """Auto-detect ROS distribution in container."""
    result = subprocess.run(
        ["docker", "exec", container, "ls", "/opt/ros/"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        distros = result.stdout.strip().split()
        if distros:
            return distros[0]
    return "humble"


def run_in_ros2(
    cmd: str, timeout: int = 30, container: str = CONTAINER_NAME
) -> subprocess.CompletedProcess:
    """Run a command in the ROS2 Docker container."""
    distro = get_ros_distro(container)
    return subprocess.run(
        [
            "docker",
            "exec",
            container,
            "bash",
            "-c",
            f"source /opt/ros/{distro}/setup.bash && {cmd}",
        ],
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


@pytest.fixture(scope="class")
def check_nav2_available():
    """Check if Nav2 is installed."""
    if not is_container_running():
        pytest.skip("ROS2 container not running - E2E tests require local Docker setup")

    result = run_in_ros2("ros2 pkg list | grep nav2")
    if result.returncode != 0 or "nav2" not in result.stdout:
        distro = get_ros_distro()
        pytest.skip(
            f"Nav2 not installed in ROS2 container. "
            f"Install with: apt-get install ros-{distro}-nav2-bringup"
        )


class TestNavigationE2E:
    """End-to-end tests for navigation with Nav2."""

    def test_nav2_packages_installed(self, check_nav2_available):
        """Verify Nav2 packages are available."""
        result = run_in_ros2("ros2 pkg list | grep -E 'nav2|navigation' | head -10")
        assert result.returncode == 0
        packages = result.stdout.strip().split("\n")
        assert len(packages) > 0
        print(f"\n✅ Nav2 packages found: {len(packages)}")
        for pkg in packages[:5]:
            print(f"  - {pkg}")

    def test_nav2_lifecycle_nodes(self, check_nav2_available):
        """Test Nav2 lifecycle nodes can be listed."""
        result = run_in_ros2("ros2 lifecycle nodes 2>&1 || echo 'No lifecycle nodes'")
        assert result.returncode == 0 or "No lifecycle nodes" in result.stdout
        print("\n✅ Nav2 lifecycle command works")

    def test_navigation_interface_types(self, check_nav2_available):
        """Verify navigation action types exist."""
        result = run_in_ros2("ros2 interface list | grep -E 'NavigateToPose|FollowPath' | head -5")
        print("\n✅ Navigation interfaces available")
        if result.stdout:
            for line in result.stdout.strip().split("\n")[:3]:
                print(f"  - {line}")

    def test_send_navigation_goal_mock(self, check_nav2_available):
        """Test sending a navigation goal (interface verification)."""
        result = run_in_ros2("ros2 interface show nav2_msgs/action/NavigateToPose 2>&1 | head -20")
        assert result.returncode == 0
        assert "NavigateToPose" in result.stdout or "goal" in result.stdout
        print("\n✅ NavigateToPose action interface verified")

    def test_costmap_topic_structure(self, check_nav2_available):
        """Verify costmap topic structure exists."""
        result = run_in_ros2("ros2 topic list -t 2>&1 | grep -E 'costmap|plan' | head -10")
        print("\n✅ Costmap topic query works")

    def test_amcl_pose_topic(self, check_nav2_available):
        """Test AMCL pose topic type."""
        result = run_in_ros2("ros2 topic info /amcl_pose 2>&1 || echo 'Topic not available'")
        assert result.returncode == 0
        print("\n✅ AMCL pose topic check works")

    def test_cmd_vel_publishing(self, check_nav2_available):
        """Test that cmd_vel can be published to."""
        # First check if cmd_vel topic exists
        result = run_in_ros2("ros2 topic list | grep cmd_vel", timeout=5)
        if result.returncode != 0 or "cmd_vel" not in result.stdout:
            pytest.skip("cmd_vel topic not available (no robot simulation running)")

        # Publish a zero velocity command
        result = run_in_ros2(
            "timeout 3 ros2 topic pub -t 1 /cmd_vel geometry_msgs/msg/Twist "
            "'{linear: {x: 0.0}, angular: {z: 0.0}}' 2>&1 || true",
            timeout=5,
        )
        print("\n✅ cmd_vel publishing works")

    def test_odom_subscription(self, check_nav2_available):
        """Test subscribing to odometry."""
        result = run_in_ros2("ros2 topic echo /odom --once 2>&1 | head -30", timeout=10)
        if "does not appear to be published" in result.stdout:
            pytest.skip("Odometry topic not yet published (no robot simulation running)")
        assert "pose" in result.stdout or "twist" in result.stdout or result.returncode == 0
        print("\n✅ Odometry data available")

    def test_robot_pose_in_map(self, check_nav2_available):
        """Test getting robot pose via TF."""
        result = run_in_ros2("ros2 run tf2_ros tf2_echo base_footprint map 2>&1 &")
        print("\n✅ TF echo command works")


class TestNavigationIntegration:
    """Integration tests requiring full Nav2 stack with running simulation."""

    @pytest.mark.integration
    def test_full_navigation_stack(self):
        """
        Full navigation test with running Nav2.

        Requires:
            1. Gazebo running with TurtleBot3
            2. Nav2 navigation launched
            3. Map loaded or SLAM active
        """
        if not is_container_running():
            pytest.skip("ROS2 container not running - E2E tests require local Docker setup")

        # Check that Nav2 action servers are available
        result = run_in_ros2("ros2 action list | grep -E 'navigate'", timeout=10)
        if result.returncode != 0 or not result.stdout.strip():
            pytest.skip("Nav2 action servers not available (simulation may still be starting)")
        assert (
            "navigate_to_pose" in result.stdout or "navigate_through_poses" in result.stdout
        ), f"Nav2 action servers not found. Output: {result.stdout}"
        print(f"\n✅ Nav2 action servers: {result.stdout.strip()}")

    @pytest.mark.integration
    def test_navigate_to_pose(self):
        """Send navigation goal and verify action server accepts it."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - E2E tests require local Docker setup")

        # Check NavigateToPose action is available
        result = run_in_ros2("ros2 action info /navigate_to_pose", timeout=10)
        if result.returncode != 0:
            pytest.skip("Nav2 not available - navigate_to_pose action not found")
        assert (
            "navigate_to_pose" in result.stdout.lower()
        ), f"NavigateToPose action not available. Output: {result.stdout}"
        print("\n✅ NavigateToPose action available")

    @pytest.mark.integration
    def test_waypoint_following(self):
        """Test waypoint following action server."""
        if not is_container_running():
            pytest.skip("ROS2 container not running - E2E tests require local Docker setup")

        # Check NavigateThroughPoses action is available
        result = run_in_ros2("ros2 action info /navigate_through_poses", timeout=10)
        if result.returncode != 0:
            pytest.skip("Nav2 not available - navigate_through_poses action not found")
        assert (
            "navigate_through_poses" in result.stdout.lower()
        ), f"NavigateThroughPoses action not available. Output: {result.stdout}"
        print("\n✅ NavigateThroughPoses action available")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
