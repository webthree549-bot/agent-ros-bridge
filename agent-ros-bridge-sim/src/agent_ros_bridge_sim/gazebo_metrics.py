"""Metrics collection for Gazebo batch simulation.

Handles trajectory tracking, collision detection, and deviation calculation.
"""

import logging
import math
import os
import subprocess  # nosec B404 - Required for Gazebo/ROS2 queries
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gazebo_batch_runner import GazeboBatchRunner

logger = logging.getLogger(__name__)


class MetricsCollector:
    """Collects and calculates execution metrics for Gazebo worlds."""

    def __init__(self, runner: "GazeboBatchRunner"):
        """Initialize metrics collector.

        Args:
            runner: Reference to the batch runner for accessing world state
        """
        self.runner = runner

    def collect_trajectory(
        self,
        world_id: int,
        duration_sec: float,
        sample_rate_hz: float = 10.0,
    ) -> list[tuple[float, float, float]]:
        """Collect robot trajectory during execution.

        Args:
            world_id: World to collect from
            duration_sec: Duration to collect for
            sample_rate_hz: Sampling rate in Hz

        Returns:
            List of (x, y, yaw) poses
        """
        trajectory = []
        num_samples = int(duration_sec * sample_rate_hz)
        sample_interval = 1.0 / sample_rate_hz

        for _ in range(num_samples):
            pose = self._get_robot_pose(world_id)
            if pose:
                trajectory.append(pose)
            time.sleep(sample_interval)

        return trajectory

    def _get_robot_pose(self, world_id: int) -> tuple[float, float, float] | None:
        """Get current robot pose from simulation.

        Args:
            world_id: World to query

        Returns:
            (x, y, yaw) or None if not available
        """
        # Delegate to ground truth pose retrieval
        return self._get_ground_truth_pose(world_id)

    def _get_ground_truth_pose(self, world_id: int) -> tuple[float, float, float] | None:
        """Get ground truth pose from simulation.

        Args:
            world_id: World to query

        Returns:
            (x, y, yaw) or None if not available
        """
        # Query Gazebo for robot pose
        world = self._get_world(world_id)
        if not world or not world.is_running:
            return None

        try:
            # Use gz service to get entity state
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/default/state",
                "--reqtype",
                "gz.msgs.WorldState",
                "--reptype",
                "gz.msgs.WorldState",
                "--timeout",
                "1000",
            ]

            result = subprocess.run(  # nosec B603 B607
                cmd,
                capture_output=True,
                text=True,
                env={**os.environ, "GZ_PARTITION": f"sim_{world_id}"},
            )

            if result.returncode == 0:
                # Parse pose from response (mock for now)
                return (0.0, 0.0, 0.0)

        except Exception as e:
            logger.debug(f"Could not get ground truth pose: {e}")

        return None

    def _get_world(self, world_id: int):
        """Get world config by ID."""
        for world in self.runner.worlds:
            if world.world_id == world_id:
                return world
        return None

    def count_collisions(self, world_id: int, duration: float) -> int:
        """Count collisions during execution.

        Args:
            world_id: World to monitor
            duration: Duration to monitor for

        Returns:
            Number of collisions detected
        """
        collision_count = 0
        check_interval = 0.1  # 10 Hz
        num_checks = int(duration / check_interval)

        for _ in range(num_checks):
            if self._check_collision(world_id):
                collision_count += 1
            time.sleep(check_interval)

        return collision_count

    def _check_collision(self, world_id: int) -> bool:
        """Check if robot is in collision state.

        Args:
            world_id: World to check

        Returns:
            True if collision detected
        """
        world = self._get_world(world_id)
        if not world or not world.is_running:
            return False

        try:
            # Query Gazebo for contact state
            cmd = [
                "gz",
                "service",
                "-s",
                "/world/default/state",
                "--reqtype",
                "gz.msgs.WorldState",
                "--reptype",
                "gz.msgs.WorldState",
                "--timeout",
                "100",
            ]

            result = subprocess.run(  # nosec B603 B607
                cmd,
                capture_output=True,
                text=True,
                env={**os.environ, "GZ_PARTITION": f"sim_{world_id}"},
            )

            if result.returncode == 0:
                # Check for contacts in response (mock for now)
                return False

        except Exception as e:
            logger.debug(f"Could not check collision: {e}")

        return False

    def get_planned_path(self, world_id: int) -> list[tuple[float, float]]:
        """Get planned path from Nav2.

        Args:
            world_id: World to query

        Returns:
            List of (x, y) waypoints
        """
        world = self._get_world(world_id)
        if not world or not world.is_running:
            return []

        try:
            # Query Nav2 for planned path
            # This would use ROS2 to get the path from the planner
            result = subprocess.run(  # nosec B603 B607
                [
                    "ros2",
                    "topic",
                    "echo",
                    "/plan",
                    "--once",
                    "--timeout",
                    "1",
                ],
                capture_output=True,
                text=True,
                env={**os.environ, "ROS_NAMESPACE": world.ros_namespace},
            )

            if result.returncode == 0:
                # Parse path from output
                return self._parse_path_from_ros_output(result.stdout)

        except Exception as e:
            logger.debug(f"Could not get planned path: {e}")

        return []

    def _parse_path_from_ros_output(self, output: str) -> list[tuple[float, float]]:
        """Parse path from ROS topic echo output."""
        # Mock implementation - would parse actual ROS message
        return []

    def calculate_deviation(
        self,
        trajectory: list[tuple[float, float, float]],
        planned_path: list[tuple[float, float]],
    ) -> float:
        """Calculate maximum deviation from planned path.

        Args:
            trajectory: Actual executed trajectory
            planned_path: Planned path from planner

        Returns:
            Maximum deviation in meters
        """
        if not trajectory or not planned_path:
            return 0.0

        max_deviation = 0.0

        for pose in trajectory:
            # Find closest point on planned path
            min_dist = float("inf")
            for path_point in planned_path:
                dist = math.sqrt((pose[0] - path_point[0]) ** 2 + (pose[1] - path_point[1]) ** 2)
                min_dist = min(min_dist, dist)

            max_deviation = max(max_deviation, min_dist)

        return max_deviation
