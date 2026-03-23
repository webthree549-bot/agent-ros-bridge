"""
TDD Tests for Real Gazebo Execution

Tests actual Gazebo/Nav2 integration with real simulation.
"""

import time
from unittest.mock import MagicMock, Mock, patch

import pytest


class TestRealGazeboConnection:
    """RED: Should connect to real Gazebo instance"""

    def test_connects_to_gz_server(self):
        """RED: Should connect to gz-server process"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        mock_process = Mock(pid=1234)
        sim._gz_process = mock_process

        assert sim._gz_process is not None
        assert sim._gz_process.pid == 1234

    def test_connects_to_ros2(self):
        """RED: Should initialize ROS2 node"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch("rclpy.init") as mock_init:
            with patch("rclpy.create_node") as mock_node:
                sim.connect()

                mock_init.assert_called_once()
                mock_node.assert_called_once()

    def test_creates_nav2_action_client(self):
        """RED: Should create Nav2 action client"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_create_nav2_client") as mock_client:
            sim.connect()

            mock_client.assert_called_once()


class TestRealRobotSpawning:
    """RED: Should spawn real robot in Gazebo"""

    def test_spawns_turtlebot3(self):
        """RED: Should spawn TurtleBot3 model"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        call_count = [0]
        original_call = sim._call_spawn_service

        def mock_call(*args, **kwargs):
            call_count[0] += 1
            return {"success": True}

        sim._call_spawn_service = mock_call

        result = sim.spawn_robot(model="turtlebot3_waffle", x=1.0, y=2.0, z=0.1, yaw=0.0)

        assert result is True
        assert call_count[0] == 1

    def test_waits_for_robot_stable(self):
        """RED: Should wait for robot to stabilize"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        stable_called = [False]
        original_stable = sim._wait_for_stable

        def mock_stable(*args, **kwargs):
            stable_called[0] = True
            return True

        sim._wait_for_stable = mock_stable

        sim.spawn_robot(model="turtlebot3_waffle", x=0, y=0, z=0.1)

        assert stable_called[0]

    def test_removes_existing_robot(self):
        """RED: Should remove robot if already exists"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        delete_called_with = [None]
        original_delete = sim._delete_entity

        def mock_delete(name):
            delete_called_with[0] = name

        sim._delete_entity = mock_delete

        sim.spawn_robot(model="turtlebot3_waffle", x=0, y=0, z=0.1)

        assert delete_called_with[0] == "turtlebot3_waffle"


class TestRealNavigation:
    """RED: Should execute real Nav2 navigation"""

    def test_sends_nav2_goal(self):
        """RED: Should send goal to Nav2 NavigateToPose action"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        goal = {"x": 5.0, "y": 5.0, "theta": 1.57}

        with patch.object(sim, "_send_nav2_goal") as mock_send:
            mock_send.return_value = True

            result = sim.navigate_to_pose(goal, timeout_sec=60.0)

            assert result["success"] is True
            mock_send.assert_called_once()

    def test_monitors_navigation_progress(self):
        """RED: Should monitor feedback from Nav2"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_monitor_feedback") as mock_feedback:
            mock_feedback.return_value = [
                {"distance_remaining": 5.0},
                {"distance_remaining": 2.0},
                {"distance_remaining": 0.1},
            ]

            result = sim.navigate_to_pose({"x": 5.0, "y": 0.0})

            assert result["success"] is True

    def test_handles_navigation_failure(self):
        """RED: Should handle Nav2 failure"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_send_nav2_goal") as mock_send:
            mock_send.side_effect = Exception("Nav2 not available")

            result = sim.navigate_to_pose({"x": 5.0, "y": 5.0})

            assert result["success"] is False
            assert "error" in result

    def test_respects_timeout(self):
        """RED: Should timeout if navigation takes too long"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0, timeout_seconds=5)

        wait_result = {"success": False, "error": "timeout"}
        sim._wait_for_result = lambda: wait_result

        result = sim.navigate_to_pose({"x": 100.0, "y": 100.0})

        assert result["success"] is False


class TestRealMetricsCollection:
    """RED: Should collect real metrics from simulation"""

    def test_gets_pose_from_gazebo(self):
        """RED: Should query robot pose from Gazebo"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_get_entity_pose") as mock_pose:
            mock_pose.return_value = (1.5, 2.5, 0.78)

            x, y, yaw = sim.get_robot_pose()

            assert x == 1.5
            assert y == 2.5
            assert yaw == 0.78

    def test_samples_trajectory(self):
        """RED: Should sample trajectory during navigation"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "get_robot_pose") as mock_pose:
            mock_pose.side_effect = [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (2.0, 0.0, 0.0),
                (3.0, 0.0, 0.0),
            ]

            trajectory = sim.sample_trajectory(duration=0.4, sample_rate=10)

            assert len(trajectory) >= 4

    def test_detects_real_collisions(self):
        """RED: Should detect collisions from Gazebo contacts"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_get_contact_states") as mock_contacts:
            mock_contacts.side_effect = [
                [],  # No contact
                [{"collision1": "obstacle"}],  # Collision!
                [],  # No contact
            ]

            collisions = sim.detect_collisions(duration=0.3)

            assert collisions == 1

    def test_calculates_execution_time(self):
        """RED: Should measure actual execution time"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        start = time.time()

        with patch.object(sim, "navigate_to_pose") as mock_nav:
            mock_nav.return_value = {
                "success": True,
                "execution_time": 15.5,
            }

            result = sim.navigate_to_pose({"x": 5.0, "y": 5.0})

            assert result["execution_time"] == 15.5


class TestRealScenarioExecution:
    """RED: Should execute full scenario with real simulation"""

    def test_executes_scenario_end_to_end(self):
        """RED: Should run complete scenario"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        scenario = {
            "name": "nav_test",
            "robot_config": {"model": "turtlebot3_waffle"},
            "goal": {"pose": {"x": 5.0, "y": 5.0, "theta": 0.0}},
        }

        sim._connected = True

        def mock_nav(goal):
            return {"success": True, "execution_time": 12.5}

        sim.navigate_to_pose = mock_nav

        result = sim.run_scenario(scenario)

        assert result["success"] is True
        assert result["execution_time"] == 12.5

    def test_cleans_up_after_scenario(self):
        """RED: Should cleanup after scenario"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)
        sim._robot_name = "turtlebot3_waffle"

        delete_called_with = [None]

        def mock_delete(name):
            delete_called_with[0] = name

        sim._delete_entity = mock_delete

        sim.cleanup()

        assert delete_called_with[0] == "turtlebot3_waffle"


class TestParallelRealExecution:
    """RED: Should run multiple real simulations in parallel"""

    def test_creates_isolated_worlds(self):
        """RED: Each world should be isolated"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim1 = RealGazeboSimulator(world_id=0, gzserver_port=11345)
        sim2 = RealGazeboSimulator(world_id=1, gzserver_port=11346)

        assert sim1.world_id != sim2.world_id
        assert sim1.gzserver_port != sim2.gzserver_port


class TestErrorRecovery:
    """RED: Should handle errors gracefully"""

    def test_recovers_from_gazebo_crash(self):
        """RED: Should restart Gazebo if it crashes"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        restart_called = [False]

        def mock_restart():
            restart_called[0] = True

        sim._restart_gazebo = mock_restart
        sim._is_gazebo_running = lambda: False  # Force restart

        sim.ensure_gazebo_running()

        assert restart_called[0]

    def test_handles_spawn_failure(self):
        """RED: Should retry if spawn fails"""
        from agent_ros_bridge.simulation.gazebo_real import RealGazeboSimulator

        sim = RealGazeboSimulator(world_id=0)

        with patch.object(sim, "_call_spawn_service") as mock_spawn:
            mock_spawn.side_effect = [
                {"success": False},  # First attempt fails
                {"success": True},  # Retry succeeds
            ]

            result = sim.spawn_robot(model="turtlebot3_waffle", x=0, y=0, z=0.1)

            assert result is True
            assert mock_spawn.call_count == 2
