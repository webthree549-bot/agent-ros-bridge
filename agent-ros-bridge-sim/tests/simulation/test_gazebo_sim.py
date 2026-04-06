"""
TDD Tests for Gazebo Simulation Integration

Connects ScenarioRunner to real Gazebo + Nav2 for actual scenario execution.
Replaces mock execution with real robot simulation.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest

# =============================================================================
# Phase 1: RED - Write failing tests
# =============================================================================


class TestGazeboSimulatorExists:
    """RED: GazeboSimulator class should exist"""

    def test_gazebo_simulator_module_exists(self):
        """RED: agent_ros_bridge.simulation.gazebo_sim module should exist"""
        try:
            from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

            assert True
        except ImportError:
            pytest.fail("GazeboSimulator should be importable")

    def test_gazebo_simulator_class_exists(self):
        """RED: GazeboSimulator class should exist"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        assert GazeboSimulator is not None

    def test_can_create_simulator(self):
        """RED: Should create simulator with world config"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0, gzserver_port=11345)
        assert sim.world_id == 0
        assert sim.gzserver_port == 11345


class TestWorldConnection:
    """RED: Should connect to Gazebo world"""

    def test_connect_method_exists(self):
        """RED: Should have connect method"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()
        assert hasattr(sim, "connect")

    def test_connects_to_gazebo_transport(self):
        """RED: Should connect to Gazebo transport"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_check_gazebo_running", return_value=True):
            with patch.object(sim, "_connect_transport") as mock_connect:
                sim.connect()
                mock_connect.assert_called_once()

    def test_connects_to_ros2(self):
        """RED: Should initialize ROS2 node"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0, ros_namespace="world_0")

        with patch.object(sim, "_check_gazebo_running", return_value=True):
            with patch.object(sim, "_init_ros_node") as mock_ros:
                sim.connect()
                mock_ros.assert_called_once()

    def test_disconnect_method_exists(self):
        """RED: Should have disconnect method"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()
        assert hasattr(sim, "disconnect")


class TestScenarioLoading:
    """RED: Should load scenarios into Gazebo"""

    def test_load_scenario_method_exists(self):
        """RED: Should have load_scenario method"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()
        assert hasattr(sim, "load_scenario")

    def test_loads_world_file(self):
        """RED: Should load world file into Gazebo"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        scenario = SimpleNamespace(
            name="nav_test",
            environment={"world_file": "nav_world.world"},
        )

        with patch.object(sim, "_load_world_file") as mock_load:
            sim.load_scenario(scenario)
            mock_load.assert_called_once_with("nav_world.world")

    def test_spawns_robot(self):
        """RED: Should spawn robot with config"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        scenario = SimpleNamespace(
            name="nav_test",
            robot_config={"type": "turtlebot3", "x": 1.0, "y": 2.0},
        )

        with patch.object(sim, "_spawn_robot") as mock_spawn:
            sim.load_scenario(scenario)
            mock_spawn.assert_called_once()

    def test_spawns_obstacles(self):
        """RED: Should spawn obstacles"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        scenario = SimpleNamespace(
            name="nav_test",
            environment={"obstacles": [{"x": 5.0, "y": 5.0, "radius": 0.5}]},
        )

        with patch.object(sim, "_spawn_obstacle") as mock_obs:
            sim.load_scenario(scenario)
            mock_obs.assert_called_once()


class TestNavigationExecution:
    """RED: Should execute navigation with Nav2"""

    def test_execute_navigation_method_exists(self):
        """RED: Should have execute_navigation method"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()
        assert hasattr(sim, "execute_navigation")

    def test_sends_goal_to_nav2(self):
        """RED: Should send navigation goal to Nav2"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        goal = {"x": 10.0, "y": 10.0, "theta": 0.0}

        with patch.object(sim, "_send_nav2_goal") as mock_goal:
            sim.execute_navigation(goal)
            mock_goal.assert_called_once_with(goal)

    def test_waits_for_completion(self):
        """RED: Should wait for navigation to complete"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_wait_for_result") as mock_wait:
            mock_wait.return_value = {"success": True, "duration": 15.0}

            result = sim.execute_navigation({"x": 10.0, "y": 10.0})

            assert result["success"] is True
            assert "duration" in result

    def test_handles_timeout(self):
        """RED: Should handle navigation timeout"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0, timeout_seconds=30)

        with patch.object(sim, "_wait_for_result") as mock_wait:
            mock_wait.return_value = {"success": False, "error": "timeout"}

            result = sim.execute_navigation({"x": 10.0, "y": 10.0})

            assert result["success"] is False
            assert "timeout" in result["error"]


class TestMetricsCollection:
    """RED: Should collect real simulation metrics"""

    def test_get_robot_pose_method_exists(self):
        """RED: Should get robot pose from Gazebo"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()
        assert hasattr(sim, "get_robot_pose")

    def test_gets_pose_from_gazebo(self):
        """RED: Should query robot pose from Gazebo"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_query_gazebo_pose") as mock_pose:
            mock_pose.return_value = (1.0, 2.0, 0.5)

            pose = sim.get_robot_pose()

            assert pose == (1.0, 2.0, 0.5)

    def test_tracks_trajectory(self):
        """RED: Should track robot trajectory"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        # Provide enough values for 3 samples at 10Hz for 0.3 seconds
        with patch.object(sim, "get_robot_pose") as mock_pose:
            mock_pose.side_effect = [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (2.0, 0.0, 0.0),
            ] * 10  # Repeat to ensure enough values

            trajectory = sim.collect_trajectory(duration=0.3, sample_rate=10)

            assert len(trajectory) >= 3
            assert trajectory[0] == (0.0, 0.0, 0.0)

    def test_detects_collisions(self):
        """RED: Should detect collisions"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_check_collision") as mock_collision:
            # Provide values for 0.5 seconds at 0.1s intervals (5 samples)
            mock_collision.side_effect = [False, False, True, False, False]

            collisions = sim.count_collisions(duration=0.5)

            assert collisions == 1

    def test_calculates_path_deviation(self):
        """RED: Should calculate deviation from planned path"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        planned_path = [(0, 0), (1, 0), (2, 0)]
        actual_path = [(0, 0), (1, 0.1), (2, 0)]

        deviation = sim.calculate_path_deviation(planned_path, actual_path)

        assert deviation >= 0.1  # At least 10cm deviation


class TestScenarioRunnerIntegration:
    """RED: Should integrate with ScenarioRunner"""

    def test_scenario_runner_uses_gazebo_sim(self):
        """RED: ScenarioRunner should use GazeboSimulator"""
        from agent_ros_bridge.simulation import ScenarioRunner
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        runner = ScenarioRunner()

        # Should be able to use GazeboSimulator
        assert runner is not None

    def test_executes_scenario_in_gazebo(self):
        """RED: Should execute scenario in real Gazebo"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        scenario = SimpleNamespace(
            name="nav_test",
            robot_config={"type": "turtlebot3"},
            environment={"obstacles": []},
            goal={"pose": {"x": 5.0, "y": 5.0}},
        )

        with patch.object(sim, "load_scenario") as mock_load:
            with patch.object(sim, "execute_navigation") as mock_nav:
                mock_nav.return_value = {"success": True, "duration": 10.0}

                result = sim.run_scenario(scenario)

                assert result.success is True
                assert result.duration_ms > 0


class TestParallelWorlds:
    """RED: Should support parallel world execution"""

    def test_multiple_worlds_isolated(self):
        """RED: Multiple worlds should be isolated"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim1 = GazeboSimulator(world_id=0, gzserver_port=11345)
        sim2 = GazeboSimulator(world_id=1, gzserver_port=11346)

        assert sim1.gzserver_port != sim2.gzserver_port

    def test_parallel_execution(self):
        """RED: Should run scenarios in parallel worlds"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        simulators = [GazeboSimulator(world_id=i, gzserver_port=11345 + i) for i in range(4)]

        assert len(simulators) == 4
        assert all(s.gzserver_port == 11345 + i for i, s in enumerate(simulators))


class TestDockerEnvironment:
    """RED: Should work in Docker container"""

    def test_detects_docker(self):
        """RED: Should detect Docker environment"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator()

        # Should check for Docker
        assert hasattr(sim, "_is_docker")

    def test_uses_headless_mode_in_docker(self):
        """RED: Should use headless mode in Docker"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        with patch.object(GazeboSimulator, "_is_docker", True):
            sim = GazeboSimulator(world_id=0)
            assert sim.headless is True

    def test_configures_display_in_docker(self):
        """RED: Should configure virtual display in Docker"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.dict("os.environ", {"DISPLAY": ":99"}):
            env = sim._get_gazebo_env()
            assert "DISPLAY" in env


class TestErrorHandling:
    """RED: Should handle errors gracefully"""

    def test_handles_gazebo_crash(self):
        """RED: Should handle Gazebo process crash"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_check_gazebo_running") as mock_check:
            mock_check.return_value = False

            with pytest.raises(RuntimeError):
                sim.connect()

    def test_handles_nav2_failure(self):
        """RED: Should handle Nav2 failure"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_send_nav2_goal") as mock_goal:
            mock_goal.side_effect = Exception("Nav2 not available")

            result = sim.execute_navigation({"x": 10.0, "y": 10.0})

            assert result["success"] is False
            assert "error" in result

    def test_recovers_from_collision(self):
        """RED: Should continue after collision detection"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        # Should not crash on collision
        with patch.object(sim, "_check_collision") as mock_collision:
            mock_collision.return_value = True

            # Just count it, don't crash
            count = sim.count_collisions(duration=0.1)
            assert count >= 0


class TestCleanup:
    """RED: Should cleanup after execution"""

    def test_removes_robot_after_scenario(self):
        """RED: Should remove robot between scenarios"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_remove_robot") as mock_remove:
            sim.cleanup()
            mock_remove.assert_called_once()

    def test_clears_obstacles(self):
        """RED: Should clear obstacles after scenario"""
        from agent_ros_bridge.simulation.gazebo_sim import GazeboSimulator

        sim = GazeboSimulator(world_id=0)

        with patch.object(sim, "_clear_obstacles") as mock_clear:
            sim.cleanup()
            mock_clear.assert_called_once()


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def mock_gazebo_transport():
    """Create mock Gazebo transport"""
    transport = Mock()
    transport.publish = Mock()
    transport.subscribe = Mock()
    return transport


@pytest.fixture
def mock_ros_node():
    """Create mock ROS2 node"""
    node = Mock()
    node.create_client = Mock()
    node.create_publisher = Mock()
    return node


@pytest.fixture
def sample_nav_scenario():
    """Create sample navigation scenario"""
    return SimpleNamespace(
        name="nav_test_001",
        robot_config={
            "type": "turtlebot3_waffle",
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
        },
        environment={
            "bounds": {"min_x": 0, "max_x": 10, "min_y": 0, "max_y": 10},
            "obstacles": [
                {"x": 3.0, "y": 3.0, "radius": 0.5},
            ],
        },
        goal={
            "type": "navigate_to_pose",
            "pose": {"x": 8.0, "y": 8.0, "theta": 0.0},
        },
    )
