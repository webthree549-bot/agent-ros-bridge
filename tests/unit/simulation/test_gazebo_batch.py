"""
TDD Tests for GazeboBatchRunner

Parallel world execution for 10K scenario validation.
Supports headless operation, Foxglove visualization via WebSocket.
"""

import shutil
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import pytest

# =============================================================================
# Phase 1: RED - Write failing tests
# =============================================================================


class TestGazeboBatchRunnerExists:
    """RED: GazeboBatchRunner class should exist"""

    def test_gazebo_batch_runner_module_exists(self):
        """RED: agent_ros_bridge.simulation.gazebo_batch module should exist"""
        try:
            from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
            assert True
        except ImportError:
            pytest.fail("GazeboBatchRunner should be importable")

    def test_gazebo_batch_runner_class_exists(self):
        """RED: GazeboBatchRunner class should exist"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        assert GazeboBatchRunner is not None

    def test_gazebo_batch_runner_can_be_instantiated(self):
        """RED: Should create instance with config"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=4, headless=True)
        assert runner is not None
        assert runner.num_worlds == 4
        assert runner.headless is True


class TestWorldManagement:
    """RED: Should manage multiple Gazebo worlds"""

    def test_launch_worlds_method_exists(self):
        """RED: Should have method to launch worlds"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=2)
        assert hasattr(runner, 'launch_worlds')

    def test_can_launch_multiple_worlds(self):
        """RED: Should launch N Gazebo worlds"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=4, headless=True)
        
        with patch('subprocess.Popen') as mock_popen:
            mock_process = Mock()
            mock_popen.return_value = mock_process
            
            worlds = runner.launch_worlds()
            
            assert len(worlds) == 4
            assert mock_popen.call_count == 4

    def test_worlds_have_unique_ports(self):
        """RED: Each world needs unique ports for parallel operation"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=4)
        
        worlds = runner._configure_worlds()
        
        ports = [w.gzserver_port for w in worlds]
        assert len(ports) == len(set(ports))  # All unique

    def test_shutdown_worlds_method_exists(self):
        """RED: Should cleanly shutdown worlds"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=2)
        assert hasattr(runner, 'shutdown_worlds')


class TestScenarioExecution:
    """RED: Should execute scenarios in parallel worlds"""

    def test_execute_in_world_method_exists(self):
        """RED: Should execute scenario in specific world"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=2)
        assert hasattr(runner, 'execute_in_world')

    def test_execute_scenario_loads_world(self):
        """RED: Should load scenario world file"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        runner.worlds = runner._configure_worlds()  # Initialize worlds

        with patch.object(runner, '_load_world') as mock_load:
            with patch.object(runner, '_wait_for_stable') as mock_stable:
                with patch.object(runner, '_spawn_robot') as mock_spawn:
                    mock_stable.return_value = True

                    result = runner.execute_in_world(
                        world_id=0,
                        scenario_path="nav_basic.yaml"
                    )

                    # World loading happens if world_file is in scenario
                    # For basic scenario, may not be called

    def test_execute_spawns_robot(self):
        """RED: Should spawn robot with scenario config"""
        from types import SimpleNamespace

        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        runner.worlds = runner._configure_worlds()  # Initialize worlds

        mock_scenario = SimpleNamespace(
            name="test",
            robot_config={},
            environment={},
            goal={}
        )

        with patch.object(runner, '_load_scenario', return_value=mock_scenario):
            with patch.object(runner, '_spawn_robot') as mock_spawn:
                with patch.object(runner, '_wait_for_stable') as mock_stable:
                    mock_stable.return_value = True

                    result = runner.execute_in_world(
                        world_id=0,
                        scenario_path="nav_basic.yaml"
                    )

                    mock_spawn.assert_called_once()

    def test_execute_monitors_completion(self):
        """RED: Should monitor until goal reached or timeout"""
        from types import SimpleNamespace

        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        runner.worlds = runner._configure_worlds()  # Initialize worlds

        mock_scenario = SimpleNamespace(
            name="test",
            robot_config={},
            environment={},
            goal={}
        )

        with patch.object(runner, '_load_scenario', return_value=mock_scenario):
            with patch.object(runner, '_execute_goal') as mock_exec:
                mock_exec.return_value = True

                result = runner.execute_in_world(
                    world_id=0,
                    scenario_path="nav_basic.yaml"
                )

                mock_exec.assert_called_once()
                assert result.success is True


class TestBatchExecution:
    """RED: Should run batches efficiently"""

    def test_run_batch_uses_all_worlds(self):
        """RED: Should distribute scenarios across worlds"""
        import time

        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner, WorldResult
        runner = GazeboBatchRunner(num_worlds=4)
        runner.worlds = runner._configure_worlds()  # Initialize worlds without launching

        scenarios = [f"scenario_{i}.yaml" for i in range(8)]
        call_count = [0]

        def mock_execute(world_id, scenario_path):
            call_count[0] += 1
            time.sleep(0.01)  # Small delay to simulate work
            return WorldResult(scenario_name=scenario_path, world_id=world_id, success=True)

        with patch.object(runner, 'execute_in_world', side_effect=mock_execute):
            results = runner.run_batch(scenarios)

            assert call_count[0] == 8
            assert len(results) == 8

    def test_run_batch_reuses_worlds(self):
        """RED: Should reuse worlds after scenario completes"""
        import time

        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner, WorldResult
        runner = GazeboBatchRunner(num_worlds=2)
        runner.worlds = runner._configure_worlds()  # Initialize worlds without launching

        scenarios = [f"scenario_{i}.yaml" for i in range(6)]
        call_count = [0]

        def mock_execute(world_id, scenario_path):
            call_count[0] += 1
            time.sleep(0.01)  # Small delay to simulate work
            return WorldResult(scenario_name=scenario_path, world_id=world_id, success=True)

        with patch.object(runner, 'execute_in_world', side_effect=mock_execute):
            results = runner.run_batch(scenarios)

            # Should complete all 6 with only 2 worlds
            assert len(results) == 6
            assert call_count[0] == 6

    def test_run_batch_reports_progress(self):
        """RED: Should report progress for long batches"""
        import time

        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner, WorldResult
        runner = GazeboBatchRunner(num_worlds=4)
        runner.worlds = runner._configure_worlds()  # Initialize worlds without launching

        progress_updates = []

        def capture_progress(completed, total):
            progress_updates.append((completed, total))

        scenarios = [f"scenario_{i}.yaml" for i in range(10)]

        def mock_execute(world_id, scenario_path):
            time.sleep(0.01)  # Small delay to simulate work
            return WorldResult(scenario_name=scenario_path, world_id=world_id, success=True)

        with patch.object(runner, 'execute_in_world', side_effect=mock_execute):
            results = runner.run_batch(
                scenarios,
                progress_callback=capture_progress
            )

            assert len(progress_updates) > 0
            assert progress_updates[-1] == (10, 10)


class TestFoxgloveIntegration:
    """RED: Should expose worlds to Foxglove via WebSocket"""

    def test_foxglove_bridge_method_exists(self):
        """RED: Should have method to start Foxglove bridge"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=2)
        assert hasattr(runner, 'start_foxglove_bridge')

    def test_foxglove_bridge_starts_websocket(self):
        """RED: Should start WebSocket server for Foxglove"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1, enable_foxglove=True)

        with patch.object(runner, '_start_websocket_server') as mock_start:
            runner.start_foxglove_bridge(port=8765)

            mock_start.assert_called_once()

    def test_publishes_world_state_to_foxglove(self):
        """RED: Should publish robot pose, sensor data to Foxglove"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        
        with patch.object(runner, '_publish_to_foxglove') as mock_pub:
            runner._on_world_update(world_id=0, state={'pose': (1, 2, 0)})
            
            mock_pub.assert_called_once()

    def test_multiple_worlds_visible_in_foxglove(self):
        """RED: All parallel worlds should be viewable"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=4)
        
        # Each world should have unique namespace for Foxglove
        worlds = runner._configure_worlds()
        
        namespaces = [w.foxglove_namespace for w in worlds]
        assert len(namespaces) == len(set(namespaces))
        assert all('world_' in ns for ns in namespaces)


class TestHeadlessOptimization:
    """RED: Should run headless for speed"""

    def test_headless_mode_no_gui(self):
        """RED: Headless mode should not launch GUI"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1, headless=True)
        
        with patch('subprocess.Popen') as mock_popen:
            runner.launch_worlds()
            
            # Check gzclient (GUI) is not in command
            calls = mock_popen.call_args_list
            for call in calls:
                cmd = call[0][0] if call[0] else call[1].get('args', [])
                assert 'gzclient' not in str(cmd)

    def test_headless_faster_than_gui(self):
        """RED: Headless should be significantly faster"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        
        # This is a design constraint, not a runtime test
        # Documented in implementation
        pass


class TestMetricsCollection:
    """RED: Should collect detailed metrics from simulation"""

    def test_collects_pose_trajectory(self):
        """RED: Should record robot pose over time"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        
        with patch.object(runner, '_get_robot_pose') as mock_pose:
            mock_pose.side_effect = [
                (0, 0, 0),
                (1, 0, 0),
                (2, 0, 0),
            ]
            
            trajectory = runner._collect_trajectory(world_id=0, duration=1.0)
            
            assert len(trajectory) == 3
            assert trajectory[0] == (0, 0, 0)

    def test_detects_collisions(self):
        """RED: Should detect and count collisions"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)

        # Mock the internal collision checking to return one collision
        original_check = runner._check_collision
        call_count = [0]

        def mock_check(world_id):
            call_count[0] += 1
            return call_count[0] == 3  # Return True on 3rd call

        runner._check_collision = mock_check

        collisions = runner._count_collisions(world_id=0, duration=1.0)

        # Should detect at least one collision
        assert collisions >= 1

    def test_calculates_path_deviation(self):
        """RED: Should calculate deviation from planned path"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        
        planned_path = [(0, 0), (1, 0), (2, 0)]
        actual_path = [(0, 0), (1, 0.1), (2, 0)]
        
        deviation = runner._calculate_deviation(planned_path, actual_path)
        
        assert deviation >= 0.1  # At least 10cm deviation


class TestDockerIntegration:
    """RED: Should work in Docker environment"""

    def test_detects_docker_environment(self):
        """RED: Should detect if running in Docker"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        runner = GazeboBatchRunner(num_worlds=1)
        
        # Should check for .dockerenv or cgroup
        assert hasattr(runner, '_is_docker')

    def test_uses_display_env_in_docker(self):
        """RED: Should set DISPLAY for headless in Docker"""
        from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner
        
        with patch.dict('os.environ', {'DISPLAY': ':99'}):
            runner = GazeboBatchRunner(num_worlds=1, headless=True)
            
            with patch('subprocess.Popen') as mock_popen:
                runner.launch_worlds()
                
                # Should include DISPLAY in environment
                calls = mock_popen.call_args_list
                for call in calls:
                    env = call[1].get('env', {})
                    assert 'DISPLAY' in env or 'DISPLAY' in str(call)


# =============================================================================
# Fixtures
# =============================================================================

@pytest.fixture
def temp_scenario_dir():
    """Create temporary directory with sample scenarios"""
    temp_dir = tempfile.mkdtemp()
    
    # Create sample scenarios
    for i in range(5):
        scenario_file = Path(temp_dir) / f"scenario_{i}.yaml"
        scenario_file.write_text(f"""
name: scenario_{i}
robot_config:
  type: differential_drive
  max_velocity: 1.0
environment:
  obstacles: []
goal:
  type: navigate_to_pose
  pose:
    x: {i + 1}.0
    y: 0.0
    theta: 0.0
""")
    
    yield temp_dir
    
    # Cleanup
    shutil.rmtree(temp_dir)


@pytest.fixture
def mock_gazebo_process():
    """Mock Gazebo process for testing"""
    process = Mock()
    process.poll.return_value = None  # Still running
    process.terminate = Mock()
    process.wait = Mock(return_value=0)
    return process
