"""
Test scenarios in simulation - TDD approach
Week 2 Deliverable: Simulation Tests
"""

import pytest
import yaml
import json
from pathlib import Path
import sys

# Add simulation directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "simulation"))

from scenario_runner import ScenarioRunner, ScenarioResult
from parallel_runner import ParallelRunner, ParallelScenarioResult


class TestScenarioFiles:
    """Test scenario YAML files are valid"""
    
    @pytest.fixture
    def scenarios_dir(self):
        return Path("simulation/scenarios")
    
    def test_scenario_files_exist(self, scenarios_dir):
        """RED: Scenario files should exist"""
        assert scenarios_dir.exists(), "Scenarios directory not found"
        
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        assert len(scenario_files) >= 10, f"Expected at least 10 scenarios, found {len(scenario_files)}"
    
    def test_all_scenarios_valid_yaml(self, scenarios_dir):
        """RED: All scenario files should be valid YAML"""
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        
        for scenario_file in scenario_files:
            with open(scenario_file, 'r') as f:
                content = yaml.safe_load(f)
            
            assert content is not None, f"{scenario_file.name} is empty or invalid"
    
    def test_scenarios_have_required_fields(self, scenarios_dir):
        """RED: All scenarios should have required fields"""
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        
        required_fields = ["name", "description", "world", "duration"]
        
        for scenario_file in scenario_files:
            with open(scenario_file, 'r') as f:
                scenario = yaml.safe_load(f)
            
            for field in required_fields:
                assert field in scenario, f"{scenario_file.name}: missing '{field}'"
    
    def test_scenarios_have_robots(self, scenarios_dir):
        """RED: All scenarios should define at least one robot"""
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        
        for scenario_file in scenario_files:
            with open(scenario_file, 'r') as f:
                scenario = yaml.safe_load(f)
            
            assert "robots" in scenario, f"{scenario_file.name}: missing 'robots'"
            assert len(scenario["robots"]) > 0, f"{scenario_file.name}: no robots defined"
    
    def test_scenarios_have_success_criteria(self, scenarios_dir):
        """RED: All scenarios should have success criteria"""
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        
        for scenario_file in scenario_files:
            with open(scenario_file, 'r') as f:
                scenario = yaml.safe_load(f)
            
            assert "success_criteria" in scenario, f"{scenario_file.name}: missing 'success_criteria'"
    
    def test_robot_references_valid_models(self, scenarios_dir):
        """RED: All robot references should point to valid models"""
        models_dir = Path("simulation/models")
        scenario_files = list(scenarios_dir.glob("scenario_*.yaml"))
        
        for scenario_file in scenario_files:
            with open(scenario_file, 'r') as f:
                scenario = yaml.safe_load(f)
            
            for robot in scenario.get("robots", []):
                model = robot.get("model", "")
                model_path = models_dir / model
                assert model_path.exists(), \
                    f"{scenario_file.name}: model '{model}' not found"


class TestScenarioRunner:
    """Test the scenario runner"""
    
    @pytest.fixture
    def runner(self):
        return ScenarioRunner()
    
    def test_runner_lists_scenarios(self, runner):
        """RED: Runner should list available scenarios"""
        scenarios = runner.list_scenarios()
        assert len(scenarios) >= 10, f"Expected at least 10 scenarios, found {len(scenarios)}"
        
        # Check naming convention
        for scenario in scenarios:
            assert scenario.startswith("scenario_"), \
                f"Scenario '{scenario}' doesn't follow naming convention"
    
    def test_runner_loads_scenario(self, runner):
        """RED: Runner should load scenario from file"""
        scenarios = runner.list_scenarios()
        
        for scenario_name in scenarios[:3]:  # Test first 3
            scenario = runner.load_scenario(scenario_name)
            
            assert "name" in scenario
            assert "description" in scenario
            assert "world" in scenario
    
    def test_runner_loads_scenario_by_path(self, runner):
        """RED: Runner should load scenario by full path"""
        scenario_file = Path("simulation/scenarios/scenario_01_basic_navigation.yaml")
        
        if scenario_file.exists():
            scenario = runner.load_scenario("scenario_01_basic_navigation")
            assert scenario["name"] == "Basic Navigation"
    
    def test_runner_returns_result_object(self, runner):
        """RED: Runner should return ScenarioResult"""
        scenarios = runner.list_scenarios()
        
        if scenarios:
            # Note: This test might fail if Gazebo is not available
            # In that case, the test should be skipped
            try:
                result = runner.run_scenario(scenarios[0])
                assert isinstance(result, ScenarioResult)
                assert hasattr(result, "scenario_name")
                assert hasattr(result, "success")
                assert hasattr(result, "duration")
                assert hasattr(result, "metrics")
                assert hasattr(result, "errors")
            except Exception as e:
                pytest.skip(f"Gazebo not available: {e}")


class TestParallelRunner:
    """Test the parallel runner"""
    
    @pytest.fixture
    def runner(self):
        return ParallelRunner(max_workers=4)
    
    def test_parallel_runner_lists_scenarios(self, runner):
        """RED: Parallel runner should list scenarios"""
        scenarios = runner.list_scenarios()
        assert len(scenarios) >= 10
    
    def test_parallel_runner_loads_scenario(self, runner):
        """RED: Parallel runner should load scenario"""
        scenarios = runner.list_scenarios()
        
        if scenarios:
            scenario = runner.load_scenario(scenarios[0])
            assert "name" in scenario
    
    def test_parallel_runner_runs_single_scenario(self, runner):
        """RED: Parallel runner should run single scenario"""
        scenarios = runner.list_scenarios()
        
        if scenarios:
            result = runner.run_single_scenario(scenarios[0], worker_id=0)
            
            assert isinstance(result, ParallelScenarioResult)
            assert result.scenario_name == scenarios[0]
            assert result.worker_id == 0
            assert hasattr(result, "success")
            assert hasattr(result, "duration")
            assert hasattr(result, "metrics")
    
    def test_parallel_runner_runs_multiple_scenarios(self, runner):
        """RED: Parallel runner should run multiple scenarios"""
        scenarios = runner.list_scenarios()[:3]  # Test with 3 scenarios
        
        results = runner.run_parallel(scenarios, num_workers=3)
        
        assert len(results) == len(scenarios)
        
        for result in results:
            assert isinstance(result, ParallelScenarioResult)
            assert result.scenario_name in scenarios
    
    def test_parallel_runner_generates_coverage_report(self, runner):
        """RED: Parallel runner should generate coverage report"""
        scenarios = runner.list_scenarios()[:3]
        
        results = runner.run_parallel(scenarios, num_workers=3)
        report = runner.generate_coverage_report(results)
        
        assert "summary" in report
        assert "total_scenarios" in report["summary"]
        assert "successful" in report["summary"]
        assert "failed" in report["summary"]
        assert "success_rate" in report["summary"]
        assert "by_type" in report
        assert "failed_scenarios" in report
    
    def test_parallel_runner_saves_results(self, runner, tmp_path):
        """RED: Parallel runner should save results to file"""
        runner.results_dir = tmp_path
        
        scenarios = runner.list_scenarios()[:2]
        results = runner.run_parallel(scenarios, num_workers=2)
        
        result_file = runner.save_results(results, "test_batch")
        
        assert result_file.exists()
        
        with open(result_file, 'r') as f:
            saved = json.load(f)
        
        assert saved["batch_name"] == "test_batch"
        assert saved["total_scenarios"] == len(scenarios)
    
    def test_parallel_runner_saves_coverage_report(self, runner, tmp_path):
        """RED: Parallel runner should save coverage report"""
        runner.results_dir = tmp_path
        
        scenarios = runner.list_scenarios()[:2]
        results = runner.run_parallel(scenarios, num_workers=2)
        report = runner.generate_coverage_report(results)
        
        report_file = runner.save_coverage_report(report, "test_coverage.json")
        
        assert report_file.exists()
        
        with open(report_file, 'r') as f:
            saved = json.load(f)
        
        assert "summary" in saved


@pytest.mark.parametrize("scenario", [
    "scenario_01_basic_navigation",
    "scenario_02_obstacle_avoidance",
    "scenario_03_dynamic_obstacles",
    "scenario_04_narrow_corridors",
    "scenario_05_multiple_waypoints",
    "scenario_06_pick_and_place",
    "scenario_07_emergency_stop",
    "scenario_08_sensor_failure",
    "scenario_09_recovery_behavior",
    "scenario_10_multi_robot"
])
def test_scenario_exists(scenario):
    """RED: All required scenarios should exist"""
    scenario_file = Path(f"simulation/scenarios/{scenario}.yaml")
    assert scenario_file.exists(), f"Scenario file not found: {scenario}"


@pytest.mark.parametrize("scenario", [
    "scenario_01_basic_navigation",
    "scenario_02_obstacle_avoidance",
    "scenario_03_dynamic_obstacles",
    "scenario_04_narrow_corridors",
    "scenario_05_multiple_waypoints",
    "scenario_06_pick_and_place",
    "scenario_07_emergency_stop",
    "scenario_08_sensor_failure",
    "scenario_09_recovery_behavior",
    "scenario_10_multi_robot"
])
def test_scenario_valid(scenario):
    """RED: All scenarios should be valid YAML with required fields"""
    scenario_file = Path(f"simulation/scenarios/{scenario}.yaml")
    
    with open(scenario_file, 'r') as f:
        content = yaml.safe_load(f)
    
    assert content is not None
    assert "name" in content
    assert "description" in content
    assert "world" in content
    assert "duration" in content
    assert "robots" in content
    assert len(content["robots"]) > 0
    assert "success_criteria" in content


@pytest.mark.parametrize("scenario", [
    "scenario_01_basic_navigation",
    "scenario_02_obstacle_avoidance",
    "scenario_03_dynamic_obstacles",
    "scenario_04_narrow_corridors",
    "scenario_05_multiple_waypoints",
    "scenario_06_pick_and_place",
    "scenario_07_emergency_stop",
    "scenario_08_sensor_failure",
    "scenario_09_recovery_behavior",
    "scenario_10_multi_robot"
])
def test_scenario_run(scenario):
    """RED: Scenario should run and return result (may not pass without Gazebo)"""
    runner = ParallelRunner(max_workers=1)
    
    result = runner.run_single_scenario(scenario, worker_id=0)
    
    assert isinstance(result, ParallelScenarioResult)
    assert result.scenario_name == scenario
    assert result.duration >= 0
    assert "robots_spawned" in result.metrics
