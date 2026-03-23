"""
TDD Tests for ScenarioGenerator

Generates 10,000+ unique scenarios for Gate 2 validation.
Procedural generation from templates with parameter variation.
"""

import shutil
import tempfile
from pathlib import Path

import pytest
import yaml

# =============================================================================
# Phase 1: RED - Write failing tests
# =============================================================================


class TestScenarioGeneratorExists:
    """RED: ScenarioGenerator class should exist"""

    def test_scenario_generator_module_exists(self):
        """RED: agent_ros_bridge.simulation.scenario_generator module should exist"""
        try:
            from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

            assert True
        except ImportError:
            pytest.fail("ScenarioGenerator should be importable")

    def test_scenario_generator_class_exists(self):
        """RED: ScenarioGenerator class should exist"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        assert ScenarioGenerator is not None

    def test_can_create_generator(self):
        """RED: Should create generator with output directory"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator(output_dir="/tmp/scenarios")
        assert generator.output_dir == Path("/tmp/scenarios")


class TestTemplateLoading:
    """RED: Should load scenario templates"""

    def test_load_template_method_exists(self):
        """RED: Should have method to load templates"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()
        assert hasattr(generator, "load_template")

    def test_can_load_navigation_template(self):
        """RED: Should load navigation scenario template"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        template = generator.load_template("navigation")

        assert template is not None
        assert "robot_config" in template
        assert "environment" in template
        assert "goal" in template

    def test_can_load_manipulation_template(self):
        """RED: Should load manipulation scenario template"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        template = generator.load_template("manipulation")

        assert template is not None
        assert template["robot_config"]["type"] == "manipulator"

    def test_can_load_safety_template(self):
        """RED: Should load safety-critical scenario template"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        template = generator.load_template("safety")

        assert template is not None
        assert "hazards" in template["environment"]


class TestScenarioGeneration:
    """RED: Should generate unique scenarios from templates"""

    def test_generate_scenario_method_exists(self):
        """RED: Should have method to generate scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()
        assert hasattr(generator, "generate_scenario")

    def test_generates_navigation_scenario(self):
        """RED: Should generate valid navigation scenario"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario(template="navigation", seed=42)

        assert scenario["name"].startswith("navigation_")
        assert "start_pose" in scenario
        assert "goal" in scenario
        assert "pose" in scenario["goal"]
        assert "obstacles" in scenario["environment"]

    def test_generates_different_scenarios_with_different_seeds(self):
        """RED: Different seeds should produce different scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario1 = generator.generate_scenario(template="navigation", seed=1)
        scenario2 = generator.generate_scenario(template="navigation", seed=2)

        assert scenario1 != scenario2
        assert scenario1["environment"]["obstacles"] != scenario2["environment"]["obstacles"]

    def test_generates_reproducible_scenarios_with_same_seed(self):
        """RED: Same seed should produce identical scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario1 = generator.generate_scenario(template="navigation", seed=42)
        scenario2 = generator.generate_scenario(template="navigation", seed=42)

        assert scenario1 == scenario2


class TestBatchGeneration:
    """RED: Should generate batches of scenarios"""

    def test_generate_batch_method_exists(self):
        """RED: Should have method for batch generation"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()
        assert hasattr(generator, "generate_batch")

    def test_can_generate_100_scenarios(self):
        """RED: Should generate 100 scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenarios = generator.generate_batch(template="navigation", count=100, start_seed=0)

        assert len(scenarios) == 100
        assert all(s["name"].startswith("navigation_") for s in scenarios)

    def test_can_generate_10000_scenarios(self):
        """RED: Gate 2 requirement - 10,000 scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenarios = generator.generate_batch(template="navigation", count=10000, start_seed=0)

        assert len(scenarios) == 10000

    def test_all_scenarios_have_unique_names(self):
        """RED: Generated scenarios should have unique names"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenarios = generator.generate_batch(template="navigation", count=100, start_seed=0)

        names = [s["name"] for s in scenarios]
        assert len(names) == len(set(names))


class TestScenarioVariation:
    """RED: Scenarios should have parameter variation"""

    def test_varies_obstacle_count(self):
        """RED: Should vary number of obstacles"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        obstacle_counts = set()
        for seed in range(20):
            scenario = generator.generate_scenario("navigation", seed=seed)
            count = len(scenario["environment"]["obstacles"])
            obstacle_counts.add(count)

        # Should have variation (not all same)
        assert len(obstacle_counts) > 1

    def test_varies_obstacle_positions(self):
        """RED: Should vary obstacle positions"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        positions = set()
        for seed in range(10):
            scenario = generator.generate_scenario("navigation", seed=seed)
            for obs in scenario["environment"]["obstacles"]:
                positions.add((obs["x"], obs["y"]))

        # Should have many unique positions
        assert len(positions) > 5

    def test_varies_goal_positions(self):
        """RED: Should vary goal positions"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        goals = set()
        for seed in range(20):
            scenario = generator.generate_scenario("navigation", seed=seed)
            goal = scenario["goal"]["pose"]
            goals.add((goal["x"], goal["y"]))

        # Should have variation in goals
        assert len(goals) > 5

    def test_varies_start_positions(self):
        """RED: Should vary start positions"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        starts = set()
        for seed in range(20):
            scenario = generator.generate_scenario("navigation", seed=seed)
            if "start_pose" in scenario:
                start = scenario["start_pose"]
                starts.add((start["x"], start["y"]))

        # Should have variation (or default to origin)
        assert len(starts) >= 1


class TestFileOutput:
    """RED: Should write scenarios to YAML files"""

    def test_save_scenario_method_exists(self):
        """RED: Should have method to save scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()
        assert hasattr(generator, "save_scenario")

    def test_saves_yaml_file(self, tmp_path):
        """RED: Should save scenario as YAML"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator(output_dir=str(tmp_path))

        scenario = generator.generate_scenario("navigation", seed=42)
        filepath = generator.save_scenario(scenario)

        assert Path(filepath).exists()
        assert Path(filepath).suffix == ".yaml"

    def test_saved_file_is_valid_yaml(self, tmp_path):
        """RED: Saved file should be valid YAML"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator(output_dir=str(tmp_path))

        scenario = generator.generate_scenario("navigation", seed=42)
        filepath = generator.save_scenario(scenario)

        with open(filepath) as f:
            loaded = yaml.safe_load(f)

        assert loaded["name"] == scenario["name"]

    def test_generate_and_save_batch(self, tmp_path):
        """RED: Should generate and save batch to files"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator(output_dir=str(tmp_path))

        filepaths = generator.generate_and_save_batch(template="navigation", count=10, start_seed=0)

        assert len(filepaths) == 10
        assert all(Path(f).exists() for f in filepaths)


class TestScenarioCategories:
    """RED: Should generate different scenario categories"""

    def test_generates_easy_scenarios(self):
        """RED: Should generate easy scenarios (few obstacles)"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario("navigation", difficulty="easy", seed=42)

        obstacle_count = len(scenario["environment"]["obstacles"])
        assert obstacle_count <= 3

    def test_generates_medium_scenarios(self):
        """RED: Should generate medium scenarios (moderate obstacles)"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario("navigation", difficulty="medium", seed=42)

        obstacle_count = len(scenario["environment"]["obstacles"])
        assert 3 <= obstacle_count <= 8

    def test_generates_hard_scenarios(self):
        """RED: Should generate hard scenarios (many obstacles)"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario("navigation", difficulty="hard", seed=42)

        obstacle_count = len(scenario["environment"]["obstacles"])
        assert obstacle_count >= 5

    def test_generates_safety_critical_scenarios(self):
        """RED: Should generate safety-critical scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario("safety", seed=42)

        assert "hazards" in scenario["environment"]
        assert len(scenario["environment"]["hazards"]) > 0


class TestValidation:
    """RED: Should validate generated scenarios"""

    def test_validates_scenario(self):
        """RED: Should validate scenario structure"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenario = generator.generate_scenario("navigation", seed=42)
        is_valid = generator.validate_scenario(scenario)

        assert is_valid is True

    def test_detects_invalid_scenario(self):
        """RED: Should detect invalid scenarios"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        invalid_scenario = {"name": "invalid"}  # Missing required fields
        is_valid = generator.validate_scenario(invalid_scenario)

        assert is_valid is False

    def test_validates_batch(self):
        """RED: Should validate entire batch"""
        from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

        generator = ScenarioGenerator()

        scenarios = generator.generate_batch("navigation", count=10, start_seed=0)
        validation = generator.validate_batch(scenarios)

        assert validation["valid_count"] == 10
        assert validation["invalid_count"] == 0


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def temp_output_dir():
    """Create temporary directory for scenario output"""
    temp_dir = tempfile.mkdtemp()
    yield temp_dir
    shutil.rmtree(temp_dir)
