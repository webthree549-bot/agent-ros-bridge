"""
ScenarioGenerator - Procedural Generation for 10K+ Scenarios

Generates unique, reproducible scenarios for Gate 2 validation.
Supports navigation, manipulation, and safety-critical scenarios.
"""

import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# Built-in scenario templates
DEFAULT_TEMPLATES = {
    "navigation": {
        "robot_config": {
            "type": "differential_drive",
            "max_velocity": 1.0,
            "max_acceleration": 0.5,
            "radius": 0.3,
        },
        "environment": {
            "bounds": {
                "min_x": 0.0,
                "max_x": 20.0,
                "min_y": 0.0,
                "max_y": 20.0,
            },
            "obstacles": [],  # To be filled procedurally
        },
        "goal": {
            "type": "navigate_to_pose",
            "pose": {},  # To be filled procedurally
        },
    },
    "manipulation": {
        "robot_config": {
            "type": "manipulator",
            "dof": 6,
            "max_joint_velocity": 1.5,
        },
        "environment": {
            "table": {
                "position": [0.5, 0.0, 0.0],
                "size": [1.0, 0.6, 0.02],
            },
            "objects": [],  # To be filled procedurally
        },
        "goal": {
            "type": "pick_and_place",
            "object": "",  # To be filled procedurally
            "target": {},  # To be filled procedurally
        },
    },
    "safety": {
        "robot_config": {
            "type": "differential_drive",
            "max_velocity": 0.5,  # Slower for safety
            "max_acceleration": 0.3,
            "sensors": ["lidar", "camera", "bumper"],
        },
        "environment": {
            "bounds": {
                "min_x": 0.0,
                "max_x": 10.0,
                "min_y": 0.0,
                "max_y": 10.0,
            },
            "obstacles": [],
            "hazards": [],  # Safety-critical elements
            "humans": [],  # Human obstacles
        },
        "goal": {
            "type": "navigate_safely",
            "pose": {},
            "safety_constraints": {
                "min_human_distance": 1.0,
                "max_speed_near_humans": 0.3,
            },
        },
    },
}

# Difficulty parameters
DIFFICULTY_PARAMS = {
    "easy": {
        "obstacle_count_range": (0, 3),
        "obstacle_size_range": (0.1, 0.3),
        "goal_distance_range": (2.0, 5.0),
    },
    "medium": {
        "obstacle_count_range": (3, 8),
        "obstacle_size_range": (0.2, 0.5),
        "goal_distance_range": (5.0, 10.0),
    },
    "hard": {
        "obstacle_count_range": (5, 15),
        "obstacle_size_range": (0.3, 0.8),
        "goal_distance_range": (8.0, 15.0),
    },
}


@dataclass
class GenerationConfig:
    """Configuration for scenario generation"""

    output_dir: Path
    template_dir: Path | None = None
    default_difficulty: str = "medium"
    bounds_padding: float = 1.0  # Keep obstacles away from edges
    min_obstacle_distance: float = 0.5  # Min distance between obstacles


class ScenarioGenerator:
    """
    Generates procedural scenarios for simulation testing.

    Features:
    - Multiple scenario types (navigation, manipulation, safety)
    - Difficulty levels (easy, medium, hard)
    - Reproducible with seeds
    - Batch generation (10K+ scenarios)
    - YAML output for ScenarioRunner
    """

    def __init__(
        self,
        output_dir: str | Path = "scenarios",
        template_dir: str | Path | None = None,
    ):
        """
        Initialize scenario generator.

        Args:
            output_dir: Directory to save generated scenarios
            template_dir: Optional directory with custom templates
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.template_dir = Path(template_dir) if template_dir else None

        # Load templates
        self.templates = self._load_templates()

    def _load_templates(self) -> dict[str, dict]:
        """Load scenario templates from built-in and custom sources"""
        templates = DEFAULT_TEMPLATES.copy()

        # Load custom templates if provided
        if self.template_dir and self.template_dir.exists():
            for template_file in self.template_dir.glob("*.yaml"):
                name = template_file.stem
                with open(template_file) as f:
                    templates[name] = yaml.safe_load(f)

        return templates

    def load_template(self, template_name: str) -> dict[str, Any]:
        """
        Load a scenario template.

        Args:
            template_name: Name of template (navigation, manipulation, safety)

        Returns:
            Template dictionary
        """
        if template_name not in self.templates:
            raise ValueError(
                f"Unknown template: {template_name}. " f"Available: {list(self.templates.keys())}"
            )

        # Return deep copy
        import copy

        return copy.deepcopy(self.templates[template_name])

    def generate_scenario(
        self, template: str, seed: int, difficulty: str | None = None, **kwargs
    ) -> dict[str, Any]:
        """
        Generate a single scenario.

        Args:
            template: Template name
            seed: Random seed for reproducibility
            difficulty: Difficulty level (easy, medium, hard)
            **kwargs: Additional parameters

        Returns:
            Scenario dictionary
        """
        # Set random seed for reproducibility
        rng = random.Random(seed)

        # Get difficulty params
        difficulty = difficulty or "medium"
        diff_params = DIFFICULTY_PARAMS.get(difficulty, DIFFICULTY_PARAMS["medium"])

        # Load base template
        scenario = self.load_template(template)

        # Generate unique name
        scenario["name"] = f"{template}_{seed:06d}_{difficulty}"

        if template == "navigation":
            self._generate_navigation_scenario(scenario, rng, diff_params)
        elif template == "manipulation":
            self._generate_manipulation_scenario(scenario, rng, diff_params)
        elif template == "safety":
            self._generate_safety_scenario(scenario, rng, diff_params)

        return scenario

    def _generate_navigation_scenario(
        self,
        scenario: dict,
        rng: random.Random,
        diff_params: dict,
    ) -> None:
        """Generate navigation-specific scenario elements"""
        bounds = scenario["environment"]["bounds"]
        padding = 1.0

        # Generate start pose (near origin)
        scenario["start_pose"] = {
            "x": rng.uniform(padding, padding + 2.0),
            "y": rng.uniform(padding, padding + 2.0),
            "theta": rng.uniform(-3.14, 3.14),
        }

        # Generate goal pose (far from start)
        min_dist, max_dist = diff_params["goal_distance_range"]
        goal_dist = rng.uniform(min_dist, max_dist)

        scenario["goal"]["pose"] = {
            "x": scenario["start_pose"]["x"] + goal_dist * rng.uniform(0.5, 1.0),
            "y": scenario["start_pose"]["y"] + goal_dist * rng.uniform(0.5, 1.0),
            "theta": rng.uniform(-3.14, 3.14),
        }

        # Clip to bounds
        scenario["goal"]["pose"]["x"] = max(
            padding, min(bounds["max_x"] - padding, scenario["goal"]["pose"]["x"])
        )
        scenario["goal"]["pose"]["y"] = max(
            padding, min(bounds["max_y"] - padding, scenario["goal"]["pose"]["y"])
        )

        # Generate obstacles
        min_obs, max_obs = diff_params["obstacle_count_range"]
        num_obstacles = rng.randint(min_obs, max_obs)

        obstacles = []
        for i in range(num_obstacles):
            min_size, max_size = diff_params["obstacle_size_range"]

            # Try to place obstacle without overlap
            max_attempts = 100
            for _ in range(max_attempts):
                x = rng.uniform(padding, bounds["max_x"] - padding)
                y = rng.uniform(padding, bounds["max_y"] - padding)
                radius = rng.uniform(min_size, max_size) / 2

                # Check distance from start and goal
                dist_to_start = (
                    (x - scenario["start_pose"]["x"]) ** 2 + (y - scenario["start_pose"]["y"]) ** 2
                ) ** 0.5
                dist_to_goal = (
                    (x - scenario["goal"]["pose"]["x"]) ** 2
                    + (y - scenario["goal"]["pose"]["y"]) ** 2
                ) ** 0.5

                if dist_to_start > 1.0 and dist_to_goal > 1.0:
                    # Check distance from other obstacles
                    valid = True
                    for obs in obstacles:
                        dist = ((x - obs["x"]) ** 2 + (y - obs["y"]) ** 2) ** 0.5
                        if dist < (radius + obs["radius"] + 0.5):
                            valid = False
                            break

                    if valid:
                        obstacles.append(
                            {
                                "x": round(x, 2),
                                "y": round(y, 2),
                                "radius": round(radius, 2),
                            }
                        )
                        break

        scenario["environment"]["obstacles"] = obstacles

    def _generate_manipulation_scenario(
        self,
        scenario: dict,
        rng: random.Random,
        diff_params: dict,
    ) -> None:
        """Generate manipulation-specific scenario elements"""
        # Generate objects on table
        num_objects = rng.randint(1, 5)
        objects = []

        for i in range(num_objects):
            obj_type = rng.choice(["cube", "cylinder", "sphere"])
            objects.append(
                {
                    "name": f"obj_{i}",
                    "type": obj_type,
                    "position": [
                        round(rng.uniform(0.3, 0.7), 2),
                        round(rng.uniform(-0.2, 0.2), 2),
                        round(rng.uniform(0.05, 0.1), 2),
                    ],
                    "size": round(rng.uniform(0.03, 0.08), 2),
                }
            )

        scenario["environment"]["objects"] = objects

        # Select pick and place targets
        if objects:
            pick_obj = rng.choice(objects)
            scenario["goal"]["object"] = pick_obj["name"]
            scenario["goal"]["target"] = {
                "position": [
                    round(rng.uniform(0.3, 0.7), 2),
                    round(rng.uniform(0.1, 0.3), 2),  # Other side of table
                    pick_obj["position"][2],
                ],
            }

    def _generate_safety_scenario(
        self,
        scenario: dict,
        rng: random.Random,
        diff_params: dict,
    ) -> None:
        """Generate safety-critical scenario elements"""
        bounds = scenario["environment"]["bounds"]
        padding = 0.5

        # Generate start and goal like navigation
        scenario["start_pose"] = {
            "x": rng.uniform(padding, padding + 1.0),
            "y": rng.uniform(padding, padding + 1.0),
            "theta": rng.uniform(-3.14, 3.14),
        }

        scenario["goal"]["pose"] = {
            "x": rng.uniform(bounds["max_x"] - padding - 2.0, bounds["max_x"] - padding),
            "y": rng.uniform(bounds["max_y"] - padding - 2.0, bounds["max_y"] - padding),
            "theta": rng.uniform(-3.14, 3.14),
        }

        # Generate static obstacles
        min_obs, max_obs = diff_params["obstacle_count_range"]
        num_obstacles = rng.randint(min_obs // 2, max_obs // 2)

        obstacles = []
        for i in range(num_obstacles):
            obstacles.append(
                {
                    "x": round(rng.uniform(padding, bounds["max_x"] - padding), 2),
                    "y": round(rng.uniform(padding, bounds["max_y"] - padding), 2),
                    "radius": round(rng.uniform(0.1, 0.3), 2),
                }
            )

        scenario["environment"]["obstacles"] = obstacles

        # Generate safety hazards (zones robot must avoid)
        num_hazards = rng.randint(1, 3)
        hazards = []
        for i in range(num_hazards):
            hazards.append(
                {
                    "type": rng.choice(["drop_zone", "collision_zone", "no_entry"]),
                    "position": {
                        "x": round(rng.uniform(padding, bounds["max_x"] - padding), 2),
                        "y": round(rng.uniform(padding, bounds["max_y"] - padding), 2),
                    },
                    "radius": round(rng.uniform(0.5, 1.5), 2),
                }
            )

        scenario["environment"]["hazards"] = hazards

        # Generate human obstacles (moving)
        num_humans = rng.randint(1, 3)
        humans = []
        for i in range(num_humans):
            humans.append(
                {
                    "id": i,
                    "start_position": {
                        "x": round(rng.uniform(padding, bounds["max_x"] - padding), 2),
                        "y": round(rng.uniform(padding, bounds["max_y"] - padding), 2),
                    },
                    "velocity": round(rng.uniform(0.5, 1.2), 2),
                    "path": "random_walk",  # Or specific path
                }
            )

        scenario["environment"]["humans"] = humans

    def generate_batch(
        self,
        template: str,
        count: int,
        start_seed: int = 0,
        difficulty: str | None = None,
    ) -> list[dict[str, Any]]:
        """
        Generate a batch of scenarios.

        Args:
            template: Template name
            count: Number of scenarios to generate
            start_seed: Starting seed for reproducibility
            difficulty: Difficulty level for all scenarios

        Returns:
            List of scenario dictionaries
        """
        scenarios = []

        for i in range(count):
            scenario = self.generate_scenario(
                template=template,
                seed=start_seed + i,
                difficulty=difficulty,
            )
            scenarios.append(scenario)

        return scenarios

    def save_scenario(
        self,
        scenario: dict[str, Any],
        filepath: str | Path | None = None,
    ) -> str:
        """
        Save scenario to YAML file.

        Args:
            scenario: Scenario dictionary
            filepath: Optional custom filepath

        Returns:
            Path to saved file
        """
        if filepath is None:
            filepath = self.output_dir / f"{scenario['name']}.yaml"
        else:
            filepath = Path(filepath)

        with open(filepath, "w") as f:
            yaml.dump(scenario, f, default_flow_style=False, sort_keys=False)

        return str(filepath)

    def generate_and_save_batch(
        self,
        template: str,
        count: int,
        start_seed: int = 0,
        difficulty: str | None = None,
    ) -> list[str]:
        """
        Generate and save a batch of scenarios.

        Args:
            template: Template name
            count: Number of scenarios
            start_seed: Starting seed
            difficulty: Difficulty level

        Returns:
            List of filepaths
        """
        scenarios = self.generate_batch(template, count, start_seed, difficulty)

        filepaths = []
        for scenario in scenarios:
            filepath = self.save_scenario(scenario)
            filepaths.append(filepath)

        return filepaths

    def validate_scenario(self, scenario: dict[str, Any]) -> bool:
        """
        Validate scenario structure.

        Args:
            scenario: Scenario dictionary

        Returns:
            True if valid, False otherwise
        """
        required_keys = ["name", "robot_config", "environment", "goal"]

        for key in required_keys:
            if key not in scenario:
                return False

        # Validate robot_config
        if "type" not in scenario["robot_config"]:
            return False

        # Validate environment
        return "bounds" in scenario["environment"] or "obstacles" in scenario["environment"]

    def validate_batch(self, scenarios: list[dict[str, Any]]) -> dict[str, Any]:
        """
        Validate a batch of scenarios.

        Args:
            scenarios: List of scenario dictionaries

        Returns:
            Validation report
        """
        valid_count = 0
        invalid_count = 0
        invalid_names = []

        for scenario in scenarios:
            if self.validate_scenario(scenario):
                valid_count += 1
            else:
                invalid_count += 1
                invalid_names.append(scenario.get("name", "unknown"))

        return {
            "total_count": len(scenarios),
            "valid_count": valid_count,
            "invalid_count": invalid_count,
            "invalid_names": invalid_names,
            "valid_rate": valid_count / len(scenarios) if scenarios else 0.0,
        }


# Convenience functions


def generate_10k_scenarios(
    output_dir: str = "scenarios/batch_10k",
    template: str = "navigation",
    difficulty: str = "medium",
) -> list[str]:
    """
    Generate 10,000 scenarios for Gate 2 validation.

    Args:
        output_dir: Output directory
        template: Scenario template
        difficulty: Difficulty level

    Returns:
        List of scenario filepaths
    """
    generator = ScenarioGenerator(output_dir=output_dir)

    print(f"Generating 10,000 {difficulty} {template} scenarios...")

    filepaths = generator.generate_and_save_batch(
        template=template,
        count=10000,
        start_seed=0,
        difficulty=difficulty,
    )

    # Validate
    print("Validating generated scenarios...")
    scenarios = [generator.load_template("navigation") for _ in range(100)]  # Dummy load
    scenarios = generator.generate_batch(template, 100, 0, difficulty)
    validation = generator.validate_batch(scenarios)

    print(f"Validation: {validation['valid_count']}/{validation['total_count']} valid")

    return filepaths


if __name__ == "__main__":
    # Generate sample scenarios
    generator = ScenarioGenerator()

    # Generate one of each difficulty
    for difficulty in ["easy", "medium", "hard"]:
        scenario = generator.generate_scenario("navigation", seed=42, difficulty=difficulty)
        filepath = generator.save_scenario(scenario)
        print(f"Generated {difficulty} scenario: {filepath}")
