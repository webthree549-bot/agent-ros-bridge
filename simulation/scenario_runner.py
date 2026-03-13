"""
Scenario Runner - Executes simulation test scenarios
Week 2 Deliverable: Scenario Runner
"""

import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ScenarioResult:
    """Result of running a scenario"""

    scenario_name: str
    success: bool
    duration: float
    metrics: dict[str, Any] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())


class ScenarioRunner:
    """Runs simulation scenarios and records results"""

    def __init__(self, scenarios_dir: str = "simulation/scenarios"):
        self.scenarios_dir = Path(scenarios_dir)
        self.results_dir = Path("results")
        self.results_dir.mkdir(exist_ok=True)
        self.gazebo_process = None

    def list_scenarios(self) -> list[str]:
        """List all available scenarios"""
        scenarios = []
        if self.scenarios_dir.exists():
            for scenario_file in sorted(self.scenarios_dir.glob("scenario_*.yaml")):
                scenarios.append(scenario_file.stem)
        return scenarios

    def load_scenario(self, scenario_name: str) -> dict[str, Any]:
        """Load a scenario from YAML file"""
        scenario_file = self.scenarios_dir / f"{scenario_name}.yaml"

        if not scenario_file.exists():
            # Try with .yaml extension
            scenario_file = self.scenarios_dir / scenario_name

        if not scenario_file.exists():
            raise FileNotFoundError(f"Scenario not found: {scenario_name}")

        with open(scenario_file) as f:
            return yaml.safe_load(f)

    def launch_gazebo(self, world: str = "empty_warehouse") -> bool:
        """Launch Gazebo with specified world"""
        try:
            # Check if Gazebo is already running
            result = subprocess.run(["pgrep", "-f", "gazebo"], capture_output=True, text=True)

            if result.returncode == 0:
                print("Gazebo already running, skipping launch")
                return True

            world_file = f"simulation/worlds/{world}.sdf"

            # Launch Gazebo in headless mode for testing
            self.gazebo_process = subprocess.Popen(
                ["gazebo", "--headless-rendering", world_file],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            # Wait for Gazebo to start
            time.sleep(5)

            return self.gazebo_process.poll() is None

        except Exception as e:
            print(f"Failed to launch Gazebo: {e}")
            return False

    def stop_gazebo(self):
        """Stop Gazebo simulation"""
        if self.gazebo_process:
            try:
                os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                self.gazebo_process.wait(timeout=5)
            except:
                try:
                    os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGKILL)
                except:
                    pass
            self.gazebo_process = None

        # Also kill any remaining gazebo processes
        subprocess.run(["pkill", "-f", "gazebo"], capture_output=True)

    def spawn_robot(self, robot_config: dict[str, Any]) -> bool:
        """Spawn a robot in the simulation"""
        try:
            name = robot_config["name"]
            model = robot_config["model"]
            pose = robot_config.get("pose", {})

            x = pose.get("x", 0.0)
            y = pose.get("y", 0.0)
            z = pose.get("z", 0.1)
            yaw = pose.get("yaw", 0.0)

            model_file = f"simulation/models/{model}/model.sdf"

            # Use ROS2 spawn entity
            cmd = [
                "ros2",
                "run",
                "gazebo_ros",
                "spawn_entity.py",
                "-entity",
                name,
                "-file",
                model_file,
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
                "-Y",
                str(yaw),
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode != 0:
                print(f"Failed to spawn robot: {result.stderr}")
                return False

            return True

        except Exception as e:
            print(f"Error spawning robot: {e}")
            return False

    def spawn_obstacle(self, obstacle_config: dict[str, Any]) -> bool:
        """Spawn an obstacle in the simulation"""
        try:
            name = obstacle_config["name"]
            obs_type = obstacle_config["type"]
            pose = obstacle_config.get("pose", {})
            size = obstacle_config.get("size", {})

            x = pose.get("x", 0.0)
            y = pose.get("y", 0.0)
            z = pose.get("z", 0.5)

            # Create temporary SDF for obstacle
            if obs_type == "box":
                sx = size.get("x", 1.0)
                sy = size.get("y", 1.0)
                sz = size.get("z", 1.0)
                geometry = f"""
                <box>
                  <size>{sx} {sy} {sz}</size>
                </box>
                """
            elif obs_type == "cylinder":
                radius = size.get("radius", 0.5)
                length = size.get("length", 1.0)
                geometry = f"""
                <cylinder>
                  <radius>{radius}</radius>
                  <length>{length}</length>
                </cylinder>
                """
            else:
                geometry = "<box><size>1 1 1</size></box>"

            sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          {geometry}
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          {geometry}
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

            temp_sdf = f"/tmp/{name}.sdf"
            with open(temp_sdf, "w") as f:
                f.write(sdf_content)

            cmd = [
                "ros2",
                "run",
                "gazebo_ros",
                "spawn_entity.py",
                "-entity",
                name,
                "-file",
                temp_sdf,
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result.returncode == 0

        except Exception as e:
            print(f"Error spawning obstacle: {e}")
            return False

    def run_scenario(self, scenario_name: str) -> ScenarioResult:
        """Run a single scenario and return results"""
        start_time = time.time()
        errors = []
        logs = []
        metrics = {}

        try:
            # Load scenario
            logs.append(f"Loading scenario: {scenario_name}")
            scenario = self.load_scenario(scenario_name)

            # Extract scenario info
            name = scenario.get("name", scenario_name)
            description = scenario.get("description", "")
            world = scenario.get("world", "empty_warehouse")
            duration = scenario.get("duration", 60)

            logs.append(f"Scenario: {name}")
            logs.append(f"Description: {description}")
            logs.append(f"World: {world}")
            logs.append(f"Duration: {duration}s")

            # Launch Gazebo
            logs.append("Launching Gazebo...")
            if not self.launch_gazebo(world):
                errors.append("Failed to launch Gazebo")
                return ScenarioResult(
                    scenario_name=name,
                    success=False,
                    duration=time.time() - start_time,
                    metrics=metrics,
                    errors=errors,
                    logs=logs,
                )

            # Spawn robots
            robots = scenario.get("robots", [])
            for robot in robots:
                logs.append(f"Spawning robot: {robot['name']}")
                if not self.spawn_robot(robot):
                    errors.append(f"Failed to spawn robot: {robot['name']}")

            # Spawn obstacles
            obstacles = scenario.get("obstacles", [])
            for obstacle in obstacles:
                logs.append(f"Spawning obstacle: {obstacle['name']}")
                if not self.spawn_obstacle(obstacle):
                    errors.append(f"Failed to spawn obstacle: {obstacle['name']}")

            # Run scenario for specified duration
            logs.append(f"Running scenario for {duration}s...")
            time.sleep(min(duration, 5))  # Cap at 5s for testing

            # Collect metrics (placeholder)
            metrics["robots_spawned"] = len(robots)
            metrics["obstacles_spawned"] = len(obstacles)
            metrics["scenario_duration"] = duration

            # Determine success
            success = len(errors) == 0

            logs.append("Scenario completed")

        except Exception as e:
            errors.append(f"Exception: {str(e)}")
            success = False
            logs.append(f"Error: {str(e)}")

        finally:
            # Cleanup
            self.stop_gazebo()

        return ScenarioResult(
            scenario_name=name,
            success=success,
            duration=time.time() - start_time,
            metrics=metrics,
            errors=errors,
            logs=logs,
        )

    def save_result(self, result: ScenarioResult):
        """Save scenario result to file"""
        result_file = self.results_dir / f"{result.scenario_name}_{int(time.time())}.json"

        with open(result_file, "w") as f:
            json.dump(
                {
                    "scenario_name": result.scenario_name,
                    "success": result.success,
                    "duration": result.duration,
                    "metrics": result.metrics,
                    "errors": result.errors,
                    "logs": result.logs,
                    "timestamp": result.timestamp,
                },
                f,
                indent=2,
            )

        return result_file

    def generate_report(self, results: list[ScenarioResult]) -> str:
        """Generate a summary report of all scenario results"""
        report_lines = [
            "=" * 60,
            "SIMULATION TEST REPORT",
            "=" * 60,
            f"Generated: {datetime.now().isoformat()}",
            f"Total Scenarios: {len(results)}",
            "",
        ]

        passed = sum(1 for r in results if r.success)
        failed = len(results) - passed

        report_lines.extend(
            [
                f"PASSED: {passed}",
                f"FAILED: {failed}",
                f"SUCCESS RATE: {passed/len(results)*100:.1f}%" if results else "N/A",
                "",
                "-" * 60,
                "DETAILED RESULTS",
                "-" * 60,
                "",
            ]
        )

        for result in results:
            status = "✓ PASS" if result.success else "✗ FAIL"
            report_lines.append(f"{status} | {result.scenario_name} ({result.duration:.2f}s)")

            if result.errors:
                for error in result.errors:
                    report_lines.append(f"    Error: {error}")

        report_lines.extend(["", "=" * 60, "END OF REPORT", "=" * 60])

        return "\n".join(report_lines)


def main():
    """Main entry point for scenario runner"""
    import argparse

    parser = argparse.ArgumentParser(description="Run simulation scenarios")
    parser.add_argument("--scenario", help="Specific scenario to run")
    parser.add_argument("--all", action="store_true", help="Run all scenarios")
    parser.add_argument("--list", action="store_true", help="List available scenarios")
    parser.add_argument("--output", default="results", help="Output directory for results")

    args = parser.parse_args()

    runner = ScenarioRunner()

    if args.list:
        scenarios = runner.list_scenarios()
        print("Available scenarios:")
        for scenario in scenarios:
            print(f"  - {scenario}")
        return

    if args.all:
        scenarios = runner.list_scenarios()
        results = []

        for scenario in scenarios:
            print(f"\nRunning scenario: {scenario}")
            result = runner.run_scenario(scenario)
            results.append(result)
            runner.save_result(result)

            status = "PASSED" if result.success else "FAILED"
            print(f"Result: {status}")

            if result.errors:
                for error in result.errors:
                    print(f"  Error: {error}")

        # Generate report
        report = runner.generate_report(results)
        print("\n" + report)

        # Save report
        report_file = Path(args.output) / f"report_{int(time.time())}.txt"
        report_file.parent.mkdir(exist_ok=True)
        with open(report_file, "w") as f:
            f.write(report)
        print(f"\nReport saved to: {report_file}")

    elif args.scenario:
        print(f"Running scenario: {args.scenario}")
        result = runner.run_scenario(args.scenario)
        runner.save_result(result)

        status = "PASSED" if result.success else "FAILED"
        print(f"\nResult: {status}")
        print(f"Duration: {result.duration:.2f}s")

        if result.metrics:
            print("\nMetrics:")
            for key, value in result.metrics.items():
                print(f"  {key}: {value}")

        if result.errors:
            print("\nErrors:")
            for error in result.errors:
                print(f"  - {error}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
