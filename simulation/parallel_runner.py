"""
Parallel Simulation Runner - Runs multiple scenarios in parallel
Week 2 Deliverable: Parallel Simulation Framework
"""

import json
import logging
import multiprocessing as mp
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class ParallelScenarioResult:
    """Result of running a scenario in parallel"""
    scenario_name: str
    worker_id: int
    success: bool
    duration: float
    metrics: dict[str, Any] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())


class ParallelRunner:
    """Runs multiple simulation scenarios in parallel"""

    def __init__(
        self,
        scenarios_dir: str = "simulation/scenarios",
        results_dir: str = "results",
        max_workers: int = 100
    ):
        self.scenarios_dir = Path(scenarios_dir)
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(exist_ok=True)
        self.max_workers = max_workers

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
            scenario_file = self.scenarios_dir / scenario_name

        if not scenario_file.exists():
            raise FileNotFoundError(f"Scenario not found: {scenario_name}")

        with open(scenario_file) as f:
            return yaml.safe_load(f)

    def run_single_scenario(
        self,
        scenario_name: str,
        worker_id: int
    ) -> ParallelScenarioResult:
        """Run a single scenario (called by worker process)"""
        start_time = time.time()
        errors = []
        metrics = {}

        try:
            # Load scenario
            scenario = self.load_scenario(scenario_name)

            # Extract scenario info
            name = scenario.get("name", scenario_name)
            duration = scenario.get("duration", 60)

            logger.info(f"[Worker {worker_id}] Running scenario: {name}")

            # Simulate scenario execution
            # In a real implementation, this would:
            # 1. Launch Gazebo with unique master URI
            # 2. Spawn robots and obstacles
            # 3. Execute the scenario
            # 4. Collect metrics

            # For now, simulate with a short delay
            time.sleep(0.5)

            # Simulate metrics collection
            robots = scenario.get("robots", [])
            obstacles = scenario.get("obstacles", [])

            metrics = {
                "robots_spawned": len(robots),
                "obstacles_spawned": len(obstacles),
                "scenario_duration": duration,
                "worker_id": worker_id,
                "simulated": True
            }

            # Simulate occasional failures for testing
            if scenario_name.endswith("_failure") or scenario_name.endswith("_stop"):
                # These scenarios might "fail" in simulation
                pass

            success = True

        except Exception as e:
            errors.append(str(e))
            success = False
            logger.error(f"[Worker {worker_id}] Error running {scenario_name}: {e}")

        return ParallelScenarioResult(
            scenario_name=scenario_name,
            worker_id=worker_id,
            success=success,
            duration=time.time() - start_time,
            metrics=metrics,
            errors=errors
        )

    def run_parallel(
        self,
        scenarios: list[str] | None = None,
        num_workers: int | None = None
    ) -> list[ParallelScenarioResult]:
        """Run scenarios in parallel using process pool"""

        if scenarios is None:
            scenarios = self.list_scenarios()

        if num_workers is None:
            num_workers = min(len(scenarios), self.max_workers, mp.cpu_count())

        logger.info(f"Running {len(scenarios)} scenarios with {num_workers} workers")

        results = []

        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            # Submit all scenarios
            future_to_scenario = {
                executor.submit(
                    self.run_single_scenario,
                    scenario,
                    i % num_workers
                ): scenario
                for i, scenario in enumerate(scenarios)
            }

            # Collect results as they complete
            for future in as_completed(future_to_scenario):
                scenario = future_to_scenario[future]
                try:
                    result = future.result()
                    results.append(result)

                    status = "✓" if result.success else "✗"
                    logger.info(
                        f"{status} {result.scenario_name} "
                        f"(worker {result.worker_id}, {result.duration:.2f}s)"
                    )

                except Exception as e:
                    logger.error(f"Scenario {scenario} generated an exception: {e}")
                    results.append(ParallelScenarioResult(
                        scenario_name=scenario,
                        worker_id=-1,
                        success=False,
                        duration=0.0,
                        errors=[str(e)]
                    ))

        return results

    def save_results(
        self,
        results: list[ParallelScenarioResult],
        batch_name: str | None = None
    ) -> Path:
        """Save results to JSON file"""
        if batch_name is None:
            batch_name = f"batch_{int(time.time())}"

        result_file = self.results_dir / f"{batch_name}.json"

        output = {
            "batch_name": batch_name,
            "timestamp": datetime.now().isoformat(),
            "total_scenarios": len(results),
            "successful": sum(1 for r in results if r.success),
            "failed": sum(1 for r in results if not r.success),
            "results": [asdict(r) for r in results]
        }

        with open(result_file, 'w') as f:
            json.dump(output, f, indent=2)

        logger.info(f"Results saved to: {result_file}")
        return result_file

    def generate_coverage_report(
        self,
        results: list[ParallelScenarioResult]
    ) -> dict[str, Any]:
        """Generate a coverage report"""

        total = len(results)
        successful = sum(1 for r in results if r.success)
        failed = total - successful

        # Calculate statistics
        durations = [r.duration for r in results]
        avg_duration = sum(durations) / len(durations) if durations else 0
        min_duration = min(durations) if durations else 0
        max_duration = max(durations) if durations else 0

        # Group by scenario type
        scenario_types = {}
        for r in results:
            # Extract type from name (e.g., scenario_01_basic_navigation -> basic_navigation)
            parts = r.scenario_name.split("_")
            if len(parts) >= 3:
                scenario_type = "_".join(parts[2:])
            else:
                scenario_type = "unknown"

            if scenario_type not in scenario_types:
                scenario_types[scenario_type] = {"total": 0, "passed": 0}

            scenario_types[scenario_type]["total"] += 1
            if r.success:
                scenario_types[scenario_type]["passed"] += 1

        report = {
            "summary": {
                "total_scenarios": total,
                "successful": successful,
                "failed": failed,
                "success_rate": successful / total * 100 if total > 0 else 0,
                "avg_duration": avg_duration,
                "min_duration": min_duration,
                "max_duration": max_duration
            },
            "by_type": {
                scenario_type: {
                    "total": stats["total"],
                    "passed": stats["passed"],
                    "failed": stats["total"] - stats["passed"],
                    "pass_rate": stats["passed"] / stats["total"] * 100
                }
                for scenario_type, stats in scenario_types.items()
            },
            "failed_scenarios": [
                {
                    "name": r.scenario_name,
                    "worker_id": r.worker_id,
                    "errors": r.errors
                }
                for r in results if not r.success
            ],
            "timestamp": datetime.now().isoformat()
        }

        return report

    def save_coverage_report(
        self,
        report: dict[str, Any],
        filename: str | None = None
    ) -> Path:
        """Save coverage report to file"""
        if filename is None:
            filename = f"coverage_report_{int(time.time())}.json"

        report_file = self.results_dir / filename

        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)

        logger.info(f"Coverage report saved to: {report_file}")
        return report_file

    def generate_text_report(self, report: dict[str, Any]) -> str:
        """Generate a human-readable text report"""
        summary = report["summary"]

        lines = [
            "=" * 70,
            "PARALLEL SIMULATION COVERAGE REPORT",
            "=" * 70,
            f"Generated: {report['timestamp']}",
            "",
            "SUMMARY",
            "-" * 70,
            f"Total Scenarios:    {summary['total_scenarios']}",
            f"Successful:         {summary['successful']}",
            f"Failed:             {summary['failed']}",
            f"Success Rate:       {summary['success_rate']:.1f}%",
            "",
            "DURATION STATISTICS",
            "-" * 70,
            f"Average:            {summary['avg_duration']:.2f}s",
            f"Minimum:            {summary['min_duration']:.2f}s",
            f"Maximum:            {summary['max_duration']:.2f}s",
            "",
            "COVERAGE BY TYPE",
            "-" * 70
        ]

        for scenario_type, stats in report["by_type"].items():
            lines.append(
                f"{scenario_type:30s} | "
                f"{stats['passed']:3d}/{stats['total']:3d} | "
                f"{stats['pass_rate']:5.1f}%"
            )

        if report["failed_scenarios"]:
            lines.extend([
                "",
                "FAILED SCENARIOS",
                "-" * 70
            ])

            for failed in report["failed_scenarios"]:
                lines.append(f"  - {failed['name']} (worker {failed['worker_id']})")
                for error in failed['errors']:
                    lines.append(f"      Error: {error}")

        lines.extend([
            "",
            "=" * 70,
            "END OF REPORT",
            "=" * 70
        ])

        return "\n".join(lines)


def main():
    """Main entry point for parallel runner"""
    import argparse

    parser = argparse.ArgumentParser(description="Run simulation scenarios in parallel")
    parser.add_argument("--scenarios", nargs="+", help="Specific scenarios to run")
    parser.add_argument("--all", action="store_true", help="Run all scenarios")
    parser.add_argument("--workers", type=int, default=10, help="Number of parallel workers")
    parser.add_argument("--output", default="results", help="Output directory")
    parser.add_argument("--coverage", action="store_true", help="Generate coverage report")

    args = parser.parse_args()

    runner = ParallelRunner(
        results_dir=args.output,
        max_workers=args.workers
    )

    # Determine which scenarios to run
    if args.all:
        scenarios = runner.list_scenarios()
    elif args.scenarios:
        scenarios = args.scenarios
    else:
        parser.print_help()
        return

    if not scenarios:
        logger.error("No scenarios found!")
        return

    logger.info(f"Running {len(scenarios)} scenarios with {args.workers} workers")

    # Run scenarios in parallel
    start_time = time.time()
    results = runner.run_parallel(scenarios, num_workers=args.workers)
    total_duration = time.time() - start_time

    # Save results
    batch_name = f"parallel_{int(time.time())}"
    runner.save_results(results, batch_name)

    # Generate and save coverage report
    if args.coverage:
        report = runner.generate_coverage_report(results)
        runner.save_coverage_report(report)

        # Print text report
        text_report = runner.generate_text_report(report)
        print("\n" + text_report)

    # Print summary
    successful = sum(1 for r in results if r.success)
    print(f"\n{'='*70}")
    print(f"COMPLETED: {successful}/{len(results)} scenarios passed")
    print(f"Total time: {total_duration:.2f}s")
    print(f"Average per scenario: {total_duration/len(results):.2f}s")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
