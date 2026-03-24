#!/usr/bin/env python3
"""
Gate 2 Validation Runner - Real Gazebo Edition

Executes Gate 2 validation (10,000 scenarios) in real Gazebo simulation
with Nav2 and TurtleBot3.

Usage:
    python scripts/run_gate2_real_gazebo.py [--batch-size 100] [--max-scenarios 1000]
"""

import asyncio
import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from agent_ros_bridge.simulation.real_gazebo import RealGazeboSimulator, GazeboMetrics
from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator


class Gate2RealValidator:
    """Runs Gate 2 validation in real Gazebo"""
    
    def __init__(self, output_dir: str = "gate2_real_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.simulator: RealGazeboSimulator | None = None
        self.results: list[GazeboMetrics] = []
        
        # Gate 2 thresholds
        self.success_threshold = 0.95
        self.max_collision_rate = 0.05
        self.max_safety_violations = 0
        
    async def setup(self):
        """Initialize simulator"""
        print("=" * 70)
        print("🎯 GATE 2 VALIDATION - REAL GAZEBO EDITION")
        print("=" * 70)
        print()
        print("⚠️  This will run scenarios in REAL Gazebo with physics!")
        print("   Each scenario may take 30-60 seconds to complete.")
        print()
        
        print("🔧 Setting up Real Gazebo Simulator...")
        self.simulator = RealGazeboSimulator(
            world_file="turtlebot3_world",
            robot_model="turtlebot3_burger",
            use_nav2=True,
        )
        
        success = await self.simulator.start()
        if not success:
            raise RuntimeError("Failed to start Real Gazebo Simulator")
        
        print("✅ Simulator ready!")
        print()
    
    async def run_validation(
        self,
        num_scenarios: int = 100,
        batch_size: int = 10,
    ):
        """
        Run Gate 2 validation.
        
        Args:
            num_scenarios: Number of scenarios to run
            batch_size: Scenarios per batch (with checkpointing)
        """
        print(f"🚀 Running {num_scenarios} scenarios in Real Gazebo...")
        print(f"   Batch size: {batch_size}")
        print()
        
        # Generate scenarios
        print("📋 Generating scenarios...")
        generator = ScenarioGenerator(seed=42)
        scenarios = generator.generate_scenarios(num_scenarios)
        print(f"   Generated {len(scenarios)} scenarios")
        print()
        
        # Run scenarios
        start_time = time.time()
        
        for i, scenario in enumerate(scenarios, 1):
            print(f"▶️  Scenario {i}/{num_scenarios}: {scenario['id']}")
            
            try:
                # Execute in real Gazebo
                metrics = await self.simulator.execute_scenario(scenario)
                self.results.append(metrics)
                
                # Print result
                status = "✅ PASS" if metrics.success else "❌ FAIL"
                print(f"   {status} in {metrics.completion_time_sec:.1f}s")
                
                if metrics.collision_count > 0:
                    print(f"   ⚠️  {metrics.collision_count} collisions")
                
                if metrics.error_message:
                    print(f"   ⚠️  Error: {metrics.error_message}")
                
            except Exception as e:
                print(f"   ❌ Exception: {e}")
                # Record failure
                self.results.append(GazeboMetrics(
                    scenario_id=scenario['id'],
                    success=False,
                    completion_time_sec=0.0,
                    path_length_m=0.0,
                    max_deviation_m=0.0,
                    collision_count=0,
                    safety_violations=0,
                    error_message=str(e),
                ))
            
            # Checkpoint every batch
            if i % batch_size == 0:
                self._save_checkpoint(i)
                self._print_progress(i, num_scenarios, start_time)
            
            print()
        
        total_time = time.time() - start_time
        print(f"✅ Validation complete in {total_time/60:.1f} minutes")
        print()
    
    def _save_checkpoint(self, completed: int):
        """Save checkpoint of results"""
        checkpoint_file = self.output_dir / f"checkpoint_{completed}.json"
        
        checkpoint_data = {
            "timestamp": datetime.now().isoformat(),
            "completed_scenarios": completed,
            "results": [
                {
                    "scenario_id": r.scenario_id,
                    "success": r.success,
                    "completion_time_sec": r.completion_time_sec,
                    "collision_count": r.collision_count,
                }
                for r in self.results
            ],
        }
        
        with open(checkpoint_file, 'w') as f:
            json.dump(checkpoint_data, f, indent=2)
    
    def _print_progress(self, completed: int, total: int, start_time: float):
        """Print progress statistics"""
        elapsed = time.time() - start_time
        avg_time = elapsed / completed
        remaining = (total - completed) * avg_time
        
        # Calculate current metrics
        successes = sum(1 for r in self.results if r.success)
        collisions = sum(r.collision_count for r in self.results)
        
        success_rate = successes / len(self.results) if self.results else 0
        collision_rate = collisions / len(self.results) if self.results else 0
        
        print()
        print("📊 Progress Report:")
        print(f"   Completed: {completed}/{total}")
        print(f"   Success Rate: {success_rate*100:.1f}% (target: {self.success_threshold*100:.0f}%)")
        print(f"   Collision Rate: {collision_rate*100:.1f}% (max: {self.max_collision_rate*100:.0f}%)")
        print(f"   Avg Time: {avg_time:.1f}s per scenario")
        print(f"   Est. Remaining: {remaining/60:.1f} minutes")
        print()
    
    def generate_report(self) -> dict:
        """Generate Gate 2 validation report"""
        if not self.results:
            return {"error": "No results to report"}
        
        total = len(self.results)
        successes = sum(1 for r in self.results if r.success)
        collisions = sum(r.collision_count for r in self.results)
        safety_violations = sum(r.safety_violations for r in self.results)
        
        success_rate = successes / total
        collision_rate = collisions / total
        
        avg_time = sum(r.completion_time_sec for r in self.results) / total
        avg_path = sum(r.path_length_m for r in self.results) / total
        
        report = {
            "validation_type": "Gate 2 - Real Gazebo",
            "timestamp": datetime.now().isoformat(),
            "total_scenarios": total,
            "metrics": {
                "success_rate": success_rate,
                "success_rate_percent": f"{success_rate*100:.2f}%",
                "collision_rate": collision_rate,
                "collision_rate_percent": f"{collision_rate*100:.2f}%",
                "safety_violations": safety_violations,
                "avg_completion_time_sec": avg_time,
                "avg_path_length_m": avg_path,
            },
            "thresholds": {
                "min_success_rate": self.success_threshold,
                "max_collision_rate": self.max_collision_rate,
                "max_safety_violations": self.max_safety_violations,
            },
            "passed": (
                success_rate >= self.success_threshold
                and collision_rate <= self.max_collision_rate
                and safety_violations == self.max_safety_violations
            ),
            "details": {
                "successes": successes,
                "failures": total - successes,
                "collisions": collisions,
            },
        }
        
        return report
    
    def save_report(self):
        """Save validation report to file"""
        report = self.generate_report()
        
        report_file = self.output_dir / "gate2_report.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        # Also save CSV for analysis
        csv_file = self.output_dir / "results.csv"
        with open(csv_file, 'w') as f:
            f.write("scenario_id,success,time_sec,path_m,collisions\n")
            for r in self.results:
                f.write(f"{r.scenario_id},{r.success},{r.completion_time_sec:.2f},"
                        f"{r.path_length_m:.2f},{r.collision_count}\n")
        
        return report
    
    def print_report(self):
        """Print validation report"""
        report = self.generate_report()
        
        print("=" * 70)
        print("📊 GATE 2 VALIDATION REPORT - REAL GAZEBO")
        print("=" * 70)
        print()
        print(f"Total Scenarios: {report['total_scenarios']}")
        print()
        print("Results:")
        print(f"  ✅ Success Rate: {report['metrics']['success_rate_percent']}")
        print(f"     (Threshold: ≥{report['thresholds']['min_success_rate']*100:.0f}%)")
        print()
        print(f"  ⚠️  Collision Rate: {report['metrics']['collision_rate_percent']}")
        print(f"     (Threshold: ≤{report['thresholds']['max_collision_rate']*100:.0f}%)")
        print()
        print(f"  🛡️  Safety Violations: {report['metrics']['safety_violations']}")
        print(f"     (Threshold: {report['thresholds']['max_safety_violations']}")
        print()
        print(f"  ⏱️  Avg Time: {report['metrics']['avg_completion_time_sec']:.1f}s")
        print(f"  📏 Avg Path: {report['metrics']['avg_path_length_m']:.1f}m")
        print()
        
        if report['passed']:
            print("🎉 GATE 2 PASSED! System ready for deployment.")
        else:
            print("❌ GATE 2 FAILED. Review failures and re-run.")
        
        print()
        print(f"📁 Results saved to: {self.output_dir}")
        print("=" * 70)
    
    async def cleanup(self):
        """Cleanup simulator"""
        if self.simulator:
            await self.simulator.stop()


async def main():
    parser = argparse.ArgumentParser(
        description="Run Gate 2 validation in Real Gazebo"
    )
    parser.add_argument(
        "--num-scenarios",
        type=int,
        default=10,
        help="Number of scenarios to run (default: 10, use 10000 for full)",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=5,
        help="Checkpoint every N scenarios (default: 5)",
    )
    parser.add_argument(
        "--output-dir",
        default="gate2_real_results",
        help="Output directory for results",
    )
    
    args = parser.parse_args()
    
    validator = Gate2RealValidator(output_dir=args.output_dir)
    
    try:
        await validator.setup()
        await validator.run_validation(
            num_scenarios=args.num_scenarios,
            batch_size=args.batch_size,
        )
        validator.save_report()
        validator.print_report()
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await validator.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
