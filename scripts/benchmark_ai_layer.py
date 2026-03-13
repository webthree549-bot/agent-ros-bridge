#!/usr/bin/env python3
"""
Performance Benchmark for v0.6.1 AI Layer
Week 3 - Performance testing and optimization validation

Usage:
    python scripts/benchmark_ai_layer.py
    python scripts/benchmark_ai_layer.py --iterations 1000
    python scripts/benchmark_ai_layer.py --output results.json
"""

import argparse
import json
import statistics
import time
from datetime import datetime
from typing import Any

# Check ROS2 availability
try:
    import rclpy

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("WARNING: ROS2 not available. Benchmarks will be skipped.")


def benchmark_intent_parser(iterations: int = 100) -> dict[str, Any]:
    """Benchmark intent parser performance."""

    if not ROS2_AVAILABLE:
        return {"error": "ROS2 not available"}

    try:
        from agent_ros_bridge.ai.intent_parser import IntentParserNode
        from agent_ros_bridge_msgs.srv import ParseIntent

        if not rclpy.ok():
            rclpy.init()

        parser = IntentParserNode()

        test_utterances = [
            "go to kitchen",
            "pick up the cup",
            "navigate to the living room",
            "stop",
            "what is your status",
            "grab the bottle",
            "move to the bedroom",
            "emergency stop",
        ]

        latencies = []

        for i in range(iterations):
            utterance = test_utterances[i % len(test_utterances)]

            request = ParseIntent.Request()
            request.utterance = utterance

            start = time.time()
            response = ParseIntent.Response()
            parser.parse_intent_callback(request, response)
            latency = (time.time() - start) * 1000

            latencies.append(latency)

        parser.destroy_node()

        # Calculate statistics
        latencies.sort()
        results = {
            "component": "intent_parser",
            "iterations": iterations,
            "avg_ms": statistics.mean(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p50_ms": latencies[int(len(latencies) * 0.5)],
            "p95_ms": latencies[int(len(latencies) * 0.95)],
            "p99_ms": latencies[int(len(latencies) * 0.99)],
            "std_dev_ms": statistics.stdev(latencies) if len(latencies) > 1 else 0,
            "target_ms": 10.0,
            "target_met": latencies[int(len(latencies) * 0.95)] < 10.0,
        }

        return results

    except Exception as e:
        return {"error": str(e)}


def benchmark_safety_validator(iterations: int = 100) -> dict[str, Any]:
    """Benchmark safety validator performance."""

    if not ROS2_AVAILABLE:
        return {"error": "ROS2 not available"}

    try:
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        validator = SafetyValidatorNode()

        # Test trajectories
        safe_trajectory = {
            "waypoints": [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 1.0, "y": 0.0, "z": 0.0}],
            "velocities": [0.5, 0.5],
            "accelerations": [0.1, 0.1],
        }

        limits = {
            "max_velocity": 1.0,
            "max_acceleration": 2.0,
            "workspace_bounds": {
                "x_min": -5.0,
                "x_max": 5.0,
                "y_min": -5.0,
                "y_max": 5.0,
                "z_min": 0.0,
                "z_max": 2.0,
            },
        }

        latencies = []

        for _ in range(iterations):
            start = time.time()
            result = validator.validate_trajectory(safe_trajectory, limits)
            latency = (time.time() - start) * 1000
            latencies.append(latency)

        # Calculate statistics
        latencies.sort()
        results = {
            "component": "safety_validator",
            "iterations": iterations,
            "avg_ms": statistics.mean(latencies),
            "min_ms": min(latencies),
            "max_ms": max(latencies),
            "p50_ms": latencies[int(len(latencies) * 0.5)],
            "p95_ms": latencies[int(len(latencies) * 0.95)],
            "p99_ms": latencies[int(len(latencies) * 0.99)],
            "std_dev_ms": statistics.stdev(latencies) if len(latencies) > 1 else 0,
            "target_ms": 10.0,
            "target_met": latencies[int(len(latencies) * 0.95)] < 10.0,
        }

        return results

    except Exception as e:
        return {"error": str(e)}


def print_results(results: dict[str, Any]):
    """Print benchmark results in a formatted table."""

    print("\n" + "=" * 70)
    print(f"BENCHMARK RESULTS: {results['component']}")
    print("=" * 70)

    if "error" in results:
        print(f"ERROR: {results['error']}")
        return

    print(f"Iterations: {results['iterations']}")
    print(f"Target: <{results['target_ms']}ms (p95)")
    print("-" * 70)
    print(f"  Average:  {results['avg_ms']:>8.3f} ms")
    print(f"  Min:      {results['min_ms']:>8.3f} ms")
    print(f"  Max:      {results['max_ms']:>8.3f} ms")
    print(f"  p50:      {results['p50_ms']:>8.3f} ms")
    print(f"  p95:      {results['p95_ms']:>8.3f} ms")
    print(f"  p99:      {results['p99_ms']:>8.3f} ms")
    print(f"  Std Dev:  {results['std_dev_ms']:>8.3f} ms")
    print("-" * 70)

    status = "✅ PASS" if results["target_met"] else "❌ FAIL"
    print(f"  Target Met: {status}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Benchmark v0.6.1 AI Layer")
    parser.add_argument("--iterations", type=int, default=100, help="Number of iterations")
    parser.add_argument("--output", type=str, help="Output file for JSON results")
    parser.add_argument(
        "--component",
        type=str,
        choices=["intent", "safety", "all"],
        default="all",
        help="Component to benchmark",
    )

    args = parser.parse_args()

    print("\n🔍 Agent ROS Bridge v0.6.1 - AI Layer Performance Benchmark")
    print(f"Started: {datetime.now().isoformat()}")
    print(f"Iterations: {args.iterations}")

    all_results = {}

    if args.component in ["intent", "all"]:
        print("\n📊 Benchmarking Intent Parser...")
        intent_results = benchmark_intent_parser(args.iterations)
        print_results(intent_results)
        all_results["intent_parser"] = intent_results

    if args.component in ["safety", "all"]:
        print("\n📊 Benchmarking Safety Validator...")
        safety_results = benchmark_safety_validator(args.iterations)
        print_results(safety_results)
        all_results["safety_validator"] = safety_results

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    all_passed = all(r.get("target_met", False) for r in all_results.values() if "error" not in r)

    for name, results in all_results.items():
        if "error" in results:
            print(f"  {name}: ERROR - {results['error']}")
        else:
            status = "✅ PASS" if results["target_met"] else "❌ FAIL"
            print(
                f"  {name}: {status} (p95: {results['p95_ms']:.2f}ms, target: <{results['target_ms']}ms)"
            )

    overall = "✅ ALL TARGETS MET" if all_passed else "❌ SOME TARGETS MISSED"
    print(f"\n  Overall: {overall}")
    print("=" * 70)

    # Save to file if requested
    if args.output:
        output_data = {
            "timestamp": datetime.now().isoformat(),
            "iterations": args.iterations,
            "results": all_results,
        }
        with open(args.output, "w") as f:
            json.dump(output_data, f, indent=2)
        print(f"\n💾 Results saved to: {args.output}")


if __name__ == "__main__":
    main()
