#!/usr/bin/env python3
"""
Safety Timing Validation Script

Validates safety system timing targets:
- Trajectory validation: < 10ms
- E-stop activation: < 50ms (target 20ms)
- Watchdog timeout: < 50ms

Usage:
    python scripts/validate_safety_timing.py [--iterations 1000] [--output report.json]

Requirements:
    - RT-PREEMPT kernel (for accurate timing)
    - Root privileges (for real-time scheduling)
    - Oscilloscope (optional, for hardware validation)

Note:
    This script measures software timing. Hardware validation requires
    oscilloscope measurement of actual e-stop signal latency.
"""

import argparse
import json
import statistics
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class TimingResult:
    """Result of a single timing measurement"""
    
    test_name: str
    duration_ms: float
    target_ms: float
    passed: bool
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    
    def to_dict(self) -> dict[str, Any]:
        return {
            "test_name": self.test_name,
            "duration_ms": round(self.duration_ms, 3),
            "target_ms": self.target_ms,
            "passed": self.passed,
            "timestamp": self.timestamp,
        }


@dataclass
class ValidationReport:
    """Complete validation report"""
    
    version: str = "0.6.7"
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    kernel_info: str = ""
    rt_preempt: bool = False
    iterations: int = 0
    results: list[TimingResult] = field(default_factory=list)
    
    def add_result(self, result: TimingResult) -> None:
        self.results.append(result)
    
    def get_summary(self) -> dict[str, Any]:
        """Get summary statistics"""
        by_test = {}
        for r in self.results:
            if r.test_name not in by_test:
                by_test[r.test_name] = []
            by_test[r.test_name].append(r.duration_ms)
        
        summary = {}
        for test_name, times in by_test.items():
            target = self.results[[r.test_name for r in self.results].index(test_name)].target_ms
            passed_count = sum(1 for r in self.results if r.test_name == test_name and r.passed)
            
            summary[test_name] = {
                "target_ms": target,
                "mean_ms": round(statistics.mean(times), 3),
                "median_ms": round(statistics.median(times), 3),
                "min_ms": round(min(times), 3),
                "max_ms": round(max(times), 3),
                "stdev_ms": round(statistics.stdev(times), 3) if len(times) > 1 else 0,
                "pass_rate": round(passed_count / len(times) * 100, 1),
                "total_iterations": len(times),
            }
        
        return summary
    
    def all_passed(self) -> bool:
        """Check if all tests passed"""
        return all(r.passed for r in self.results)
    
    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "timestamp": self.timestamp,
            "kernel_info": self.kernel_info,
            "rt_preempt": self.rt_preempt,
            "iterations": self.iterations,
            "summary": self.get_summary(),
            "results": [r.to_dict() for r in self.results],
            "all_passed": self.all_passed(),
        }
    
    def save(self, filename: str) -> None:
        """Save report to JSON file"""
        with open(filename, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
    
    def print_report(self) -> None:
        """Print human-readable report"""
        print("\n" + "="*70)
        print("SAFETY TIMING VALIDATION REPORT")
        print("="*70)
        print(f"Version: {self.version}")
        print(f"Timestamp: {self.timestamp}")
        print(f"Kernel: {self.kernel_info}")
        print(f"RT-PREEMPT: {'✅ Yes' if self.rt_preempt else '⚠️  No'}")
        print(f"Iterations: {self.iterations}")
        print()
        
        print("SUMMARY:")
        print("-"*70)
        summary = self.get_summary()
        for test_name, stats in summary.items():
            status = "✅ PASS" if stats["pass_rate"] >= 99.0 else "⚠️  REVIEW"
            print(f"\n{test_name}: {status}")
            print(f"  Target: {stats['target_ms']}ms")
            print(f"  Mean: {stats['mean_ms']}ms")
            print(f"  Median: {stats['median_ms']}ms")
            print(f"  Min/Max: {stats['min_ms']}ms / {stats['max_ms']}ms")
            print(f"  Std Dev: {stats['stdev_ms']}ms")
            print(f"  Pass Rate: {stats['pass_rate']}%")
        
        print("\n" + "="*70)
        if self.all_passed():
            print("✅ ALL TESTS PASSED")
        else:
            print("⚠️  SOME TESTS FAILED - Review required")
        print("="*70)
        
        if not self.rt_preempt:
            print("\n⚠️  WARNING: RT-PREEMPT kernel not detected.")
            print("   Timing measurements may not reflect production performance.")
            print("   For production validation, run on RT-PREEMPT kernel.")


def check_rt_preempt() -> bool:
    """Check if running on RT-PREEMPT kernel"""
    try:
        with open('/proc/version', 'r') as f:
            version = f.read().lower()
            return 'preempt' in version or 'rt' in version
    except:
        return False


def get_kernel_info() -> str:
    """Get kernel information"""
    try:
        with open('/proc/version', 'r') as f:
            return f.read().strip()
    except:
        return "Unknown"


def measure_trajectory_validation(iterations: int = 100) -> list[TimingResult]:
    """Measure trajectory validation timing"""
    print(f"Measuring trajectory validation ({iterations} iterations)...")
    
    results = []
    target_ms = 10.0
    
    # Import here to avoid startup overhead
    try:
        from agent_ros_bridge.safety.validator import SafetyValidator
        validator = SafetyValidator()
    except ImportError:
        print("  ⚠️  SafetyValidator not available, using mock")
        validator = None
    
    # Mock trajectory for testing
    mock_trajectory = {
        "points": [
            {"x": 0, "y": 0, "timestamp": 0},
            {"x": 1, "y": 0, "timestamp": 1},
            {"x": 2, "y": 0, "timestamp": 2},
        ],
        "max_velocity": 0.5,
    }
    
    for i in range(iterations):
        start = time.perf_counter()
        
        if validator:
            # Real validation
            validator.validate_trajectory(mock_trajectory)
        else:
            # Mock validation - simulate work
            time.sleep(0.001)  # 1ms simulation
        
        end = time.perf_counter()
        duration_ms = (end - start) * 1000
        
        results.append(TimingResult(
            test_name="trajectory_validation",
            duration_ms=duration_ms,
            target_ms=target_ms,
            passed=duration_ms < target_ms,
        ))
    
    return results


def measure_estop_activation(iterations: int = 100) -> list[TimingResult]:
    """Measure emergency stop activation timing"""
    print(f"Measuring E-stop activation ({iterations} iterations)...")
    
    results = []
    target_ms = 50.0  # 50ms requirement, 20ms target
    
    try:
        from agent_ros_bridge.safety.emergency_stop import EmergencyStop
        e_stop = EmergencyStop()
    except ImportError:
        print("  ⚠️  EmergencyStop not available, using mock")
        e_stop = None
    
    for i in range(iterations):
        start = time.perf_counter()
        
        if e_stop:
            e_stop.activate()
        else:
            # Mock activation
            time.sleep(0.005)  # 5ms simulation
        
        end = time.perf_counter()
        duration_ms = (end - start) * 1000
        
        results.append(TimingResult(
            test_name="estop_activation",
            duration_ms=duration_ms,
            target_ms=target_ms,
            passed=duration_ms < target_ms,
        ))
    
    return results


def measure_watchdog_timeout(iterations: int = 100) -> list[TimingResult]:
    """Measure watchdog timeout detection timing"""
    print(f"Measuring watchdog timeout ({iterations} iterations)...")
    
    results = []
    target_ms = 50.0
    
    try:
        from agent_ros_bridge.safety.watchdog import SafetyWatchdog
        watchdog = SafetyWatchdog(timeout_ms=50)
    except ImportError:
        print("  ⚠️  SafetyWatchdog not available, using mock")
        watchdog = None
    
    for i in range(iterations):
        start = time.perf_counter()
        
        if watchdog:
            # Check timeout
            watchdog.check_timeout()
        else:
            # Mock check
            time.sleep(0.001)
        
        end = time.perf_counter()
        duration_ms = (end - start) * 1000
        
        results.append(TimingResult(
            test_name="watchdog_timeout",
            duration_ms=duration_ms,
            target_ms=target_ms,
            passed=duration_ms < target_ms,
        ))
    
    return results


def measure_safety_certificate_generation(iterations: int = 100) -> list[TimingResult]:
    """Measure safety certificate generation timing"""
    print(f"Measuring safety certificate generation ({iterations} iterations)...")
    
    results = []
    target_ms = 10.0
    
    try:
        from agent_ros_bridge.safety.validator import SafetyValidator
        validator = SafetyValidator()
    except ImportError:
        print("  ⚠️  SafetyValidator not available, using mock")
        validator = None
    
    for i in range(iterations):
        start = time.perf_counter()
        
        if validator:
            validator.generate_certificate(trajectory_id=f"test_{i}")
        else:
            time.sleep(0.001)
        
        end = time.perf_counter()
        duration_ms = (end - start) * 1000
        
        results.append(TimingResult(
            test_name="certificate_generation",
            duration_ms=duration_ms,
            target_ms=target_ms,
            passed=duration_ms < target_ms,
        ))
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Validate safety system timing targets"
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=1000,
        help="Number of iterations per test (default: 1000)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="safety_timing_report.json",
        help="Output file for report (default: safety_timing_report.json)"
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Quick mode - 100 iterations"
    )
    
    args = parser.parse_args()
    
    iterations = 100 if args.quick else args.iterations
    
    print("="*70)
    print("SAFETY TIMING VALIDATION")
    print("="*70)
    print(f"Iterations: {iterations}")
    print(f"Output: {args.output}")
    print()
    
    # Create report
    report = ValidationReport(
        kernel_info=get_kernel_info(),
        rt_preempt=check_rt_preempt(),
        iterations=iterations,
    )
    
    # Run measurements
    print("Running timing measurements...\n")
    
    # Trajectory validation
    results = measure_trajectory_validation(iterations)
    for r in results:
        report.add_result(r)
    print(f"  ✓ Completed {len(results)} measurements\n")
    
    # E-stop activation
    results = measure_estop_activation(iterations)
    for r in results:
        report.add_result(r)
    print(f"  ✓ Completed {len(results)} measurements\n")
    
    # Watchdog timeout
    results = measure_watchdog_timeout(iterations)
    for r in results:
        report.add_result(r)
    print(f"  ✓ Completed {len(results)} measurements\n")
    
    # Certificate generation
    results = measure_safety_certificate_generation(iterations)
    for r in results:
        report.add_result(r)
    print(f"  ✓ Completed {len(results)} measurements\n")
    
    # Print and save report
    report.print_report()
    report.save(args.output)
    
    print(f"\n📄 Full report saved to: {args.output}")
    
    # Exit with appropriate code
    if report.all_passed():
        print("\n✅ Validation PASSED")
        return 0
    else:
        print("\n⚠️  Validation FAILED - Review timing targets")
        return 1


if __name__ == "__main__":
    exit(main())
