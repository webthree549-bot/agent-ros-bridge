#!/usr/bin/env python3
"""
Quick 100-scenario test inside Docker container
Validates the pipeline before running full 10K
"""

import sys
sys.path.insert(0, '/workspace')

from agent_ros_bridge.validation.scenario_10k import Scenario10KGenerator
from pathlib import Path

print("🎯 Starting 100-Scenario Validation Test")
print("=" * 50)

# Create generator
gen = Scenario10KGenerator(
    output_dir="/workspace/scenarios/test_100",
    max_workers=4,
)

# Generate 100 scenarios
print("\n📋 Generating 100 scenarios...")
scenario_files = gen.generate(count=100)
print(f"   ✓ Generated {len(scenario_files)} scenarios")

# Execute with progress
print("\n🚀 Executing scenarios...")
def progress(completed, total):
    pct = (completed / total) * 100
    print(f"   Progress: {completed}/{total} ({pct:.1f}%)", end='\r')

results = gen.execute_batch(scenario_files, progress_callback=progress)
print(f"\n   ✓ Completed {len(results)} executions")

# Validate
print("\n✅ Validating results...")
validation = gen.validate_results(results)

print(f"\n📊 Results:")
print(f"   Total Scenarios: {validation['total_scenarios']}")
print(f"   Successful: {validation['successful']}")
print(f"   Failed: {validation['failed']}")
print(f"   Success Rate: {validation['success_rate']*100:.2f}%")
print(f"   Safety Violations: {validation['total_safety_violations']}")
print(f"\n🎯 Gate 2 Test: {'✅ PASSED' if validation['gate2_passed'] else '❌ FAILED'}")

# Generate report
print("\n📝 Generating report...")
report = gen.generate_report(results)
report_path = Path("/workspace/scenarios/test_100/report.html")
report_path.parent.mkdir(parents=True, exist_ok=True)
with open(report_path, 'w') as f:
    f.write(report)
print(f"   ✓ Report saved: {report_path}")

print("\n" + "=" * 50)
print("✅ 100-Scenario Test Complete")
