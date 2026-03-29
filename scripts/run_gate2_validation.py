#!/usr/bin/env python3
"""
Full 10,000-Scenario Gate 2 Validation

This script runs the complete Gate 2 validation:
- Generate 10,000 scenarios
- Execute with parallel workers
- Validate >95% success, 0 safety violations
- Generate reports
"""

import sys
sys.path.insert(0, '/workspace')

from agent_ros_bridge.validation.scenario_10k import Scenario10KGenerator
from pathlib import Path
import time

print("=" * 60)
print("🎯 GATE 2 VALIDATION: 10,000 SCENARIOS")
print("=" * 60)
print("\nRequirements:")
print("  • 10,000 scenarios executed")
print("  • >95% success rate")
print("  • 0 safety violations")
print()

start_time = time.time()

# Create generator
gen = Scenario10KGenerator(
    output_dir="/workspace/scenarios/gate2_10k",
    max_workers=8,
)

# Generate 10K scenarios
print("📋 STEP 1: Generating 10,000 scenarios...")
print("-" * 60)
scenario_files = gen.generate(count=10000)
print(f"✓ Generated {len(scenario_files):,} scenarios")
print()

# Execute with progress
print("🚀 STEP 2: Executing scenarios...")
print("-" * 60)
last_pct = 0

def progress(completed, total):
    global last_pct
    pct = int((completed / total) * 100)
    if pct != last_pct and pct % 5 == 0:
        elapsed = time.time() - start_time
        eta = (elapsed / completed) * (total - completed) if completed > 0 else 0
        print(f"   Progress: {completed:,}/{total:,} ({pct}%) - ETA: {eta/60:.1f}min")
        last_pct = pct

results = gen.execute_batch(scenario_files, progress_callback=progress)
print(f"✓ Completed {len(results):,} executions")
print()

# Validate
print("✅ STEP 3: Validating Gate 2 criteria...")
print("-" * 60)
validation = gen.validate_results(results)

print(f"\n📊 VALIDATION RESULTS:")
print(f"   Total Scenarios: {validation['total_scenarios']:,}")
print(f"   Successful:      {validation['successful']:,}")
print(f"   Failed:          {validation['failed']:,}")
print(f"   Success Rate:    {validation['success_rate']*100:.2f}%")
print(f"   Collisions:      {validation['total_collisions']}")
print(f"   Safety Viol:     {validation['total_safety_violations']}")
print()

# Gate 2 status
if validation['gate2_passed']:
    print("╔" + "═" * 58 + "╗")
    print("║" + "🎉 GATE 2: PASSED 🎉".center(58) + "║")
    print("╚" + "═" * 58 + "╝")
else:
    print("╔" + "═" * 58 + "╗")
    print("║" + "❌ GATE 2: FAILED ❌".center(58) + "║")
    print("╚" + "═" * 58 + "╝")
    if not validation['safety_passed']:
        print("   Reason: Safety violations detected!")
    if validation['success_rate'] < 0.95:
        print(f"   Reason: Success rate {validation['success_rate']*100:.1f}% < 95%")

# Generate reports
print("\n📝 STEP 4: Generating reports...")
print("-" * 60)

html_report = gen.generate_report(results)
json_report = gen.generate_json_report(results)

report_dir = Path("/workspace/scenarios/gate2_10k/reports")
report_dir.mkdir(parents=True, exist_ok=True)

html_path = report_dir / 'gate2_report.html'
json_path = report_dir / 'gate2_report.json'

with open(html_path, 'w') as f:
    f.write(html_report)

with open(json_path, 'w') as f:
    import json
    json.dump(json_report, f, indent=2)

print(f"✓ HTML Report: {html_path}")
print(f"✓ JSON Report: {json_path}")

# Summary
elapsed = time.time() - start_time
print()
print("=" * 60)
print(f"✅ VALIDATION COMPLETE")
print(f"   Time: {elapsed/60:.1f} minutes")
print(f"   Rate: {validation['success_rate']*100:.2f}% success")
print(f"   Gate 2: {'PASSED ✅' if validation['gate2_passed'] else 'FAILED ❌'}")
print("=" * 60)
