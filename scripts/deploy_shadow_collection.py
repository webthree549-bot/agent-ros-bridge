#!/usr/bin/env python3
"""
Deploy Shadow Mode Data Collection

Starts the shadow mode collector and integrates with existing systems.
Run this to begin collecting 200+ hours of AI vs human decision data.
"""

import sys
import time
from pathlib import Path

# Add project to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from agent_ros_bridge.shadow.collector import ShadowModeCollector
from agent_ros_bridge.shadow.hooks import ShadowModeHooks


def main():
    print("=" * 70)
    print("🎯 SHADOW MODE DATA COLLECTION DEPLOYMENT")
    print("=" * 70)
    print()
    print("This will start collecting AI vs human decision data.")
    print("Target: 200+ hours of shadow mode operation")
    print()

    # Configuration
    output_dir = "shadow_data"
    target_hours = 200.0

    print(f"Output Directory: {output_dir}")
    print(f"Target Duration: {target_hours} hours")
    print()

    # Create collector
    collector = ShadowModeCollector(
        output_dir=output_dir,
        target_hours=target_hours,
        checkpoint_interval=3600,  # Save every hour
    )

    # Create hooks that feed into collector
    hooks = ShadowModeHooks(
        enabled=True,
        confidence_threshold=0.0,  # Log all proposals
    )

    # Override hooks methods to also log to collector
    original_on_intent = hooks.on_intent_parsed
    original_on_human = hooks.on_human_command
    original_on_reject = hooks.on_human_rejected
    original_on_modify = hooks.on_human_modified

    def patched_on_intent(robot_id, intent_result):
        result = original_on_intent(robot_id, intent_result)
        collector.on_ai_proposal(
            robot_id=robot_id,
            intent_type=intent_result.get("intent_type", "UNKNOWN"),
            confidence=intent_result.get("confidence", 0.0),
            entities=intent_result.get("entities", []),
            reasoning=intent_result.get("reasoning", ""),
        )
        return result

    def patched_on_human(command):
        result = original_on_human(command)
        # Note: matched_ai_proposal would need to be tracked separately
        collector.on_human_decision(
            robot_id=command.get("robot_id", "unknown"),
            command=command.get("command", "unknown"),
            parameters=command.get("parameters", {}),
            matched_ai_proposal=False,  # Would need actual comparison
        )
        return result

    def patched_on_reject(robot_id, ai_proposal_id, rejection_reason):
        result = original_on_reject(robot_id, ai_proposal_id, rejection_reason)
        collector.on_rejection(robot_id, ai_proposal_id, rejection_reason)
        return result

    def patched_on_modify(robot_id, ai_proposal_id, original, modified):
        result = original_on_modify(robot_id, ai_proposal_id, original, modified)
        collector.on_modification(robot_id, original, modified)
        return result

    # Apply patches
    hooks.on_intent_parsed = patched_on_intent
    hooks.on_human_command = patched_on_human
    hooks.on_human_rejected = patched_on_reject
    hooks.on_human_modified = patched_on_modify

    # Start collection
    print("🚀 Starting collection...")
    print("Press Ctrl+C to stop")
    print()

    collector.start_collection()

    # Print status periodically
    try:
        while collector._running:
            time.sleep(60)  # Update every minute
            status = collector.get_status()
            print(
                f"Progress: {status['hours_elapsed']:.1f}/{target_hours} hours "
                f"({status['percent_complete']:.1f}%) | "
                f"Decisions: {status['total_decisions']}"
            )
    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        collector.stop_collection()

        # Export data
        print("\nExporting data...")
        json_path = collector.export_data("json")
        csv_path = collector.export_data("csv")

        print(f"\n✅ Collection complete!")
        print(f"JSON export: {json_path}")
        print(f"CSV export: {csv_path}")


if __name__ == "__main__":
    main()
