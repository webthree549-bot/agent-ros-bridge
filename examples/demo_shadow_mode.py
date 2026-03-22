#!/usr/bin/env python3
"""
Demo: Shadow Mode Decision Logging

This demo shows how to use the shadow mode framework to log
and compare AI proposals with human actions.

Usage:
    python examples/demo_shadow_mode.py
"""

import os
import sys
from datetime import UTC, datetime, timezone

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge.shadow import DecisionComparator, DecisionLogger
from agent_ros_bridge.shadow.models import (
    AIProposal,
    DecisionContext,
    DecisionOutcome,
    HumanAction,
)


def main():
    """Run shadow mode demo."""
    print("=" * 60)
    print("Shadow Mode Decision Logging Demo")
    print("=" * 60)

    # Initialize logger and comparator
    logger = DecisionLogger(
        db_path="/tmp/shadow_demo.db",
        jsonl_path="/tmp/shadow_demo.jsonl",
        expire_seconds=60.0,
    )
    comparator = DecisionComparator()

    # Sample scenarios
    scenarios = [
        {
            "robot_id": "bot1",
            "utterance": "Go to the kitchen",
            "ai_intent": "NAVIGATE",
            "ai_entities": [{"type": "LOCATION", "value": "kitchen"}],
            "human_command": "navigate_to",
            "human_params": {"location": "kitchen"},
            "outcome_success": True,
        },
        {
            "robot_id": "bot2",
            "utterance": "Pick up the cup",
            "ai_intent": "MANIPULATE",
            "ai_entities": [{"type": "OBJECT", "value": "cup"}],
            "human_command": "manipulate_object",
            "human_params": {"object": "cup", "action": "pick"},
            "outcome_success": True,
        },
        {
            "robot_id": "bot3",
            "utterance": "Go to the office",
            "ai_intent": "NAVIGATE",
            "ai_entities": [{"type": "LOCATION", "value": "office"}],
            "human_command": "navigate_to",
            "human_params": {"location": "conference_room"},  # Different!
            "outcome_success": False,
        },
    ]

    completed_records = []

    for i, scenario in enumerate(scenarios, 1):
        print(f"\n--- Scenario {i}: {scenario['utterance']} ---")

        # Create context
        context = DecisionContext(
            robot_pose={"x": float(i), "y": float(i * 2), "theta": 0.0},
            battery_level=85.0,
            current_task="idle",
            timestamp=datetime.now(UTC),
        )

        # Log AI proposal
        ai_proposal = AIProposal(
            intent_type=scenario["ai_intent"],
            confidence=0.92,
            entities=scenario["ai_entities"],
            reasoning=f"User wants to {scenario['ai_intent'].lower()}",
            latency_ms=15.5,
        )

        record_id = logger.log_ai_proposal(
            robot_id=scenario["robot_id"],
            proposal=ai_proposal,
            context=context,
        )
        print(f"✓ AI proposal logged: {scenario['ai_intent']}")

        # Log human action
        human_action = HumanAction(
            command=scenario["human_command"],
            parameters=scenario["human_params"],
            source="manual",
            operator_id="operator_1",
        )

        result_id = logger.log_human_action(
            robot_id=scenario["robot_id"],
            action=human_action,
        )
        print(f"✓ Human action logged: {scenario['human_command']}")

        # Get the completed record
        # (In real usage, you'd query from database)
        from agent_ros_bridge.shadow.models import DecisionRecord

        record = DecisionRecord(
            record_id=result_id,
            robot_id=scenario["robot_id"],
            timestamp=datetime.now(UTC),
            context=context,
            ai_proposal=ai_proposal,
            human_action=human_action,
            outcome=DecisionOutcome(
                success=scenario["outcome_success"],
                execution_time_ms=5000.0,
            ),
        )

        # Compare AI vs Human
        agrees, score = comparator.compare(record)
        record.agreement = agrees
        record.agreement_score = score
        completed_records.append(record)

        print(f"  Agreement: {'✓ YES' if agrees else '✗ NO'} (score: {score:.2f})")
        print(f"  Outcome: {'Success' if scenario['outcome_success'] else 'Failure'}")

    # Calculate metrics
    print("\n" + "=" * 60)
    print("Shadow Mode Metrics")
    print("=" * 60)

    metrics = comparator.calculate_metrics(completed_records)

    print(f"\nTotal Decisions: {metrics['total']}")
    print(f"Agreement Count: {metrics['agreement_count']}")
    print(f"Agreement Rate: {metrics['agreement_rate']:.1%}")
    print(f"Average Score: {metrics['average_score']:.2f}")
    print(f"High Confidence Agreement: {metrics['high_confidence_agreement']:.1%}")

    print("\nScore Distribution:")
    for range_name, count in metrics['score_distribution'].items():
        print(f"  {range_name}: {count}")

    # Show disagreements
    disagreements = comparator.get_disagreements(completed_records)
    if disagreements:
        print("\n--- Disagreements ---")
        for record, score in disagreements:
            print(f"  Robot {record.robot_id}: AI={record.ai_proposal.intent_type}, "
                  f"Human={record.human_action.command} (score: {score:.2f})")

    # Logger stats
    print("\n--- Logger Statistics ---")
    stats = logger.get_stats()
    print(f"  Total logged: {stats['total_logged']}")
    print(f"  Completed: {stats['total_completed']}")
    print(f"  Pending: {stats['pending_count']}")

    # Cleanup
    logger.close()
    print("\n✓ Demo complete!")
    print(f"  Database: {logger._db_path}")
    print(f"  JSONL: {logger._jsonl_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
