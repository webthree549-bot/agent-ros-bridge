#!/usr/bin/env python3
"""Demo: Shadow Mode Decision Logging

Usage:
    python examples/demo_shadow_mode.py
"""

import os
import sys
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge.shadow import DecisionLogger, DecisionComparator
from agent_ros_bridge.shadow.models import (
    AIProposal,
    HumanAction,
    DecisionContext,
    DecisionOutcome,
)


def main():
    """Run shadow mode demo."""
    print("=" * 60)
    print("Shadow Mode Decision Logging Demo")
    print("=" * 60)

    logger = DecisionLogger(
        db_path="/tmp/shadow_demo.db",
        jsonl_path="/tmp/shadow_demo.jsonl",
    )
    comparator = DecisionComparator()

    scenarios = [
        {
            "robot_id": "bot1",
            "ai_intent": "NAVIGATE",
            "ai_entities": [{"type": "LOCATION", "value": "kitchen"}],
            "human_command": "navigate_to",
            "human_params": {"location": "kitchen"},
            "outcome_success": True,
        },
        {
            "robot_id": "bot2",
            "ai_intent": "MANIPULATE",
            "ai_entities": [{"type": "OBJECT", "value": "cup"}],
            "human_command": "manipulate_object",
            "human_params": {"object": "cup"},
            "outcome_success": True,
        },
        {
            "robot_id": "bot3",
            "ai_intent": "NAVIGATE",
            "ai_entities": [{"type": "LOCATION", "value": "office"}],
            "human_command": "navigate_to",
            "human_params": {"location": "conference_room"},
            "outcome_success": False,
        },
    ]

    completed_records = []

    for i, scenario in enumerate(scenarios, 1):
        print(f"\n--- Scenario {i} ---")

        context = DecisionContext(
            robot_pose={"x": float(i), "y": float(i * 2), "theta": 0.0},
            battery_level=85.0,
            timestamp=datetime.now(timezone.utc),
        )

        ai_proposal = AIProposal(
            intent_type=scenario["ai_intent"],
            confidence=0.92,
            entities=scenario["ai_entities"],
        )

        record_id = logger.log_ai_proposal(
            robot_id=scenario["robot_id"],
            proposal=ai_proposal,
            context=context,
        )
        print(f"✓ AI proposal logged: {scenario['ai_intent']}")

        human_action = HumanAction(
            command=scenario["human_command"],
            parameters=scenario["human_params"],
        )

        logger.log_human_action(
            robot_id=scenario["robot_id"],
            action=human_action,
        )
        print(f"✓ Human action logged: {scenario['human_command']}")

        from agent_ros_bridge.shadow.models import DecisionRecord

        record = DecisionRecord(
            record_id=record_id,
            robot_id=scenario["robot_id"],
            timestamp=datetime.now(timezone.utc),
            context=context,
            ai_proposal=ai_proposal,
            human_action=human_action,
            outcome=DecisionOutcome(success=scenario["outcome_success"]),
        )

        agrees, score = comparator.compare(record)
        record.agreement = agrees
        record.agreement_score = score
        completed_records.append(record)

        print(f"  Agreement: {'✓ YES' if agrees else '✗ NO'} (score: {score:.2f})")

    print("\n" + "=" * 60)
    print("Metrics")
    print("=" * 60)

    metrics = comparator.calculate_metrics(completed_records)
    print(f"\nTotal: {metrics['total']}")
    print(f"Agreement Rate: {metrics['agreement_rate']:.1%}")
    print(f"Average Score: {metrics['average_score']:.2f}")

    stats = logger.get_stats()
    print(f"\nLogged: {stats['total_logged']}")
    print(f"Completed: {stats['total_completed']}")

    print("\n✓ Demo complete!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
