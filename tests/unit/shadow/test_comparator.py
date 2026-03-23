"""Comprehensive tests for shadow mode decision comparator."""

from datetime import datetime, timezone

import pytest

from agent_ros_bridge.shadow import DecisionComparator
from agent_ros_bridge.shadow.models import (
    DecisionRecord,
    AIProposal,
    HumanAction,
    DecisionContext,
    DecisionOutcome,
)


class TestDecisionComparator:
    """Test DecisionComparator functionality."""

    @pytest.fixture
    def comparator(self):
        """Create comparator instance."""
        return DecisionComparator(
            intent_match_threshold=0.8,
            entity_match_threshold=0.7,
        )

    @pytest.fixture
    def sample_context(self):
        """Create sample context."""
        return DecisionContext(
            robot_pose={"x": 1.0, "y": 2.0, "theta": 0.0},
            battery_level=85.0,
            current_task="idle",
            timestamp=datetime.now(timezone.utc),
        )

    def test_exact_intent_match(self, comparator, sample_context):
        """Test exact intent type match."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(timezone.utc),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[{"type": "LOCATION", "value": "kitchen"}],
            ),
            human_action=HumanAction(
                command="navigate_to_location",
                parameters={"location": "kitchen"},
            ),
        )

        agrees, score = comparator.compare(record)
        assert agrees is True
        assert score >= 0.8

    def test_intent_mismatch(self, comparator, sample_context):
        """Test intent type mismatch."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(timezone.utc),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[],
            ),
            human_action=HumanAction(
                command="pick_up_object",
                parameters={},
            ),
        )

        agrees, score = comparator.compare(record)
        assert agrees is False
        assert score < 0.8

    def test_synonym_match(self, comparator, sample_context):
        """Test intent match via synonym."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(timezone.utc),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[],
            ),
            human_action=HumanAction(
                command="go_to_kitchen",
                parameters={},
            ),
        )

        agrees, score = comparator.compare(record)
        assert score >= 0.7

    def test_calculate_metrics(self, comparator, sample_context):
        """Test metrics calculation."""
        records = [
            DecisionRecord(
                record_id="test1",
                robot_id="bot1",
                timestamp=datetime.now(timezone.utc),
                context=sample_context,
                ai_proposal=AIProposal(
                    intent_type="NAVIGATE",
                    confidence=0.95,
                    entities=[],
                ),
                human_action=HumanAction(command="navigate", parameters={}),
            ),
            DecisionRecord(
                record_id="test2",
                robot_id="bot2",
                timestamp=datetime.now(timezone.utc),
                context=sample_context,
                ai_proposal=AIProposal(
                    intent_type="NAVIGATE",
                    confidence=0.95,
                    entities=[],
                ),
                human_action=HumanAction(command="manipulate", parameters={}),
            ),
        ]

        metrics = comparator.calculate_metrics(records)

        assert metrics["total"] == 2
        assert metrics["agreement_count"] == 1
        assert metrics["agreement_rate"] == 0.5

    def test_calculate_metrics_empty(self, comparator):
        """Test metrics with empty list."""
        metrics = comparator.calculate_metrics([])

        assert metrics["total"] == 0
        assert metrics["agreement_rate"] == 0.0

    def test_missing_ai_proposal(self, comparator, sample_context):
        """Test comparison with missing AI proposal."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(timezone.utc),
            context=sample_context,
            ai_proposal=None,
            human_action=HumanAction(command="navigate", parameters={}),
        )

        agrees, score = comparator.compare(record)
        assert agrees is False
        assert score == 0.0
