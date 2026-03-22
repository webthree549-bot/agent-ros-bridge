"""Unit tests for shadow mode decision comparator."""

from datetime import UTC, datetime, timezone

import pytest

from agent_ros_bridge.shadow import DecisionComparator
from agent_ros_bridge.shadow.models import (
    AIProposal,
    DecisionContext,
    DecisionOutcome,
    DecisionRecord,
    HumanAction,
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
            timestamp=datetime.now(UTC),
        )

    def test_exact_intent_match(self, comparator, sample_context):
        """Test exact intent type match."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(UTC),
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
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[],
            ),
            human_action=HumanAction(
                command="pick_up_object",  # Different intent
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
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[],
            ),
            human_action=HumanAction(
                command="go_to_kitchen",  # "go" is synonym for NAVIGATE
                parameters={},
            ),
        )

        agrees, score = comparator.compare(record)
        assert score >= 0.7  # Should match via synonym

    def test_entity_match(self, comparator, sample_context):
        """Test entity value matching."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[{"type": "LOCATION", "value": "kitchen"}],
            ),
            human_action=HumanAction(
                command="navigate",
                parameters={"location": "kitchen"},  # Same entity value
            ),
        )

        agrees, score = comparator.compare(record)
        # Should have good entity score
        assert score > 0.5

    def test_entity_mismatch(self, comparator, sample_context):
        """Test entity value mismatch."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=AIProposal(
                intent_type="NAVIGATE",
                confidence=0.95,
                entities=[{"type": "LOCATION", "value": "kitchen"}],
            ),
            human_action=HumanAction(
                command="navigate",
                parameters={"location": "office"},  # Different location
            ),
        )

        agrees, score = comparator.compare(record)
        # Entity score should be lower
        assert score < 0.9

    def test_missing_ai_proposal(self, comparator, sample_context):
        """Test comparison with missing AI proposal."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=None,
            human_action=HumanAction(command="navigate", parameters={}),
        )

        agrees, score = comparator.compare(record)
        assert agrees is False
        assert score == 0.0

    def test_missing_human_action(self, comparator, sample_context):
        """Test comparison with missing human action."""
        record = DecisionRecord(
            record_id="test1",
            robot_id="bot1",
            timestamp=datetime.now(UTC),
            context=sample_context,
            ai_proposal=AIProposal(intent_type="NAVIGATE", confidence=0.95),
            human_action=None,
        )

        agrees, score = comparator.compare(record)
        assert agrees is False
        assert score == 0.0

    def test_compare_batch(self, comparator, sample_context):
        """Test batch comparison."""
        records = [
            DecisionRecord(
                record_id=f"test{i}",
                robot_id=f"bot{i}",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(intent_type="NAVIGATE", confidence=0.95),
                human_action=HumanAction(command="navigate", parameters={}),
            )
            for i in range(3)
        ]

        results = comparator.compare_batch(records)
        assert len(results) == 3
        assert all(isinstance(r, tuple) and len(r) == 2 for r in results)

    def test_calculate_metrics(self, comparator, sample_context):
        """Test metrics calculation."""
        # Create records with varying agreement
        records = [
            DecisionRecord(  # Agrees
                record_id="test1",
                robot_id="bot1",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(
                    intent_type="NAVIGATE",
                    confidence=0.95,
                    entities=[],
                ),
                human_action=HumanAction(command="navigate", parameters={}),
            ),
            DecisionRecord(  # Disagrees
                record_id="test2",
                robot_id="bot2",
                timestamp=datetime.now(UTC),
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
        assert "average_score" in metrics
        assert "score_distribution" in metrics

    def test_calculate_metrics_empty(self, comparator):
        """Test metrics with empty list."""
        metrics = comparator.calculate_metrics([])

        assert metrics["total"] == 0
        assert metrics["agreement_rate"] == 0.0
        assert metrics["average_score"] == 0.0

    def test_get_disagreements(self, comparator, sample_context):
        """Test getting disagreements."""
        records = [
            DecisionRecord(  # Agrees
                record_id="test1",
                robot_id="bot1",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(intent_type="NAVIGATE", confidence=0.95),
                human_action=HumanAction(command="navigate", parameters={}),
            ),
            DecisionRecord(  # Disagrees
                record_id="test2",
                robot_id="bot2",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(intent_type="NAVIGATE", confidence=0.95),
                human_action=HumanAction(command="manipulate", parameters={}),
            ),
        ]

        disagreements = comparator.get_disagreements(records)

        assert len(disagreements) == 1
        assert disagreements[0][0].record_id == "test2"

    def test_high_confidence_agreement(self, comparator, sample_context):
        """Test high confidence agreement metric."""
        records = [
            DecisionRecord(  # High confidence, agrees
                record_id="test1",
                robot_id="bot1",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(
                    intent_type="NAVIGATE",
                    confidence=0.98,  # High confidence
                    entities=[],
                ),
                human_action=HumanAction(command="navigate", parameters={}),
            ),
            DecisionRecord(  # Low confidence, agrees
                record_id="test2",
                robot_id="bot2",
                timestamp=datetime.now(UTC),
                context=sample_context,
                ai_proposal=AIProposal(
                    intent_type="NAVIGATE",
                    confidence=0.80,  # Below threshold
                    entities=[],
                ),
                human_action=HumanAction(command="navigate", parameters={}),
            ),
        ]

        metrics = comparator.calculate_metrics(records)

        # Only 1 of 2 is high confidence agreement
        assert metrics["high_confidence_agreement"] == 0.5
