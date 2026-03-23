"""TDD Unit tests for shadow mode decision logger - RED PHASE.

These tests define the expected behavior before implementation exists.
Run these first - they should all FAIL.
"""

import json
import os
import tempfile
from datetime import datetime, timezone

import pytest

# These imports will fail until we implement the modules
from agent_ros_bridge.shadow import DecisionLogger
from agent_ros_bridge.shadow.models import (
    AIProposal,
    HumanAction,
    DecisionContext,
    DecisionOutcome,
    DecisionStatus,
)


class TestDecisionLoggerTDD:
    """TDD tests for DecisionLogger - write these first, watch them fail."""

    @pytest.fixture
    def temp_db(self):
        """Create temporary database file."""
        fd, path = tempfile.mkstemp(suffix=".db")
        os.close(fd)
        yield path
        os.unlink(path)

    @pytest.fixture
    def temp_jsonl(self):
        """Create temporary JSONL file."""
        fd, path = tempfile.mkstemp(suffix=".jsonl")
        os.close(fd)
        yield path
        os.unlink(path)

    @pytest.fixture
    def sample_proposal(self):
        """Create sample AI proposal."""
        return AIProposal(
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
            reasoning="User wants to go to kitchen",
            latency_ms=12.5,
        )

    @pytest.fixture
    def sample_action(self):
        """Create sample human action."""
        return HumanAction(
            command="navigate_to",
            parameters={"location": "kitchen", "speed": "normal"},
            source="manual",
            operator_id="operator_1",
        )

    def test_logger_initializes_with_db_path(self, temp_db):
        """RED: DecisionLogger should accept db_path parameter."""
        logger = DecisionLogger(db_path=temp_db)
        assert logger._db_path == temp_db

    def test_logger_initializes_with_jsonl_path(self, temp_jsonl):
        """RED: DecisionLogger should accept jsonl_path parameter."""
        logger = DecisionLogger(jsonl_path=temp_jsonl)
        assert logger._jsonl_path == temp_jsonl

    def test_log_ai_proposal_returns_record_id(self, temp_db, sample_proposal):
        """RED: log_ai_proposal should return a unique record ID."""
        logger = DecisionLogger(db_path=temp_db)
        record_id = logger.log_ai_proposal(
            robot_id="bot1",
            proposal=sample_proposal,
        )
        assert record_id is not None
        assert len(record_id) > 0

    def test_log_ai_proposal_stores_pending(self, temp_db, sample_proposal):
        """RED: log_ai_proposal should store decision as pending."""
        logger = DecisionLogger(db_path=temp_db)
        logger.log_ai_proposal(robot_id="bot1", proposal=sample_proposal)
        
        pending = logger.get_pending("bot1")
        assert len(pending) == 1
        assert pending[0].robot_id == "bot1"

    def test_log_human_action_matches_pending(self, temp_db, sample_proposal, sample_action):
        """RED: log_human_action should match to pending AI proposal."""
        logger = DecisionLogger(db_path=temp_db)
        record_id = logger.log_ai_proposal(robot_id="bot1", proposal=sample_proposal)
        
        result_id = logger.log_human_action(robot_id="bot1", action=sample_action)
        
        assert result_id == record_id

    def test_log_human_action_creates_orphan_when_no_pending(self, temp_db, sample_action):
        """RED: log_human_action should create orphan record when no pending AI."""
        logger = DecisionLogger(db_path=temp_db)
        
        result_id = logger.log_human_action(robot_id="bot1", action=sample_action)
        
        assert result_id is not None

    def test_get_pending_returns_all_when_no_filter(self, temp_db, sample_proposal):
        """RED: get_pending with no filter should return all pending."""
        logger = DecisionLogger(db_path=temp_db)
        logger.log_ai_proposal("bot1", sample_proposal)
        logger.log_ai_proposal("bot2", sample_proposal)
        
        all_pending = logger.get_pending()
        assert len(all_pending) == 2

    def test_expire_old_records_removes_stale(self, temp_db, sample_proposal):
        """RED: expire_old_records should remove stale pending decisions."""
        logger = DecisionLogger(db_path=temp_db, expire_seconds=0.001)
        logger.log_ai_proposal("bot1", sample_proposal)
        
        import time
        time.sleep(0.01)
        
        expired = logger.expire_old_records()
        assert expired == 1
        assert len(logger.get_pending()) == 0

    def test_get_stats_returns_correct_counts(self, temp_db, sample_proposal, sample_action):
        """RED: get_stats should return accurate statistics."""
        logger = DecisionLogger(db_path=temp_db)
        
        # Initially empty
        stats = logger.get_stats()
        assert stats["total_logged"] == 0
        
        # After logging
        logger.log_ai_proposal("bot1", sample_proposal)
        logger.log_human_action("bot1", sample_action)
        
        stats = logger.get_stats()
        assert stats["total_logged"] == 1
        assert stats["total_completed"] == 1


class TestDecisionModelsTDD:
    """TDD tests for data models - serialization roundtrips."""

    def test_ai_proposal_to_dict(self):
        """RED: AIProposal should serialize to dict."""
        proposal = AIProposal(
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )
        data = proposal.to_dict()
        assert data["intent_type"] == "NAVIGATE"
        assert data["confidence"] == 0.95

    def test_ai_proposal_from_dict(self):
        """RED: AIProposal should deserialize from dict."""
        data = {
            "intent_type": "NAVIGATE",
            "confidence": 0.95,
            "entities": [],
            "reasoning": "test",
            "latency_ms": 10.0,
            "model_version": "v1",
        }
        proposal = AIProposal.from_dict(data)
        assert proposal.intent_type == "NAVIGATE"
        assert proposal.confidence == 0.95

    def test_human_action_roundtrip(self):
        """RED: HumanAction should roundtrip through dict."""
        original = HumanAction(
            command="navigate_to",
            parameters={"location": "kitchen"},
            operator_id="op1",
        )
        data = original.to_dict()
        restored = HumanAction.from_dict(data)
        
        assert restored.command == original.command
        assert restored.parameters == original.parameters

    def test_decision_context_has_timestamp(self):
        """RED: DecisionContext should have timestamp field."""
        context = DecisionContext(timestamp=datetime.now(timezone.utc))
        assert context.timestamp is not None
