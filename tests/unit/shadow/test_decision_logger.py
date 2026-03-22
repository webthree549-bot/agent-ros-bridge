"""Unit tests for shadow mode decision logger."""

import json
import os
import tempfile
from datetime import datetime

import pytest

from agent_ros_bridge.shadow import DecisionLogger
from agent_ros_bridge.shadow.models import (
    AIProposal,
    DecisionContext,
    DecisionOutcome,
    DecisionStatus,
    HumanAction,
)


class TestDecisionLogger:
    """Test DecisionLogger functionality."""

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
    def logger(self, temp_db, temp_jsonl):
        """Create logger with temp files."""
        return DecisionLogger(
            db_path=temp_db,
            jsonl_path=temp_jsonl,
            expire_seconds=60.0,
        )

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

    def test_init(self, temp_db, temp_jsonl):
        """Test logger initialization."""
        logger = DecisionLogger(db_path=temp_db, jsonl_path=temp_jsonl)
        assert logger._db_path == temp_db
        assert logger._jsonl_path == temp_jsonl
        assert logger._pending == {}

    def test_log_ai_proposal(self, logger, sample_proposal):
        """Test logging AI proposal."""
        record_id = logger.log_ai_proposal(
            robot_id="bot1",
            proposal=sample_proposal,
        )

        assert record_id is not None
        assert len(record_id) == 36  # UUID length
        assert "bot1" in logger._pending
        assert logger._pending["bot1"].ai_proposal == sample_proposal
        assert logger._total_logged == 1

    def test_log_human_action_with_pending(self, logger, sample_proposal, sample_action):
        """Test logging human action with pending AI proposal."""
        # First log AI proposal
        record_id = logger.log_ai_proposal(
            robot_id="bot1",
            proposal=sample_proposal,
        )

        # Then log human action
        result_id = logger.log_human_action(
            robot_id="bot1",
            action=sample_action,
        )

        assert result_id == record_id
        assert "bot1" not in logger._pending  # Should be removed
        assert logger._total_completed == 1

    def test_log_human_action_without_pending(self, logger, sample_action):
        """Test logging human action without pending AI proposal."""
        result_id = logger.log_human_action(
            robot_id="bot1",
            action=sample_action,
        )

        assert result_id is not None
        assert "bot1" not in logger._pending
        assert logger._total_logged == 1

    def test_log_outcome(self, logger, sample_proposal, sample_action):
        """Test logging outcome."""
        # Create complete record
        logger.log_ai_proposal("bot1", sample_proposal)
        record_id = logger.log_human_action("bot1", sample_action)

        # Log outcome
        outcome = DecisionOutcome(
            success=True,
            execution_time_ms=5000.0,
            metrics={"distance": 5.2},
        )

        success = logger.log_outcome(record_id, outcome)
        assert success is True

    def test_get_pending(self, logger, sample_proposal):
        """Test getting pending records."""
        # Log proposals for two robots
        logger.log_ai_proposal("bot1", sample_proposal)
        logger.log_ai_proposal("bot2", sample_proposal)

        # Get all pending
        all_pending = logger.get_pending()
        assert len(all_pending) == 2

        # Get specific robot
        bot1_pending = logger.get_pending("bot1")
        assert len(bot1_pending) == 1
        assert bot1_pending[0].robot_id == "bot1"

    def test_expire_old_records(self, logger, sample_proposal):
        """Test expiring old records."""
        # Create logger with very short expiry
        logger._expire_seconds = 0.001  # 1ms

        # Log proposal
        logger.log_ai_proposal("bot1", sample_proposal)
        assert "bot1" in logger._pending

        # Wait and expire
        import time
        time.sleep(0.01)  # 10ms

        expired = logger.expire_old_records()
        assert expired == 1
        assert "bot1" not in logger._pending
        assert logger._total_expired == 1

    def test_get_stats(self, logger, sample_proposal, sample_action):
        """Test getting statistics."""
        # Initially empty
        stats = logger.get_stats()
        assert stats["total_logged"] == 0
        assert stats["pending_count"] == 0

        # Log some decisions
        logger.log_ai_proposal("bot1", sample_proposal)
        logger.log_ai_proposal("bot2", sample_proposal)
        logger.log_human_action("bot1", sample_action)

        stats = logger.get_stats()
        assert stats["total_logged"] == 2
        assert stats["total_completed"] == 1
        assert stats["pending_count"] == 1

    def test_sqlite_persistence(self, temp_db, sample_proposal):
        """Test that records are persisted to SQLite."""
        logger = DecisionLogger(db_path=temp_db)
        logger.log_ai_proposal("bot1", sample_proposal)

        # Check database file exists and has data
        import sqlite3
        conn = sqlite3.connect(temp_db)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM decisions")
        count = cursor.fetchone()[0]
        conn.close()

        assert count == 1

    def test_jsonl_persistence(self, temp_jsonl, sample_proposal):
        """Test that records are persisted to JSONL."""
        logger = DecisionLogger(jsonl_path=temp_jsonl)
        logger.log_ai_proposal("bot1", sample_proposal)

        # Read JSONL file
        with open(temp_jsonl) as f:
            lines = f.readlines()

        assert len(lines) == 1
        record = json.loads(lines[0])
        assert record["robot_id"] == "bot1"
        assert record["ai_proposal"]["intent_type"] == "NAVIGATE"


class TestDecisionModels:
    """Test decision model serialization."""

    def test_ai_proposal_roundtrip(self):
        """Test AI proposal serialization."""
        original = AIProposal(
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )

        data = original.to_dict()
        restored = AIProposal.from_dict(data)

        assert restored.intent_type == original.intent_type
        assert restored.confidence == original.confidence
        assert restored.entities == original.entities

    def test_human_action_roundtrip(self):
        """Test human action serialization."""
        original = HumanAction(
            command="navigate_to",
            parameters={"location": "kitchen"},
            operator_id="op1",
        )

        data = original.to_dict()
        restored = HumanAction.from_dict(data)

        assert restored.command == original.command
        assert restored.parameters == original.parameters
        assert restored.operator_id == original.operator_id

    def test_decision_outcome_roundtrip(self):
        """Test outcome serialization."""
        original = DecisionOutcome(
            success=True,
            execution_time_ms=5000.0,
            metrics={"distance": 5.2},
        )

        data = original.to_dict()
        restored = DecisionOutcome.from_dict(data)

        assert restored.success == original.success
        assert restored.execution_time_ms == original.execution_time_ms
        assert restored.metrics == original.metrics
