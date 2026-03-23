"""Additional tests for shadow mode decision logger."""

import json
import os
import tempfile
from datetime import datetime, timezone

import pytest

from agent_ros_bridge.shadow import DecisionLogger
from agent_ros_bridge.shadow.models import (
    AIProposal,
    DecisionContext,
    DecisionOutcome,
    HumanAction,
)


class TestDecisionLoggerAdvanced:
    """Additional test cases for DecisionLogger."""

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

    def test_sqlite_persistence(self, temp_db):
        """Test that records are persisted to SQLite."""
        logger = DecisionLogger(db_path=temp_db)
        proposal = AIProposal(intent_type="NAVIGATE", confidence=0.95)
        logger.log_ai_proposal("bot1", proposal)

        import sqlite3
        conn = sqlite3.connect(temp_db)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM decisions")
        count = cursor.fetchone()[0]
        conn.close()

        assert count == 1

    def test_jsonl_persistence(self, temp_jsonl):
        """Test that records are persisted to JSONL."""
        logger = DecisionLogger(jsonl_path=temp_jsonl)
        proposal = AIProposal(intent_type="NAVIGATE", confidence=0.95)
        logger.log_ai_proposal("bot1", proposal)

        with open(temp_jsonl) as f:
            lines = f.readlines()

        assert len(lines) == 1
        record = json.loads(lines[0])
        assert record["robot_id"] == "bot1"

    def test_multiple_robots(self, temp_db):
        """Test logging for multiple robots."""
        logger = DecisionLogger(db_path=temp_db)
        proposal = AIProposal(intent_type="NAVIGATE", confidence=0.95)

        logger.log_ai_proposal("bot1", proposal)
        logger.log_ai_proposal("bot2", proposal)
        logger.log_ai_proposal("bot3", proposal)

        pending = logger.get_pending()
        assert len(pending) == 3

    def test_outcome_logging(self, temp_db):
        """Test logging outcomes."""
        logger = DecisionLogger(db_path=temp_db)
        proposal = AIProposal(intent_type="NAVIGATE", confidence=0.95)
        action = HumanAction(command="navigate", parameters={})

        record_id = logger.log_ai_proposal("bot1", proposal)
        logger.log_human_action("bot1", action)

        outcome = DecisionOutcome(success=True, execution_time_ms=5000.0)
        success = logger.log_outcome(record_id, outcome)

        assert success is True
