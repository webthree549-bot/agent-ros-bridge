"""Decision logger for shadow mode operation.

Logs AI proposals and human actions for later comparison.
Supports SQLite and JSONL backends.
"""

import json
import sqlite3
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .models import (
    AIProposal,
    DecisionContext,
    DecisionOutcome,
    DecisionRecord,
    DecisionStatus,
    HumanAction,
)


class DecisionLogger:
    """Logger for AI-human decision pairs.

    Usage:
        logger = DecisionLogger()

        # Log AI proposal
        logger.log_ai_proposal(
            robot_id="bot1",
            proposal=ai_proposal,
            context=context
        )

        # Log human action (matches to pending AI proposal)
        logger.log_human_action(
            robot_id="bot1",
            action=human_action
        )

        # Log outcome
        logger.log_outcome(
            record_id=record_id,
            outcome=outcome
        )
    """

    def __init__(
        self,
        db_path: str | None = None,
        jsonl_path: str | None = None,
        expire_seconds: float = 30.0,
    ):
        """Initialize decision logger.

        Args:
            db_path: Path to SQLite database (optional)
            jsonl_path: Path to JSONL file (optional)
            expire_seconds: How long to wait for human action before expiring
        """
        self._db_path = db_path
        self._jsonl_path = jsonl_path
        self._expire_seconds = expire_seconds

        # In-memory pending decisions (waiting for human action)
        self._pending: dict[str, DecisionRecord] = {}

        # Statistics
        self._total_logged = 0
        self._total_completed = 0
        self._total_expired = 0

        # Initialize backends
        if db_path:
            self._init_sqlite()
        if jsonl_path:
            Path(jsonl_path).parent.mkdir(parents=True, exist_ok=True)

    def _init_sqlite(self) -> None:
        """Initialize SQLite database."""
        Path(self._db_path).parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(self._db_path)
        cursor = conn.cursor()

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS decisions (
                record_id TEXT PRIMARY KEY,
                robot_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                context TEXT,
                ai_proposal TEXT,
                human_action TEXT,
                outcome TEXT,
                status TEXT,
                agreement BOOLEAN,
                agreement_score REAL
            )
        """)

        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_robot_id ON decisions(robot_id)
        """)

        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_timestamp ON decisions(timestamp)
        """)

        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_status ON decisions(status)
        """)

        conn.commit()
        conn.close()

    def log_ai_proposal(
        self,
        robot_id: str,
        proposal: AIProposal,
        context: DecisionContext | None = None,
    ) -> str:
        """Log an AI proposal.

        Args:
            robot_id: Unique robot identifier
            proposal: AI decision proposal
            context: Decision context (optional)

        Returns:
            record_id: Unique ID for this decision record
        """
        record_id = str(uuid.uuid4())
        timestamp = datetime.now(UTC)

        if context is None:
            from .models import DecisionContext
            context = DecisionContext(timestamp=timestamp)

        record = DecisionRecord(
            record_id=record_id,
            robot_id=robot_id,
            timestamp=timestamp,
            context=context,
            ai_proposal=proposal,
            status=DecisionStatus.PENDING,
        )

        # Store in pending
        self._pending[robot_id] = record
        self._total_logged += 1

        # Persist
        self._persist(record)

        return record_id

    def log_human_action(
        self,
        robot_id: str,
        action: HumanAction,
        outcome: DecisionOutcome | None = None,
    ) -> str | None:
        """Log a human action and match to pending AI proposal.

        Args:
            robot_id: Unique robot identifier
            action: Human action taken
            outcome: Execution outcome (optional)

        Returns:
            record_id: ID of matched record, or None if no pending proposal
        """
        # Find pending record for this robot
        record = self._pending.pop(robot_id, None)

        if record is None:
            # No pending AI proposal - create orphan record
            record_id = str(uuid.uuid4())
            timestamp = datetime.now(UTC)
            record = DecisionRecord(
                record_id=record_id,
                robot_id=robot_id,
                timestamp=timestamp,
                context=DecisionContext(timestamp=timestamp),
                human_action=action,
                outcome=outcome,
                status=DecisionStatus.COMPLETED,
            )
            self._total_logged += 1
            self._persist(record)
            return record_id

        # Update existing record
        record.human_action = action
        record.outcome = outcome
        record.status = DecisionStatus.COMPLETED
        self._total_completed += 1

        # Calculate agreement (will be done by comparator)
        # For now, just mark as completed

        self._persist(record)
        return record.record_id

    def log_outcome(
        self,
        record_id: str,
        outcome: DecisionOutcome,
    ) -> bool:
        """Log outcome for an existing record.

        Args:
            record_id: Decision record ID
            outcome: Execution outcome

        Returns:
            success: True if record was found and updated
        """
        # Update in SQLite if configured
        if self._db_path:
            conn = sqlite3.connect(self._db_path)
            cursor = conn.cursor()

            cursor.execute(
                "UPDATE decisions SET outcome = ? WHERE record_id = ?",
                (json.dumps(outcome.to_dict()), record_id)
            )

            conn.commit()
            conn.close()
            return True

        return False

    def update_agreement(
        self,
        record_id: str,
        agreement: bool,
        agreement_score: float,
    ) -> bool:
        """Update agreement status for a record.

        Args:
            record_id: Decision record ID
            agreement: True if AI and human agreed
            agreement_score: Similarity score (0.0-1.0)

        Returns:
            success: True if record was updated
        """
        if self._db_path:
            conn = sqlite3.connect(self._db_path)
            cursor = conn.cursor()

            cursor.execute(
                "UPDATE decisions SET agreement = ?, agreement_score = ? WHERE record_id = ?",
                (agreement, agreement_score, record_id)
            )

            conn.commit()
            conn.close()
            return True

        return False

    def _persist(self, record: DecisionRecord) -> None:
        """Persist record to configured backends."""
        # SQLite
        if self._db_path:
            conn = sqlite3.connect(self._db_path)
            cursor = conn.cursor()

            cursor.execute(
                """
                INSERT OR REPLACE INTO decisions (
                    record_id, robot_id, timestamp, context,
                    ai_proposal, human_action, outcome,
                    status, agreement, agreement_score
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    record.record_id,
                    record.robot_id,
                    record.timestamp.isoformat(),
                    json.dumps(record.context.to_dict()),
                    json.dumps(record.ai_proposal.to_dict()) if record.ai_proposal else None,
                    json.dumps(record.human_action.to_dict()) if record.human_action else None,
                    json.dumps(record.outcome.to_dict()) if record.outcome else None,
                    record.status.value,
                    record.agreement,
                    record.agreement_score,
                )
            )

            conn.commit()
            conn.close()

        # JSONL
        if self._jsonl_path:
            with open(self._jsonl_path, "a") as f:
                f.write(json.dumps(record.to_dict()) + "\n")

    def get_pending(self, robot_id: str | None = None) -> list[DecisionRecord]:
        """Get pending decisions.

        Args:
            robot_id: Filter by robot (optional)

        Returns:
            List of pending records
        """
        if robot_id:
            record = self._pending.get(robot_id)
            return [record] if record else []
        return list(self._pending.values())

    def expire_old_records(self) -> int:
        """Expire records that have been pending too long.

        Returns:
            count: Number of records expired
        """
        now = datetime.now(UTC)
        expired = []

        for robot_id, record in list(self._pending.items()):
            age = (now - record.timestamp).total_seconds()
            if age > self._expire_seconds:
                record.status = DecisionStatus.EXPIRED
                self._persist(record)
                expired.append(robot_id)

        for robot_id in expired:
            del self._pending[robot_id]

        self._total_expired += len(expired)
        return len(expired)

    def get_stats(self) -> dict[str, Any]:
        """Get logger statistics."""
        return {
            "total_logged": self._total_logged,
            "total_completed": self._total_completed,
            "total_expired": self._total_expired,
            "pending_count": len(self._pending),
        }

    def close(self) -> None:
        """Close logger and expire remaining records."""
        self.expire_old_records()
