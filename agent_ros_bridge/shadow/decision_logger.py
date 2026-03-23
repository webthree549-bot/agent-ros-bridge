"""Decision logger for shadow mode operation."""

import json
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .models import DecisionRecord, AIProposal, HumanAction, DecisionOutcome, DecisionContext


class DecisionLogger:
    """Logger for AI-human decision pairs."""

    def __init__(
        self,
        db_path: str | None = None,
        jsonl_path: str | None = None,
        expire_seconds: float = 30.0,
    ):
        """Initialize decision logger."""
        self._db_path = db_path
        self._jsonl_path = jsonl_path
        self._expire_seconds = expire_seconds
        self._pending: dict[str, DecisionRecord] = {}
        self._total_logged = 0
        self._total_completed = 0

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
        conn.commit()
        conn.close()

    def log_ai_proposal(
        self,
        robot_id: str,
        proposal: AIProposal,
        context: DecisionContext | None = None,
    ) -> str:
        """Log an AI proposal."""
        record_id = str(uuid.uuid4())
        timestamp = datetime.now(timezone.utc)

        if context is None:
            context = DecisionContext(timestamp=timestamp)

        record = DecisionRecord(
            record_id=record_id,
            robot_id=robot_id,
            timestamp=timestamp,
            context=context,
            ai_proposal=proposal,
            status="pending",
        )

        self._pending[robot_id] = record
        self._total_logged += 1
        self._persist(record)

        return record_id

    def log_human_action(
        self,
        robot_id: str,
        action: HumanAction,
        outcome: DecisionOutcome | None = None,
    ) -> str | None:
        """Log a human action and match to pending AI proposal."""
        record = self._pending.pop(robot_id, None)

        if record is None:
            record_id = str(uuid.uuid4())
            timestamp = datetime.now(timezone.utc)
            record = DecisionRecord(
                record_id=record_id,
                robot_id=robot_id,
                timestamp=timestamp,
                context=DecisionContext(timestamp=timestamp),
                human_action=action,
                outcome=outcome,
                status="completed",
            )
            self._total_logged += 1
            self._persist(record)
            return record_id

        record.human_action = action
        record.outcome = outcome
        record.status = "completed"
        self._total_completed += 1
        self._persist(record)
        return record.record_id

    def _persist(self, record: DecisionRecord) -> None:
        """Persist record to configured backends."""
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
                    record.status,
                    record.agreement,
                    record.agreement_score,
                )
            )
            conn.commit()
            conn.close()

        if self._jsonl_path:
            with open(self._jsonl_path, "a") as f:
                f.write(json.dumps(record.to_dict()) + "\n")

    def get_pending(self, robot_id: str | None = None) -> list[DecisionRecord]:
        """Get pending decisions."""
        if robot_id:
            record = self._pending.get(robot_id)
            return [record] if record else []
        return list(self._pending.values())

    def expire_old_records(self) -> int:
        """Expire records that have been pending too long."""
        now = datetime.now(timezone.utc)
        expired = []

        for robot_id, record in list(self._pending.items()):
            age = (now - record.timestamp).total_seconds()
            if age > self._expire_seconds:
                record.status = "expired"
                self._persist(record)
                expired.append(robot_id)

        for robot_id in expired:
            del self._pending[robot_id]

        return len(expired)

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

    def get_stats(self) -> dict[str, Any]:
        """Get logger statistics."""
        return {
            "total_logged": self._total_logged,
            "total_completed": self._total_completed,
            "pending_count": len(self._pending),
        }
