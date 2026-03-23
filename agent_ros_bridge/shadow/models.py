"""Data models for shadow mode decision tracking."""

from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum
from typing import Any


class DecisionStatus(Enum):
    """Status of a decision record."""

    PENDING = "pending"
    COMPLETED = "completed"
    EXPIRED = "expired"
    REJECTED = "rejected"
    MODIFIED = "modified"


@dataclass
class AIProposal:
    """AI-generated decision proposal."""

    intent_type: str
    confidence: float
    entities: list[dict[str, Any]] = field(default_factory=list)
    reasoning: str = ""
    latency_ms: float = 0.0
    model_version: str = ""

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "intent_type": self.intent_type,
            "confidence": self.confidence,
            "entities": self.entities,
            "reasoning": self.reasoning,
            "latency_ms": self.latency_ms,
            "model_version": self.model_version,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AIProposal":
        """Create from dictionary."""
        return cls(
            intent_type=data.get("intent_type", "UNKNOWN"),
            confidence=data.get("confidence", 0.0),
            entities=data.get("entities", []),
            reasoning=data.get("reasoning", ""),
            latency_ms=data.get("latency_ms", 0.0),
            model_version=data.get("model_version", ""),
        )


@dataclass
class HumanAction:
    """Human-executed action."""

    command: str
    parameters: dict[str, Any] = field(default_factory=dict)
    source: str = "manual"
    operator_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "command": self.command,
            "parameters": self.parameters,
            "source": self.source,
            "operator_id": self.operator_id,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "HumanAction":
        """Create from dictionary."""
        return cls(
            command=data.get("command", ""),
            parameters=data.get("parameters", {}),
            source=data.get("source", "manual"),
            operator_id=data.get("operator_id", ""),
        )


@dataclass
class DecisionOutcome:
    """Outcome of a decision execution."""

    success: bool
    execution_time_ms: float = 0.0
    error_message: str = ""
    metrics: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "success": self.success,
            "execution_time_ms": self.execution_time_ms,
            "error_message": self.error_message,
            "metrics": self.metrics,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DecisionOutcome":
        """Create from dictionary."""
        return cls(
            success=data.get("success", False),
            execution_time_ms=data.get("execution_time_ms", 0.0),
            error_message=data.get("error_message", ""),
            metrics=data.get("metrics", {}),
        )


@dataclass
class DecisionContext:
    """Context when decision was made."""

    robot_pose: dict[str, float] = field(default_factory=dict)
    battery_level: float = 0.0
    current_task: str = ""
    environment: dict[str, Any] = field(default_factory=dict)
    timestamp: datetime = field(default_factory=lambda: datetime.now(UTC))

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "robot_pose": self.robot_pose,
            "battery_level": self.battery_level,
            "current_task": self.current_task,
            "environment": self.environment,
            "timestamp": self.timestamp.isoformat(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DecisionContext":
        """Create from dictionary."""
        ts = data.get("timestamp", datetime.now(UTC).isoformat())
        if isinstance(ts, str):
            ts = datetime.fromisoformat(ts)
        return cls(
            robot_pose=data.get("robot_pose", {}),
            battery_level=data.get("battery_level", 0.0),
            current_task=data.get("current_task", ""),
            environment=data.get("environment", {}),
            timestamp=ts,
        )


@dataclass
class DecisionRecord:
    """Complete record of an AI-human decision pair."""

    record_id: str
    robot_id: str
    timestamp: datetime
    context: DecisionContext
    ai_proposal: AIProposal | None = None
    human_action: HumanAction | None = None
    outcome: DecisionOutcome | None = None
    status: str = "pending"
    agreement: bool | None = None
    agreement_score: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary."""
        return {
            "record_id": self.record_id,
            "robot_id": self.robot_id,
            "timestamp": self.timestamp.isoformat(),
            "context": self.context.to_dict() if self.context else {},
            "ai_proposal": self.ai_proposal.to_dict() if self.ai_proposal else None,
            "human_action": self.human_action.to_dict() if self.human_action else None,
            "outcome": self.outcome.to_dict() if self.outcome else None,
            "status": self.status,
            "agreement": self.agreement,
            "agreement_score": self.agreement_score,
        }
