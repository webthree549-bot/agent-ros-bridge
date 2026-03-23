"""Integration layer for shadow mode.

Connects shadow mode logging to existing system components.
"""

from typing import Any

from .decision_logger import DecisionLogger
from .models import AIProposal, HumanAction, DecisionOutcome


class ShadowModeIntegration:
    """Integration layer for shadow mode logging.

    Usage:
        integration = ShadowModeIntegration()

        # Log AI decision
        integration.log_ai_decision(
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )

        # Log human decision
        integration.log_human_decision(
            robot_id="bot1",
            command="navigate_to",
            parameters={"location": "kitchen"},
        )
    """

    def __init__(
        self,
        logger: DecisionLogger | None = None,
        enabled: bool = True,
    ):
        """Initialize integration.

        Args:
            logger: DecisionLogger instance (creates default if None)
            enabled: Whether shadow mode is enabled
        """
        self._logger = logger or DecisionLogger()
        self._enabled = enabled

    @property
    def logger(self) -> DecisionLogger:
        """Get the decision logger."""
        return self._logger

    def is_enabled(self) -> bool:
        """Check if shadow mode is enabled."""
        return self._enabled

    def log_ai_decision(
        self,
        robot_id: str,
        intent_type: str,
        confidence: float,
        entities: list[dict[str, Any]] | None = None,
        reasoning: str = "",
        latency_ms: float = 0.0,
    ) -> str | None:
        """Log an AI decision.

        Args:
            robot_id: Robot identifier
            intent_type: Type of intent (NAVIGATE, MANIPULATE, etc.)
            confidence: AI confidence score (0.0-1.0)
            entities: Extracted entities
            reasoning: AI reasoning
            latency_ms: Decision latency in milliseconds

        Returns:
            Record ID or None if disabled
        """
        if not self._enabled:
            return None

        proposal = AIProposal(
            intent_type=intent_type,
            confidence=confidence,
            entities=entities or [],
            reasoning=reasoning,
            latency_ms=latency_ms,
        )

        return self._logger.log_ai_proposal(robot_id, proposal)

    def log_human_decision(
        self,
        robot_id: str,
        command: str,
        parameters: dict[str, Any] | None = None,
        source: str = "manual",
        operator_id: str = "",
    ) -> str | None:
        """Log a human decision.

        Args:
            robot_id: Robot identifier
            command: Command executed
            parameters: Command parameters
            source: Source of command (manual, script, etc.)
            operator_id: Operator identifier

        Returns:
            Record ID or None if disabled
        """
        if not self._enabled:
            return None

        action = HumanAction(
            command=command,
            parameters=parameters or {},
            source=source,
            operator_id=operator_id,
        )

        return self._logger.log_human_action(robot_id, action)

    def log_outcome(
        self,
        record_id: str,
        success: bool,
        execution_time_ms: float = 0.0,
        error_message: str = "",
        metrics: dict[str, Any] | None = None,
    ) -> bool:
        """Log execution outcome.

        Args:
            record_id: Decision record ID
            success: Whether execution succeeded
            execution_time_ms: Execution time in milliseconds
            error_message: Error message if failed
            metrics: Additional metrics

        Returns:
            True if logged successfully
        """
        if not self._enabled:
            return False

        outcome = DecisionOutcome(
            success=success,
            execution_time_ms=execution_time_ms,
            error_message=error_message,
            metrics=metrics or {},
        )

        return self._logger.log_outcome(record_id, outcome)

    def get_metrics(self) -> dict[str, Any]:
        """Get shadow mode metrics.

        Returns:
            Metrics dictionary
        """
        stats = self._logger.get_stats()

        return {
            "total_decisions": stats.get("total_logged", 0),
            "completed_decisions": stats.get("total_completed", 0),
            "pending_decisions": stats.get("pending_count", 0),
            "enabled": self._enabled,
        }
