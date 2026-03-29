"""
Shadow Mode Hooks - Integration with Existing Systems

Auto-logs AI proposals and human commands for shadow mode analysis.
Integrates with:
- Intent parser (AI proposals)
- Gateway (human commands)
- Decision logger (storage)
- Dashboard (real-time metrics)
"""

import time
from dataclasses import dataclass
from typing import Any


@dataclass
class ShadowHooksConfig:
    """Configuration for shadow mode hooks"""

    enabled: bool = True
    confidence_threshold: float = 0.0  # Log all by default
    robot_ids: list[str] | None = None  # None = all robots
    log_rejections: bool = True
    log_modifications: bool = True
    real_time_dashboard: bool = True


class ShadowModeHooks:
    """
    Hooks into existing systems to auto-log shadow mode data.

    Features:
    - Auto-log AI proposals from intent parser
    - Auto-log human commands from gateway
    - Real-time comparison and agreement calculation
    - Dashboard updates
    - Configurable filtering
    """

    def __init__(
        self,
        enabled: bool = True,
        confidence_threshold: float = 0.0,
        robot_ids: list[str] | None = None,
        decision_logger=None,
        comparator=None,
        dashboard=None,
    ):
        """
        Initialize shadow mode hooks.

        Args:
            enabled: Whether shadow mode is active
            confidence_threshold: Minimum confidence to log (0 = all)
            robot_ids: List of robot IDs to monitor (None = all)
            decision_logger: DecisionLogger instance
            comparator: DecisionComparator instance
            dashboard: DashboardAPI instance
        """
        self.config = ShadowHooksConfig(
            enabled=enabled,
            confidence_threshold=confidence_threshold,
            robot_ids=robot_ids,
        )

        # Components (lazy init if not provided)
        self._decision_logger = decision_logger
        self._comparator = comparator
        self._dashboard = dashboard

        # State tracking
        self._pending_proposals: dict[str, dict] = {}  # robot_id -> proposal
        self._total_decisions = 0
        self._agreements = 0
        self._confidences: list[float] = []

    def _get_decision_logger(self):
        """Lazy init decision logger"""
        if self._decision_logger is None:
            from .decision_logger import DecisionLogger

            self._decision_logger = DecisionLogger()
        return self._decision_logger

    def _get_comparator(self):
        """Lazy init comparator"""
        if self._comparator is None:
            from .comparator import DecisionComparator

            self._comparator = DecisionComparator()
        return self._comparator

    def _get_dashboard(self):
        """Lazy init dashboard"""
        if self._dashboard is None:
            from .dashboard import DashboardAPI

            self._dashboard = DashboardAPI()
        return self._dashboard

    def _should_log(self, robot_id: str, confidence: float = 1.0) -> bool:
        """Check if this robot/confidence should be logged"""
        if not self.config.enabled:
            return False

        # Check robot filter
        if self.config.robot_ids is not None:
            if robot_id not in self.config.robot_ids:
                return False

        # Check confidence threshold
        return confidence >= self.config.confidence_threshold

    def on_intent_parsed(
        self,
        robot_id: str,
        intent_result: dict[str, Any],
    ) -> str | None:
        """
        Hook called when intent parser produces a result.

        Args:
            robot_id: Robot identifier
            intent_result: Intent parser output

        Returns:
            Decision ID if logged, None otherwise
        """
        confidence = intent_result.get("confidence", 1.0)

        if not self._should_log(robot_id, confidence):
            return None

        # Extract entities
        entities = intent_result.get("entities", [])

        # Create AI proposal model
        from .models import AIProposal

        proposal = AIProposal(
            intent_type=intent_result.get("intent_type", "UNKNOWN"),
            confidence=confidence,
            entities=entities,
            reasoning=f"Parsed intent: {intent_result.get('intent_type')}",
        )

        # Log AI decision
        decision_id = self._get_decision_logger().log_ai_proposal(
            robot_id=robot_id,
            proposal=proposal,
        )

        # Store pending proposal ID for comparison
        self._pending_proposals[robot_id] = decision_id

        # Update metrics
        self._total_decisions += 1
        self._confidences.append(confidence)

        return decision_id

    def on_human_command(
        self,
        command: dict[str, Any],
    ) -> str | None:
        """
        Hook called when human executes a command.

        Args:
            command: Command dict with robot_id, command, parameters

        Returns:
            Decision ID if logged, None otherwise
        """
        robot_id = command.get("robot_id", "unknown")

        if not self._should_log(robot_id):
            return None

        # Create human action model
        from .models import HumanAction

        action = HumanAction(
            command=command.get("command", "unknown"),
            parameters=command.get("parameters", {}),
            operator_id="human",
        )

        # Log human decision
        decision_id = self._get_decision_logger().log_human_action(
            robot_id=robot_id,
            action=action,
        )

        # Compare with pending AI proposal
        if robot_id in self._pending_proposals:
            # Get the AI proposal from decision logger
            pending_records = self._get_decision_logger().get_pending(robot_id)
            if pending_records:
                ai_proposal = pending_records[0].ai_proposal

                # Build a temporary DecisionRecord for comparison
                from .models import DecisionContext, DecisionRecord

                temp_record = DecisionRecord(
                    record_id="temp",
                    robot_id=robot_id,
                    timestamp=pending_records[0].timestamp,
                    context=DecisionContext(),
                    ai_proposal=ai_proposal,
                    human_action=action,
                    status="completed",
                )

                # Run comparison
                agrees, similarity = self._get_comparator().compare(temp_record)

                # Update agreement tracking
                if agrees:
                    self._agreements += 1

                # Update dashboard
                if self.config.real_time_dashboard:
                    self._update_dashboard_metrics(
                        robot_id=robot_id,
                        agreement=agrees,
                        similarity=similarity,
                    )

            # Clear pending
            del self._pending_proposals[robot_id]

        return decision_id

    def on_human_rejected(
        self,
        robot_id: str,
        ai_proposal_id: str,
        rejection_reason: str = "unknown",
    ) -> None:
        """
        Hook called when human rejects AI proposal.

        Args:
            robot_id: Robot identifier
            ai_proposal_id: ID of rejected proposal
            rejection_reason: Why it was rejected
        """
        if not self.config.enabled:
            return

        if not self.config.log_rejections:
            return

        self._get_decision_logger().log_rejection(
            robot_id=robot_id,
            ai_proposal_id=ai_proposal_id,
            rejection_reason=rejection_reason,
            timestamp=time.time(),
        )

        # Clear pending
        if robot_id in self._pending_proposals:
            del self._pending_proposals[robot_id]

    def on_human_modified(
        self,
        robot_id: str,
        ai_proposal_id: str,
        original: dict[str, Any],
        modified: dict[str, Any],
    ) -> None:
        """
        Hook called when human modifies AI proposal.

        Args:
            robot_id: Robot identifier
            ai_proposal_id: ID of original proposal
            original: Original AI proposal
            modified: Human-modified version
        """
        if not self.config.enabled:
            return

        if not self.config.log_modifications:
            return

        self._get_decision_logger().log_modification(
            robot_id=robot_id,
            ai_proposal_id=ai_proposal_id,
            original=original,
            modified=modified,
            timestamp=time.time(),
        )

    def _update_dashboard_metrics(
        self,
        robot_id: str,
        agreement: bool,
        similarity: float,
    ) -> None:
        """Update dashboard with new metrics"""
        try:
            self._get_dashboard().update_metrics(
                {
                    "total_decisions": self._total_decisions,
                    "agreement_rate": self.agreement_rate,
                    "recent_agreement": agreement,
                    "recent_similarity": similarity,
                    "robot_id": robot_id,
                }
            )
        except Exception:
            # Don't let dashboard errors break logging
            pass

    @property
    def total_decisions(self) -> int:
        """Total number of decisions logged"""
        return self._total_decisions

    @property
    def agreement_rate(self) -> float:
        """Current agreement rate between AI and human"""
        if self._total_decisions == 0:
            return 0.0
        return self._agreements / self._total_decisions

    @property
    def avg_confidence(self) -> float:
        """Average AI confidence"""
        if not self._confidences:
            return 0.0
        return sum(self._confidences) / len(self._confidences)

    def get_stats(self) -> dict[str, Any]:
        """Get summary statistics"""
        return {
            "total_decisions": self.total_decisions,
            "agreement_rate": self.agreement_rate,
            "avg_confidence": self.avg_confidence,
            "pending_proposals": len(self._pending_proposals),
            "enabled": self.config.enabled,
        }

    def wrap_intent_parser(self, parser):
        """
        Wrap an existing intent parser to auto-log.

        Args:
            parser: Intent parser instance

        Returns:
            Wrapped parser
        """
        original_parse = parser.parse

        def wrapped_parse(text: str, robot_id: str = "unknown", **kwargs):
            # Call original
            result = original_parse(text, robot_id=robot_id, **kwargs)

            # Log the AI proposal
            if isinstance(result, dict):
                self.on_intent_parsed(robot_id, result)

            return result

        parser.parse = wrapped_parse
        return parser

    def wrap_gateway(self, gateway):
        """
        Wrap an existing gateway to auto-log human commands.

        Args:
            gateway: Gateway instance

        Returns:
            Wrapped gateway
        """
        original_send = gateway.send_command

        def wrapped_send(command: dict[str, Any], **kwargs):
            # Log human command
            self.on_human_command(command)

            # Call original
            return original_send(command, **kwargs)

        gateway.send_command = wrapped_send
        return gateway

    def hook_intent_parser(self, parser_class):
        """
        Hook into intent parser class (monkey patch).

        Args:
            parser_class: Intent parser class to hook
        """
        original_init = parser_class.__init__
        hooks = self

        def hooked_init(self, *args, **kwargs):
            original_init(self, *args, **kwargs)
            # Store hooks reference
            self._shadow_hooks = hooks

        parser_class.__init__ = hooked_init

        # Hook parse method
        if hasattr(parser_class, "parse"):
            original_parse = parser_class.parse

            def hooked_parse(self, text: str, robot_id: str = "unknown", **kwargs):
                result = original_parse(self, text, robot_id=robot_id, **kwargs)

                # Log via hooks
                if hasattr(self, "_shadow_hooks"):
                    self._shadow_hooks.on_intent_parsed(robot_id, result)

                return result

            parser_class.parse = hooked_parse

    def hook_gateway(self, gateway_class):
        """
        Hook into gateway class (monkey patch).

        Args:
            gateway_class: Gateway class to hook
        """
        original_send = gateway_class.send_command

        def hooked_send(self, command: dict[str, Any], **kwargs):
            # Log via hooks
            if hasattr(self, "_shadow_hooks"):
                self._shadow_hooks.on_human_command(command)

            return original_send(self, command, **kwargs)

        gateway_class.send_command = hooked_send


# Convenience functions


def enable_shadow_mode(
    intent_parser=None,
    gateway=None,
    robot_ids: list[str] | None = None,
) -> ShadowModeHooks:
    """
    Enable shadow mode on existing components.

    Args:
        intent_parser: Intent parser instance or class
        gateway: Gateway instance or class
        robot_ids: Optional list of robot IDs to monitor

    Returns:
        ShadowModeHooks instance
    """
    hooks = ShadowModeHooks(robot_ids=robot_ids)

    if intent_parser is not None:
        if isinstance(intent_parser, type):
            hooks.hook_intent_parser(intent_parser)
        else:
            hooks.wrap_intent_parser(intent_parser)

    if gateway is not None:
        if isinstance(gateway, type):
            hooks.hook_gateway(gateway)
        else:
            hooks.wrap_gateway(gateway)

    return hooks
