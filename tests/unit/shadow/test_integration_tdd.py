"""TDD tests for shadow mode integration - RED PHASE.

These tests define how shadow mode integrates with existing systems.
Run these first - they should all FAIL.
"""

from datetime import datetime, timezone
from unittest.mock import Mock, patch

import pytest

from agent_ros_bridge.shadow import DecisionLogger

# These imports will fail until we implement the modules
from agent_ros_bridge.shadow.integration import ShadowModeIntegration


class TestShadowModeIntegrationTDD:
    """TDD tests for ShadowModeIntegration."""

    @pytest.fixture
    def integration(self):
        """Create integration instance."""
        return ShadowModeIntegration()

    @pytest.fixture
    def mock_logger(self):
        """Create mock decision logger."""
        return Mock(spec=DecisionLogger)

    def test_integration_initializes(self):
        """RED: ShadowModeIntegration should initialize."""
        integration = ShadowModeIntegration()
        assert integration is not None
        assert integration.is_enabled() is True

    def test_integration_can_be_disabled(self):
        """RED: ShadowModeIntegration should support disable."""
        integration = ShadowModeIntegration(enabled=False)
        assert integration.is_enabled() is False

    def test_log_ai_decision_calls_logger(self, integration, mock_logger):
        """RED: log_ai_decision() should call logger.log_ai_proposal()."""
        integration._logger = mock_logger

        integration.log_ai_decision(
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )

        mock_logger.log_ai_proposal.assert_called_once()
        call_args = mock_logger.log_ai_proposal.call_args
        # call_args[0] is positional args, call_args[1] is kwargs
        # robot_id is first positional arg
        assert call_args[0][0] == "bot1"

    def test_log_human_decision_calls_logger(self, integration, mock_logger):
        """RED: log_human_decision() should call logger.log_human_action()."""
        integration._logger = mock_logger

        integration.log_human_decision(
            robot_id="bot1",
            command="navigate_to",
            parameters={"location": "kitchen"},
        )

        mock_logger.log_human_action.assert_called_once()
        call_args = mock_logger.log_human_action.call_args
        # robot_id is first positional arg
        assert call_args[0][0] == "bot1"

    def test_log_outcome_calls_logger(self, integration, mock_logger):
        """RED: log_outcome() should call logger.log_outcome()."""
        integration._logger = mock_logger

        integration.log_outcome(
            record_id="test-record-id",
            success=True,
            execution_time_ms=5000.0,
        )

        mock_logger.log_outcome.assert_called_once()

    def test_integration_does_nothing_when_disabled(self, integration, mock_logger):
        """RED: Integration should do nothing when disabled."""
        integration._logger = mock_logger
        integration._enabled = False

        integration.log_ai_decision(
            robot_id="bot1",
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[],
        )

        mock_logger.log_ai_proposal.assert_not_called()

    def test_get_metrics_delegates_to_logger(self, integration, mock_logger):
        """RED: get_metrics() should delegate to logger.get_stats()."""
        integration._logger = mock_logger
        mock_logger.get_stats.return_value = {
            "total_logged": 10,
            "total_completed": 8,
            "pending_count": 2,
        }

        metrics = integration.get_metrics()

        mock_logger.get_stats.assert_called_once()
        assert metrics["total_decisions"] == 10

    def test_integration_has_logger_property(self, integration):
        """RED: Integration should expose logger property."""
        assert hasattr(integration, 'logger')
        assert integration.logger is not None
