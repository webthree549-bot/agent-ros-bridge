"""Tests for error handling and validation utilities."""

import pytest
import time
from unittest.mock import patch

from agent_ros_bridge.utils.error_handling import (
    ErrorCode,
    AgentError,
    ValidationResult,
    InputValidator,
    CircuitBreaker,
    with_retry,
    with_circuit_breaker,
    handle_error,
    llm_client,
)


class TestErrorCode:
    """Test error code enumeration."""

    def test_error_codes_exist(self):
        """All expected error codes are defined."""
        assert ErrorCode.INTENT_PARSE_FAILED.value == "AI001"
        assert ErrorCode.VALIDATION_FAILED.value == "SAF001"
        assert ErrorCode.PLANNING_FAILED.value == "MP001"
        assert ErrorCode.UNKNOWN.value == "SYS999"


class TestAgentError:
    """Test AgentError exception."""

    def test_error_with_code_and_message(self):
        """Error contains code and message."""
        error = AgentError(code=ErrorCode.INTENT_PARSE_FAILED, message="Failed to parse intent")

        assert error.code == ErrorCode.INTENT_PARSE_FAILED
        assert "Failed to parse intent" in str(error)
        assert "AI001" in str(error)

    def test_error_with_context(self):
        """Error can include context."""
        error = AgentError(
            code=ErrorCode.VALIDATION_FAILED,
            message="Validation failed",
            context={"field": "velocity", "value": 100},
        )

        assert "velocity" in str(error)
        assert "100" in str(error)


class TestInputValidator:
    """Test input validation."""

    def test_validate_empty_utterance_fails(self):
        """Empty utterance is rejected."""
        result = InputValidator.validate_utterance("")
        assert not result.valid
        assert result.error_code == ErrorCode.INTENT_PARSE_FAILED

    def test_validate_whitespace_utterance_fails(self):
        """Whitespace-only utterance is rejected."""
        result = InputValidator.validate_utterance("   ")
        assert not result.valid

    def test_validate_long_utterance_fails(self):
        """Very long utterance is rejected."""
        result = InputValidator.validate_utterance("x" * 1001)
        assert not result.valid

    def test_validate_valid_utterance(self):
        """Valid utterance is accepted and sanitized."""
        result = InputValidator.validate_utterance("  go to kitchen  ")
        assert result.valid
        assert result.sanitized_value == "go to kitchen"

    def test_validate_pose_with_nan_fails(self):
        """NaN in pose is rejected."""
        result = InputValidator.validate_pose(float("nan"), 0, 0)
        assert not result.valid

    def test_validate_pose_with_inf_fails(self):
        """Infinity in pose is rejected."""
        result = InputValidator.validate_pose(0, float("inf"), 0)
        assert not result.valid

    def test_validate_pose_out_of_bounds_fails(self):
        """Pose values out of reasonable bounds are rejected."""
        result = InputValidator.validate_pose(1e7, 0, 0)
        assert not result.valid

    def test_validate_valid_pose(self):
        """Valid pose is accepted."""
        result = InputValidator.validate_pose(1.5, 2.5, 0.5)
        assert result.valid
        assert result.sanitized_value == (1.5, 2.5, 0.5)

    def test_validate_empty_trajectory_fails(self):
        """Empty trajectory is rejected."""
        result = InputValidator.validate_trajectory([])
        assert not result.valid

    def test_validate_valid_trajectory(self):
        """Valid trajectory is accepted."""
        result = InputValidator.validate_trajectory([{"x": 0, "y": 0}, {"x": 1, "y": 1}])
        assert result.valid


class TestCircuitBreaker:
    """Test circuit breaker pattern."""

    def test_circuit_starts_closed(self):
        """Circuit starts in closed state."""
        cb = CircuitBreaker()
        assert cb.state == "closed"
        assert cb.can_execute()

    def test_circuit_opens_after_failures(self):
        """Circuit opens after threshold failures."""
        cb = CircuitBreaker(failure_threshold=3)

        for _ in range(3):
            cb.record_failure()

        assert cb.state == "open"
        assert not cb.can_execute()

    def test_circuit_enters_half_open_after_timeout(self):
        """Circuit enters half-open after recovery timeout."""
        cb = CircuitBreaker(failure_threshold=1, recovery_timeout=0.1)
        cb.record_failure()

        assert cb.state == "open"

        time.sleep(0.15)

        assert cb.can_execute()  # Should be half-open now
        assert cb.state == "half_open"

    def test_circuit_closes_after_successful_recovery(self):
        """Circuit closes after successful calls in half-open."""
        cb = CircuitBreaker(failure_threshold=2, recovery_timeout=0.1, half_open_max_calls=1)
        cb.record_failure()
        assert cb.state == "closed"  # Still closed after 1 failure

        cb.record_failure()  # Now at threshold
        assert cb.state == "open"

        time.sleep(0.15)

        # Must call can_execute to transition to half_open
        assert cb.can_execute()
        assert cb.state == "half_open"

        cb.record_success()

        assert cb.state == "closed"

    def test_circuit_reopens_after_half_open_failure(self):
        """Circuit reopens if half-open calls fail."""
        cb = CircuitBreaker(failure_threshold=1, recovery_timeout=0.1)
        cb.record_failure()
        time.sleep(0.15)

        cb.record_failure()

        assert cb.state == "open"


class TestRetryDecorator:
    """Test retry decorator."""

    def test_retry_succeeds_eventually(self):
        """Function succeeds after retries."""
        call_count = 0

        @with_retry(max_retries=2, delay=0.01)
        def flaky_function():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise ValueError("Temporary failure")
            return "success"

        result = flaky_function()

        assert result == "success"
        assert call_count == 3

    def test_retry_exhausts_and_raises(self):
        """Exception raised after all retries exhausted."""

        @with_retry(max_retries=2, delay=0.01)
        def always_fails():
            raise ValueError("Always fails")

        with pytest.raises(ValueError):
            always_fails()


class TestErrorHandler:
    """Test error handling utility."""

    def test_handle_timeout_error(self):
        """TimeoutError mapped to TIMEOUT code."""
        error = handle_error(TimeoutError("Connection timeout"))

        assert error.code == ErrorCode.TIMEOUT
        assert "Connection timeout" in error.message

    def test_handle_value_error(self):
        """ValueError mapped to INTENT_PARSE_FAILED."""
        error = handle_error(ValueError("Invalid input"))

        assert error.code == ErrorCode.INTENT_PARSE_FAILED

    def test_handle_unknown_error(self):
        """Unknown error types mapped to UNKNOWN."""
        error = handle_error(RuntimeError("Unexpected"))

        assert error.code == ErrorCode.UNKNOWN

    def test_handle_agent_error_passthrough(self):
        """AgentError passed through unchanged."""
        original = AgentError(ErrorCode.PLANNING_FAILED, "Test")
        result = handle_error(original)

        assert result is original


class TestLLMClient:
    """Test LLM client with circuit breaker."""

    def test_llm_client_has_circuit_breaker(self):
        """LLM client has circuit breaker."""
        assert llm_client.circuit_breaker is not None
        assert llm_client.timeout == 5.0

    @patch("random.random")
    def test_llm_call_succeeds(self, mock_random):
        """LLM call succeeds when no failure."""
        mock_random.return_value = 0.5  # No failure

        result = llm_client.call("Test prompt")

        assert "Processed" in result

    @patch("random.random")
    def test_llm_call_retries_on_timeout(self, mock_random):
        """LLM call retries on timeout."""
        mock_random.side_effect = [0.05, 0.05, 0.5]  # Fail twice, then succeed

        result = llm_client.call("Test prompt")

        assert "Processed" in result
