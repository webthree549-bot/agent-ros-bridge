"""
Agent ROS Bridge - Error Handling and Validation Utilities

Provides unified error handling, input validation, and circuit breaker patterns
for the AI layer and safety modules.
"""

import functools
import logging
import time
from typing import Any, Callable, Dict, Optional, TypeVar, Generic
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ErrorCode(Enum):
    """Standardized error codes for the system."""

    # AI Layer Errors
    INTENT_PARSE_FAILED = "AI001"
    CONTEXT_RESOLUTION_FAILED = "AI002"
    LLM_UNAVAILABLE = "AI003"
    LLM_TIMEOUT = "AI004"

    # Safety Errors
    VALIDATION_FAILED = "SAF001"
    WORKSPACE_VIOLATION = "SAF002"
    VELOCITY_LIMIT_EXCEEDED = "SAF003"
    EMERGENCY_STOP_TRIGGERED = "SAF004"

    # Motion Planning Errors
    PLANNING_FAILED = "MP001"
    EXECUTION_FAILED = "MP002"
    TRAJECTORY_INVALID = "MP003"

    # System Errors
    ROS2_UNAVAILABLE = "SYS001"
    DOCKER_ERROR = "SYS002"
    NETWORK_ERROR = "SYS003"
    TIMEOUT = "SYS004"
    UNKNOWN = "SYS999"


@dataclass
class AgentError(Exception):
    """Standardized error with code and context."""

    code: ErrorCode
    message: str
    context: Optional[Dict[str, Any]] = None

    def __str__(self):
        ctx = f" | Context: {self.context}" if self.context else ""
        return f"[{self.code.value}] {self.message}{ctx}"


@dataclass
class ValidationResult:
    """Result of input validation."""

    valid: bool
    error_code: Optional[ErrorCode] = None
    error_message: Optional[str] = None
    sanitized_value: Any = None


class InputValidator:
    """Unified input validation for all AI services."""

    @staticmethod
    def validate_utterance(utterance: str) -> ValidationResult:
        """Validate natural language utterance."""
        if not utterance:
            return ValidationResult(
                valid=False,
                error_code=ErrorCode.INTENT_PARSE_FAILED,
                error_message="Utterance cannot be empty",
            )

        # Sanitize first
        sanitized = utterance.strip()

        # Check if empty after sanitization
        if not sanitized:
            return ValidationResult(
                valid=False,
                error_code=ErrorCode.INTENT_PARSE_FAILED,
                error_message="Utterance cannot be empty",
            )

        if len(sanitized) > 1000:
            return ValidationResult(
                valid=False,
                error_code=ErrorCode.INTENT_PARSE_FAILED,
                error_message="Utterance too long (max 1000 chars)",
            )

        return ValidationResult(valid=True, sanitized_value=sanitized)

    @staticmethod
    def validate_pose(x: float, y: float, theta: float) -> ValidationResult:
        """Validate robot pose coordinates."""
        # Check for NaN/Inf
        for name, value in [("x", x), ("y", y), ("theta", theta)]:
            if not isinstance(value, (int, float)):
                return ValidationResult(
                    valid=False,
                    error_code=ErrorCode.TRAJECTORY_INVALID,
                    error_message=f"{name} must be a number",
                )
            if not (-1e6 <= value <= 1e6):  # Reasonable bounds
                return ValidationResult(
                    valid=False,
                    error_code=ErrorCode.TRAJECTORY_INVALID,
                    error_message=f"{name} out of reasonable bounds",
                )

        return ValidationResult(valid=True, sanitized_value=(x, y, theta))

    @staticmethod
    def validate_trajectory(waypoints: list) -> ValidationResult:
        """Validate trajectory waypoints."""
        if not waypoints:
            return ValidationResult(
                valid=False,
                error_code=ErrorCode.TRAJECTORY_INVALID,
                error_message="Trajectory must have at least one waypoint",
            )

        if len(waypoints) > 10000:
            return ValidationResult(
                valid=False,
                error_code=ErrorCode.TRAJECTORY_INVALID,
                error_message="Trajectory too long (max 10000 waypoints)",
            )

        return ValidationResult(valid=True)


class CircuitBreaker:
    """Circuit breaker pattern for LLM and external service calls."""

    def __init__(
        self,
        failure_threshold: int = 5,
        recovery_timeout: float = 30.0,
        half_open_max_calls: int = 3,
    ):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.half_open_max_calls = half_open_max_calls

        self.failures = 0
        self.last_failure_time: Optional[float] = None
        self.state = "closed"  # closed, open, half_open
        self.half_open_calls = 0

    def can_execute(self) -> bool:
        """Check if execution is allowed."""
        if self.state == "closed":
            return True

        if self.state == "open":
            if time.time() - (self.last_failure_time or 0) > self.recovery_timeout:
                self.state = "half_open"
                self.half_open_calls = 0
                logger.info("Circuit breaker entering half-open state")
                return True
            return False

        if self.state == "half_open":
            if self.half_open_calls < self.half_open_max_calls:
                self.half_open_calls += 1
                return True
            return False

        return True

    def record_success(self):
        """Record successful execution."""
        self.failures = 0
        if self.state == "half_open":
            self.state = "closed"
            self.half_open_calls = 0
            logger.info("Circuit breaker closed after successful recovery")

    def record_failure(self):
        """Record failed execution."""
        self.failures += 1
        self.last_failure_time = time.time()

        if self.failures >= self.failure_threshold:
            self.state = "open"
            logger.warning(f"Circuit breaker opened after {self.failures} failures")


def with_retry(
    max_retries: int = 3, delay: float = 1.0, backoff: float = 2.0, exceptions: tuple = (Exception,)
):
    """Decorator for retry logic."""

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            current_delay = delay
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt < max_retries:
                        logger.warning(
                            f"Attempt {attempt + 1}/{max_retries + 1} failed: {e}. "
                            f"Retrying in {current_delay}s..."
                        )
                        time.sleep(current_delay)
                        current_delay *= backoff
                    else:
                        logger.error(f"All {max_retries + 1} attempts failed")

            raise last_exception

        return wrapper

    return decorator


def with_circuit_breaker(circuit_breaker: CircuitBreaker):
    """Decorator for circuit breaker pattern."""

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if not circuit_breaker.can_execute():
                raise AgentError(
                    code=ErrorCode.LLM_UNAVAILABLE,
                    message="Circuit breaker is open - service temporarily unavailable",
                )

            try:
                result = func(*args, **kwargs)
                circuit_breaker.record_success()
                return result
            except Exception:
                circuit_breaker.record_failure()
                raise

        return wrapper

    return decorator


class LLMClient:
    """LLM client with circuit breaker and retry logic."""

    def __init__(self):
        self.circuit_breaker = CircuitBreaker(failure_threshold=3, recovery_timeout=60.0)
        self.timeout = 5.0  # 5 second timeout for LLM calls

    @with_retry(max_retries=2, delay=0.5, exceptions=(TimeoutError, ConnectionError))
    @with_circuit_breaker(CircuitBreaker(failure_threshold=3))
    def call(self, prompt: str) -> str:
        """Call LLM with timeout, retry, and circuit breaker."""
        # This is a placeholder - actual implementation would call OpenAI/Anthropic
        # For now, just simulate the pattern

        import random

        if random.random() < 0.1:  # 10% failure rate for testing
            raise TimeoutError("LLM call timed out")

        return f"Processed: {prompt[:50]}..."


# Global instances
input_validator = InputValidator()
llm_client = LLMClient()


def handle_error(error: Exception, context: Optional[Dict] = None) -> AgentError:
    """Convert any exception to standardized AgentError."""
    if isinstance(error, AgentError):
        return error

    # Map common exceptions to error codes
    error_mapping = {
        TimeoutError: ErrorCode.TIMEOUT,
        ConnectionError: ErrorCode.NETWORK_ERROR,
        ValueError: ErrorCode.INTENT_PARSE_FAILED,
        KeyError: ErrorCode.CONTEXT_RESOLUTION_FAILED,
    }

    error_code = error_mapping.get(type(error), ErrorCode.UNKNOWN)

    return AgentError(code=error_code, message=str(error), context=context)
