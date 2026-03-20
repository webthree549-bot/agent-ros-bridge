"""Tests for input validation module."""

import pytest

from agent_ros_bridge.integrations.input_validation import (
    InputValidator,
    SafetyValidator,
    ValidationResult,
    is_command_safe,
    validate_input_safe,
)


class TestValidationResult:
    """Test ValidationResult dataclass."""

    def test_validation_result_creation(self):
        """Test creating a validation result."""
        result = ValidationResult(
            is_valid=True,
            sanitized_input="hello",
            errors=[],
            warnings=[],
        )
        assert result.is_valid is True
        assert result.sanitized_input == "hello"


class TestInputValidatorInitialization:
    """Test InputValidator initialization."""

    def test_init_creates_validator(self):
        """Test that initialization creates validator."""
        validator = InputValidator()
        assert validator.MAX_LENGTH == 500
        assert validator.dangerous_regex is not None


class TestInputValidatorValidate:
    """Test input validation."""

    def test_validate_empty_input(self):
        """Test validation of empty input."""
        validator = InputValidator()
        result = validator.validate("")
        assert result.is_valid is False
        assert "empty" in result.errors[0].lower()

    def test_validate_whitespace_only(self):
        """Test validation of whitespace-only input."""
        validator = InputValidator()
        result = validator.validate("   ")
        assert result.is_valid is False

    def test_validate_valid_input(self):
        """Test validation of valid input."""
        validator = InputValidator()
        result = validator.validate("go to the kitchen")
        assert result.is_valid is True
        assert result.sanitized_input == "go to the kitchen"

    def test_validate_too_long(self):
        """Test validation of too-long input."""
        validator = InputValidator()
        long_input = "a" * 600
        result = validator.validate(long_input)
        assert result.is_valid is False
        assert "too long" in result.errors[0].lower()

    def test_validate_dangerous_characters(self):
        """Test validation blocks dangerous characters."""
        validator = InputValidator()
        result = validator.validate("go; rm -rf /")
        assert result.is_valid is False
        assert "dangerous" in result.errors[0].lower()

    def test_validate_dangerous_keywords(self):
        """Test validation blocks dangerous keywords."""
        validator = InputValidator()
        result = validator.validate("execute this command")
        assert result.is_valid is False
        assert any("exec" in err.lower() for err in result.errors)

    def test_validate_sanitization(self):
        """Test input sanitization."""
        validator = InputValidator()
        result = validator.validate("  hello  world  ")
        assert result.sanitized_input == "hello world"

    def test_validate_null_bytes_removed(self):
        """Test null bytes are removed."""
        validator = InputValidator()
        result = validator.validate("hello\x00world")
        assert "\x00" not in result.sanitized_input

    def test_validate_control_chars_removed(self):
        """Test control characters are removed."""
        validator = InputValidator()
        result = validator.validate("hello\x01\x02world")
        assert "\x01" not in result.sanitized_input
        assert "\x02" not in result.sanitized_input


class TestInputValidatorContextValidation:
    """Test context-specific validation."""

    def test_validate_with_error_state(self):
        """Test validation with robot in error state."""
        validator = InputValidator()
        context = {"robot_status": "error"}
        result = validator.validate("go forward", context)
        assert any("error state" in w.lower() for w in result.warnings)

    def test_validate_with_low_battery(self):
        """Test validation with low battery."""
        validator = InputValidator()
        context = {"battery_percent": 15}
        result = validator.validate("go forward", context)
        assert any("low battery" in w.lower() for w in result.warnings)

    def test_validate_with_emergency_stop(self):
        """Test validation with emergency stop active."""
        validator = InputValidator()
        context = {"emergency_stop": True}
        result = validator.validate("go forward", context)
        assert result.is_valid is False
        assert any("emergency stop" in e.lower() for e in result.errors)


class TestInputValidatorRobotCommand:
    """Test robot command validation."""

    def test_validate_navigate_command(self):
        """Test validation of navigate command."""
        validator = InputValidator()
        caps = {"can_navigate": True}
        result = validator.validate_robot_command("go to kitchen", caps)
        assert result.is_valid is True

    def test_validate_navigate_not_supported(self):
        """Test validation fails if navigation not supported."""
        validator = InputValidator()
        caps = {"can_navigate": False}
        result = validator.validate_robot_command("go to kitchen", caps)
        assert result.is_valid is False
        assert "navigation" in result.errors[0].lower()

    def test_validate_manipulate_command(self):
        """Test validation of manipulate command."""
        validator = InputValidator()
        caps = {"can_manipulate": True}
        result = validator.validate_robot_command("pick up the object", caps)
        assert result.is_valid is True

    def test_validate_manipulate_not_supported(self):
        """Test validation fails if manipulation not supported."""
        validator = InputValidator()
        caps = {"can_manipulate": False}
        result = validator.validate_robot_command("pick up the object", caps)
        assert result.is_valid is False
        assert "manipulation" in result.errors[0].lower()


class TestSafetyValidator:
    """Test SafetyValidator."""

    def test_safety_validator_init(self):
        """Test initialization."""
        validator = SafetyValidator()
        assert validator.DANGEROUS_COMMANDS is not None

    def test_check_safety_safe_command(self):
        """Test safety check for safe command."""
        validator = SafetyValidator()
        result = validator.check_safety("go to kitchen")
        assert result["is_safe"] is True
        assert result["requires_confirmation"] is False

    def test_check_safety_dangerous_command(self):
        """Test safety check for dangerous command."""
        validator = SafetyValidator()
        result = validator.check_safety("move fast")
        assert result["requires_confirmation"] is True
        assert "fast" in result["confirmation_prompt"].lower()

    def test_check_safety_safety_critical(self):
        """Test safety check for safety-critical command."""
        validator = SafetyValidator()
        result = validator.check_safety("emergency stop")
        assert "emergency stop" in result["safety_checks"][0].lower()

    def test_check_safety_with_humans(self):
        """Test safety check with humans nearby."""
        validator = SafetyValidator()
        env = {"humans_nearby": True}
        result = validator.check_safety("move forward", env)
        assert result["requires_confirmation"] is True
        assert any("humans" in w.lower() for w in result["warnings"])

    def test_check_safety_with_obstacles(self):
        """Test safety check with obstacles nearby."""
        validator = SafetyValidator()
        env = {"obstacles_nearby": True}
        result = validator.check_safety("move forward", env)
        assert any("obstacles" in w.lower() for w in result["warnings"])

    def test_check_safety_tight_space(self):
        """Test safety check in tight space."""
        validator = SafetyValidator()
        env = {"tight_space": True}
        result = validator.check_safety("move forward", env)
        assert result["requires_confirmation"] is True
        assert any("tight space" in w.lower() for w in result["warnings"])

    def test_get_safety_confirmation_dangerous(self):
        """Test getting confirmation for dangerous command."""
        validator = SafetyValidator()
        prompt = validator.get_safety_confirmation("move fast")
        assert prompt is not None
        assert "confirm" in prompt.lower()

    def test_get_safety_confirmation_safe(self):
        """Test no confirmation for safe command."""
        validator = SafetyValidator()
        prompt = validator.get_safety_confirmation("go to kitchen")
        assert prompt is None


class TestConvenienceFunctions:
    """Test convenience functions."""

    def test_validate_input_safe_valid(self):
        """Test convenience function with valid input."""
        result = validate_input_safe("go to kitchen")
        assert result.is_valid is True

    def test_validate_input_safe_invalid(self):
        """Test convenience function with invalid input."""
        result = validate_input_safe("")
        assert result.is_valid is False

    def test_is_command_safe_true(self):
        """Test safety check returns True for safe command."""
        result = is_command_safe("go to kitchen")
        assert result is True

    def test_is_command_safe_false(self):
        """Test safety check returns False for dangerous command."""
        result = is_command_safe("move fast")
        assert result is False

    def test_is_command_safe_with_environment(self):
        """Test safety check with environment."""
        env = {"humans_nearby": True}
        result = is_command_safe("go to kitchen", env)
        assert result is False
