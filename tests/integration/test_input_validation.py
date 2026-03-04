"""Tests for input validation.

Verifies security and safety validation.
"""

import pytest
from agent_ros_bridge.integrations.input_validation import (
    InputValidator,
    SafetyValidator,
    validate_input_safe,
    is_command_safe
)


class TestInputValidator:
    """Test input validation."""
    
    @pytest.fixture
    def validator(self):
        return InputValidator()
    
    def test_valid_input(self, validator):
        """Test that valid input passes."""
        result = validator.validate("Move forward 2 meters")
        
        assert result.is_valid is True
        assert len(result.errors) == 0
        assert result.sanitized_input == "Move forward 2 meters"
    
    def test_empty_input(self, validator):
        """Test that empty input fails."""
        result = validator.validate("")
        
        assert result.is_valid is False
        assert "empty" in result.errors[0].lower()
    
    def test_dangerous_characters_blocked(self, validator):
        """Test that dangerous characters are blocked."""
        result = validator.validate("Move forward; rm -rf /")
        
        assert result.is_valid is False
        assert any("dangerous" in e.lower() for e in result.errors)
    
    def test_command_substitution_blocked(self, validator):
        """Test that command substitution is blocked."""
        result = validator.validate("Move forward $(whoami)")
        
        assert result.is_valid is False
    
    def test_backtick_blocked(self, validator):
        """Test that backticks are blocked."""
        result = validator.validate("Move forward `ls`")
        
        assert result.is_valid is False
    
    def test_dangerous_keywords_blocked(self, validator):
        """Test that dangerous keywords are blocked."""
        result = validator.validate("Execute rm -rf")
        
        assert result.is_valid is False
        assert any("rm" in e.lower() for e in result.errors)
    
    def test_path_traversal_blocked(self, validator):
        """Test that path traversal is blocked."""
        result = validator.validate("Go to ../../etc/passwd")
        
        assert result.is_valid is False
    
    def test_length_limit(self, validator):
        """Test that long input is truncated."""
        long_input = "Move forward " * 100
        result = validator.validate(long_input)
        
        assert len(result.sanitized_input) <= InputValidator.MAX_LENGTH
    
    def test_sanitization(self, validator):
        """Test that input is sanitized."""
        result = validator.validate("Move\x00forward")
        
        assert "\x00" not in result.sanitized_input
    
    def test_whitespace_normalization(self, validator):
        """Test that whitespace is normalized."""
        result = validator.validate("Move   forward   2   meters")
        
        assert "  " not in result.sanitized_input
    
    def test_context_validation(self, validator):
        """Test context-aware validation."""
        context = {
            'emergency_stop': True,
            'battery_percent': 15
        }
        
        result = validator.validate("Move forward", context)
        
        assert result.is_valid is False  # Emergency stop active
        assert any("emergency" in e.lower() for e in result.errors)
        assert any("battery" in w.lower() for w in result.warnings)


class TestSafetyValidator:
    """Test safety validation."""
    
    @pytest.fixture
    def validator(self):
        return SafetyValidator()
    
    def test_safe_command(self, validator):
        """Test that safe command passes."""
        result = validator.check_safety("Move forward slowly")
        
        assert result['is_safe'] is True
        assert result['requires_confirmation'] is False
    
    def test_fast_movement_requires_confirmation(self, validator):
        """Test that fast movement requires confirmation."""
        result = validator.check_safety("Move forward fast")
        
        assert result['requires_confirmation'] is True
        assert result['confirmation_prompt'] is not None
    
    def test_sprint_requires_confirmation(self, validator):
        """Test that sprint requires confirmation."""
        result = validator.check_safety("Sprint forward")
        
        assert result['requires_confirmation'] is True
    
    def test_humans_nearby_warning(self, validator):
        """Test warning when humans nearby."""
        environment = {'humans_nearby': True}
        result = validator.check_safety("Move forward", environment)
        
        assert result['requires_confirmation'] is True
        assert any("humans" in w.lower() for w in result['warnings'])
    
    def test_obstacles_warning(self, validator):
        """Test warning when obstacles nearby."""
        environment = {'obstacles_nearby': True}
        result = validator.check_safety("Move forward", environment)
        
        assert any("obstacles" in w.lower() for w in result['warnings'])
    
    def test_tight_space_confirmation(self, validator):
        """Test confirmation in tight space."""
        environment = {'tight_space': True}
        result = validator.check_safety("Move forward", environment)
        
        assert result['requires_confirmation'] is True
    
    def test_get_confirmation_prompt(self, validator):
        """Test getting confirmation prompt."""
        prompt = validator.get_safety_confirmation("Move fast")
        
        assert prompt is not None
        assert "confirm" in prompt.lower()
    
    def test_no_confirmation_for_safe(self, validator):
        """Test no confirmation for safe command."""
        prompt = validator.get_safety_confirmation("Move slowly")
        
        assert prompt is None


class TestConvenienceFunctions:
    """Test convenience functions."""
    
    def test_validate_input_safe_valid(self):
        """Test validate_input_safe with valid input."""
        result = validate_input_safe("Turn left")
        
        assert result.is_valid is True
    
    def test_validate_input_safe_invalid(self):
        """Test validate_input_safe with invalid input."""
        result = validate_input_safe("; rm -rf /")
        
        assert result.is_valid is False
    
    def test_is_command_safe_true(self):
        """Test is_command_safe returns True for safe."""
        assert is_command_safe("Move slowly") is True
    
    def test_is_command_safe_false(self):
        """Test is_command_safe returns False for unsafe."""
        assert is_command_safe("Move fast") is False


class TestIntegrationWithNL:
    """Test validation integrated with NL processing."""
    
    def test_malicious_input_blocked_before_nl(self):
        """Test that malicious input is blocked before NL processing."""
        from agent_ros_bridge.integrations.nl_interpreter import RuleBasedInterpreter
        
        validator = InputValidator()
        interpreter = RuleBasedInterpreter()
        
        # Malicious input
        malicious = "Move forward; cat /etc/passwd"
        
        # Should be blocked by validator
        validation = validator.validate(malicious)
        assert validation.is_valid is False
        
        # Should not reach interpreter
        # (In real usage, would check validation before calling interpreter)
    
    def test_safe_input_reaches_interpreter(self):
        """Test that safe input reaches interpreter."""
        from agent_ros_bridge.integrations.nl_interpreter import RuleBasedInterpreter
        
        validator = InputValidator()
        interpreter = RuleBasedInterpreter()
        
        # Safe input
        safe = "Move forward 2 meters"
        
        # Should pass validator
        validation = validator.validate(safe)
        assert validation.is_valid is True
        
        # Should work with interpreter
        result = interpreter.interpret(validation.sanitized_input)
        assert result["tool"] == "ros2_publish"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
