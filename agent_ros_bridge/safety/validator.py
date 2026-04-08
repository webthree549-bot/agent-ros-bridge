"""Safety validation for robot commands."""

from dataclasses import dataclass
from typing import Any


@dataclass
class ValidationResult:
    """Result of safety validation."""
    is_safe: bool
    reason: str
    severity: str = "info"  # info, warning, error


class SafetyValidator:
    """Validates robot commands for safety."""
    
    # Safety limits
    MAX_LINEAR_VELOCITY = 2.0  # m/s
    MAX_ANGULAR_VELOCITY = 1.0  # rad/s
    
    def __init__(self, enable_cache: bool = True):
        self.violations = []
        self.enable_cache = enable_cache
    
    def validate_command(self, command: Any) -> ValidationResult:
        """Validate a command for safety.
        
        Args:
            command: Command to validate
            
        Returns:
            ValidationResult with safety status
        """
        if command is None:
            return ValidationResult(
                is_safe=False,
                reason="Command cannot be None",
                severity="error"
            )
        
        if command == "":
            return ValidationResult(
                is_safe=False,
                reason="Command cannot be empty",
                severity="error"
            )
        
        # Command is valid
        return ValidationResult(
            is_safe=True,
            reason="Command passed safety checks",
            severity="info"
        )
    
    def validate_velocity(self, cmd: dict[str, Any]) -> ValidationResult:
        """Validate velocity command.
        
        Args:
            cmd: Velocity command with linear and angular components
            
        Returns:
            ValidationResult with safety status
        """
        # Check linear velocity
        linear = cmd.get("linear", {})
        linear_x = linear.get("x", 0.0)
        
        if abs(linear_x) > self.MAX_LINEAR_VELOCITY:
            return ValidationResult(
                is_safe=False,
                reason=f"Linear velocity {linear_x} exceeds limit {self.MAX_LINEAR_VELOCITY}",
                severity="error"
            )
        
        # Check angular velocity
        angular = cmd.get("angular", {})
        angular_z = angular.get("z", 0.0)
        
        if abs(angular_z) > self.MAX_ANGULAR_VELOCITY:
            return ValidationResult(
                is_safe=False,
                reason=f"Angular velocity {angular_z} exceeds limit {self.MAX_ANGULAR_VELOCITY}",
                severity="error"
            )
        
        return ValidationResult(
            is_safe=True,
            reason="Velocity within safe limits",
            severity="info"
        )


# Alias for backwards compatibility
SafetyValidatorNode = SafetyValidator
