"""Input validation and sanitization for natural language commands.

Provides security and safety validation for user inputs.
"""

import re
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class ValidationResult:
    """Result of input validation."""
    is_valid: bool
    sanitized_input: str
    errors: List[str]
    warnings: List[str]


class InputValidator:
    """Validates and sanitizes natural language inputs.
    
    Security features:
    - Command injection prevention
    - Dangerous character filtering
    - Length limits
    - Whitelist validation
    """
    
    # Maximum input length
    MAX_LENGTH = 500
    
    # Dangerous patterns to block
    DANGEROUS_PATTERNS = [
        r'[;&|]',  # Command separators
        r'\$\(',   # Command substitution
        r'`',      # Backtick execution
        r'\$\{',   # Variable expansion
        r'[<>]',   # Redirection
        r'\.\./',  # Path traversal
        r'javascript:',  # XSS
        r'data:',   # Data URI
    ]
    
    # Allowed characters (whitelist approach)
    ALLOWED_CHARS = re.compile(r'^[\w\s\-\.,;:!?\'"()[\]]+$')
    
    # Dangerous keywords
    DANGEROUS_KEYWORDS = [
        'rm', 'del', 'delete', 'format', 'drop', 'exec',
        'eval', 'system', 'shell', 'bash', 'sh ', 'cmd ',
        'powershell', 'script', 'code', 'import', 'export'
    ]
    
    def __init__(self):
        """Initialize validator."""
        self.dangerous_regex = re.compile(
            '|'.join(self.DANGEROUS_PATTERNS),
            re.IGNORECASE
        )
    
    def validate(self, user_input: str, 
                 context: Optional[Dict] = None) -> ValidationResult:
        """Validate and sanitize user input.
        
        Args:
            user_input: Raw user input
            context: Optional context for validation
            
        Returns:
            ValidationResult with status and sanitized input
        """
        errors = []
        warnings = []
        
        # Check for empty input
        if not user_input or not user_input.strip():
            return ValidationResult(
                is_valid=False,
                sanitized_input="",
                errors=["Input cannot be empty"],
                warnings=[]
            )
        
        # Check length
        if len(user_input) > self.MAX_LENGTH:
            errors.append(f"Input too long (max {self.MAX_LENGTH} characters)")
            user_input = user_input[:self.MAX_LENGTH]
        
        # Check for dangerous patterns
        if self.dangerous_regex.search(user_input):
            errors.append("Input contains potentially dangerous characters")
        
        # Check for dangerous keywords
        input_lower = user_input.lower()
        for keyword in self.DANGEROUS_KEYWORDS:
            if keyword in input_lower:
                errors.append(f"Input contains restricted keyword: {keyword}")
        
        # Sanitize input
        sanitized = self._sanitize(user_input)
        
        # Context-specific validation
        if context:
            context_errors, context_warnings = self._validate_context(sanitized, context)
            errors.extend(context_errors)
            warnings.extend(context_warnings)
        
        return ValidationResult(
            is_valid=len(errors) == 0,
            sanitized_input=sanitized,
            errors=errors,
            warnings=warnings
        )
    
    def _sanitize(self, user_input: str) -> str:
        """Sanitize input by removing dangerous characters.
        
        Args:
            user_input: Raw input
            
        Returns:
            Sanitized input
        """
        # Remove null bytes
        sanitized = user_input.replace('\x00', '')
        
        # Remove control characters
        sanitized = ''.join(char for char in sanitized if ord(char) >= 32 or char == '\n')
        
        # Normalize whitespace
        sanitized = ' '.join(sanitized.split())
        
        # Strip dangerous characters
        sanitized = re.sub(r'[;&|`$<>]', '', sanitized)
        
        return sanitized.strip()
    
    def _validate_context(self, sanitized: str, 
                         context: Dict) -> Tuple[List[str], List[str]]:
        """Validate input against context.
        
        Args:
            sanitized: Sanitized input
            context: Context dictionary
            
        Returns:
            Tuple of (errors, warnings)
        """
        errors = []
        warnings = []
        
        # Check if robot is in a safe state
        robot_status = context.get('robot_status', 'unknown')
        if robot_status == 'error':
            warnings.append("Robot is in error state, commands may fail")
        
        # Check battery level
        battery = context.get('battery_percent', 100)
        if battery < 20:
            warnings.append(f"Low battery ({battery}%), consider charging")
        
        # Check if emergency stop is active
        if context.get('emergency_stop', False):
            errors.append("Emergency stop is active, cannot execute commands")
        
        # Validate against known locations
        known_locations = context.get('known_locations', [])
        for location in known_locations:
            if location.lower() in sanitized.lower():
                # Location is known, good
                break
        
        return errors, warnings
    
    def validate_robot_command(self, command: str, 
                              robot_capabilities: Dict) -> ValidationResult:
        """Validate command against robot capabilities.
        
        Args:
            command: Command to validate
            robot_capabilities: Robot's capabilities
            
        Returns:
            ValidationResult
        """
        # First do basic validation
        result = self.validate(command)
        
        if not result.is_valid:
            return result
        
        # Check capability requirements
        command_lower = result.sanitized_input.lower()
        
        # Navigation commands require can_navigate
        if any(word in command_lower for word in ['go', 'move', 'navigate', 'drive']):
            if not robot_capabilities.get('can_navigate', True):
                result.errors.append("Robot does not support navigation")
                result.is_valid = False
        
        # Manipulation commands require can_manipulate
        if any(word in command_lower for word in ['pick', 'grab', 'lift', 'manipulate']):
            if not robot_capabilities.get('can_manipulate', False):
                result.errors.append("Robot does not support manipulation")
                result.is_valid = False
        
        return result


class SafetyValidator:
    """Validates commands for safety concerns.
    
    Checks for potentially dangerous robot commands.
    """
    
    # Commands that require confirmation
    DANGEROUS_COMMANDS = [
        'fast', 'quick', 'sprint', 'run',
        'spin fast', 'rotate fast',
        'move fast', 'drive fast',
    ]
    
    # Commands that require safety check
    SAFETY_CRITICAL = [
        'emergency stop', 'e-stop', 'halt',
        'reset', 'restart', 'reboot',
    ]
    
    def __init__(self):
        """Initialize safety validator."""
        pass
    
    def check_safety(self, command: str, 
                    environment: Optional[Dict] = None) -> Dict[str, Any]:
        """Check if command is safe to execute.
        
        Args:
            command: Sanitized command
            environment: Optional environment info
            
        Returns:
            Safety check result
        """
        command_lower = command.lower()
        
        result = {
            'is_safe': True,
            'requires_confirmation': False,
            'confirmation_prompt': None,
            'warnings': [],
            'safety_checks': []
        }
        
        # Check for dangerous commands
        for dangerous in self.DANGEROUS_COMMANDS:
            if dangerous in command_lower:
                result['requires_confirmation'] = True
                result['confirmation_prompt'] = (
                    f"This command involves {dangerous} movement. "
                    "Please confirm it's safe to proceed."
                )
                result['warnings'].append(f"High-speed movement detected")
        
        # Check for safety-critical commands
        for critical in self.SAFETY_CRITICAL:
            if critical in command_lower:
                result['safety_checks'].append(f"Safety-critical: {critical}")
        
        # Environment checks
        if environment:
            # Check for humans nearby
            if environment.get('humans_nearby', False):
                result['requires_confirmation'] = True
                result['warnings'].append("Humans detected nearby")
            
            # Check for obstacles
            if environment.get('obstacles_nearby', False):
                result['warnings'].append("Obstacles detected nearby")
            
            # Check space constraints
            if environment.get('tight_space', False):
                result['requires_confirmation'] = True
                result['warnings'].append("Operating in tight space")
        
        return result
    
    def get_safety_confirmation(self, command: str) -> Optional[str]:
        """Get confirmation prompt for dangerous command.
        
        Args:
            command: Command to check
            
        Returns:
            Confirmation prompt or None
        """
        safety = self.check_safety(command)
        
        if safety['requires_confirmation']:
            return safety['confirmation_prompt'] or "Please confirm this action"
        
        return None


# Convenience functions

def validate_input_safe(user_input: str, 
                       context: Optional[Dict] = None) -> ValidationResult:
    """Quick validation function.
    
    Convenience function for one-off validation.
    """
    validator = InputValidator()
    return validator.validate(user_input, context)


def is_command_safe(command: str, 
                   environment: Optional[Dict] = None) -> bool:
    """Quick safety check.
    
    Convenience function for safety checking.
    """
    validator = SafetyValidator()
    result = validator.check_safety(command, environment)
    return result['is_safe'] and not result['requires_confirmation']
