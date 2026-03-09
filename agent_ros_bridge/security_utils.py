"""
Security Utilities for Agent ROS Bridge
v0.6.1 - Week 5 Production Hardening

Provides secure handling of sensitive data like API keys.
"""

import os
import re
from typing import Optional


class SecureConfig:
    """
    Secure configuration management.
    
    Ensures sensitive values like API keys are:
    - Loaded from environment variables
    - Not logged or exposed in errors
    - Validated before use
    """
    
    # Patterns that look like API keys (for redaction)
    SENSITIVE_PATTERNS = [
        (r'sk-[a-zA-Z0-9]{48}', 'sk-***'),  # OpenAI
        (r'sk-ant-[a-zA-Z0-9]{32}', 'sk-ant-***'),  # Anthropic
        (r'[a-f0-9]{64}', '***'),  # Generic hex keys
    ]
    
    @classmethod
    def get_api_key(cls, provider: str) -> Optional[str]:
        """
        Securely get API key from environment.
        
        Args:
            provider: 'openai', 'anthropic', etc.
            
        Returns:
            API key or None if not set
        """
        env_vars = {
            'openai': ['OPENAI_API_KEY', 'OPEN_AI_KEY'],
            'anthropic': ['ANTHROPIC_API_KEY', 'ANTHROPIC_KEY'],
        }
        
        for var in env_vars.get(provider, []):
            key = os.getenv(var)
            if key:
                return key
        
        return None
    
    @classmethod
    def redact_sensitive(cls, text: str) -> str:
        """
        Redact sensitive information from text.
        
        Args:
            text: Input text that may contain secrets
            
        Returns:
            Text with sensitive values redacted
        """
        result = text
        for pattern, replacement in cls.SENSITIVE_PATTERNS:
            result = re.sub(pattern, replacement, result)
        return result
    
    @classmethod
    def validate_api_key(cls, key: str, provider: str) -> bool:
        """
        Validate API key format.
        
        Args:
            key: API key to validate
            provider: Expected provider
            
        Returns:
            True if valid format
        """
        if not key or not isinstance(key, str):
            return False
        
        if provider == 'openai':
            # OpenAI keys start with 'sk-' and are 51 chars
            return key.startswith('sk-') and len(key) >= 40
        
        if provider == 'anthropic':
            # Anthropic keys start with 'sk-ant-'
            return key.startswith('sk-ant-')
        
        return True  # Unknown provider, accept any
    
    @classmethod
    def mask_key(cls, key: str) -> str:
        """
        Mask API key for display (show only first/last 4 chars).
        
        Args:
            key: API key to mask
            
        Returns:
            Masked key like 'sk-12...ab'
        """
        if not key or len(key) < 8:
            return '***'
        
        return f"{key[:4]}...{key[-4:]}"


class RateLimiter:
    """
    Simple rate limiter for API calls.
    
    Prevents excessive API usage and costs.
    """
    
    def __init__(self, max_calls: int = 100, window_sec: int = 60):
        """
        Initialize rate limiter.
        
        Args:
            max_calls: Maximum calls per window
            window_sec: Time window in seconds
        """
        self.max_calls = max_calls
        self.window_sec = window_sec
        self.calls = []
    
    def is_allowed(self) -> bool:
        """
        Check if call is allowed under rate limit.
        
        Returns:
            True if call should proceed
        """
        import time
        
        now = time.time()
        
        # Remove old calls outside window
        self.calls = [t for t in self.calls if now - t < self.window_sec]
        
        # Check if under limit
        if len(self.calls) < self.max_calls:
            self.calls.append(now)
            return True
        
        return False
    
    def get_remaining(self) -> int:
        """Get remaining calls in current window."""
        import time
        
        now = time.time()
        self.calls = [t for t in self.calls if now - t < self.window_sec]
        return max(0, self.max_calls - len(self.calls))


class AuditLogger:
    """
    Security audit logging.
    
    Logs security-relevant events without exposing sensitive data.
    """
    
    def __init__(self, logger):
        """
        Initialize audit logger.
        
        Args:
            logger: Python logger instance
        """
        self.logger = logger
    
    def log_api_call(self, provider: str, success: bool, latency_ms: float):
        """Log API call (without key)."""
        status = "success" if success else "failure"
        self.logger.info(f"API call: provider={provider}, status={status}, latency={latency_ms:.2f}ms")
    
    def log_auth_failure(self, provider: str, reason: str):
        """Log authentication failure."""
        self.logger.warning(f"Auth failure: provider={provider}, reason={reason}")
    
    def log_rate_limit(self, provider: str, remaining: int):
        """Log rate limit hit."""
        self.logger.warning(f"Rate limit: provider={provider}, remaining={remaining}")
    
    def log_suspicious_input(self, input_type: str, details: str):
        """Log potentially malicious input."""
        # Redact any sensitive info from details
        safe_details = SecureConfig.redact_sensitive(details)
        self.logger.warning(f"Suspicious input: type={input_type}, details={safe_details}")


def sanitize_input(text: str, max_length: int = 1000) -> str:
    """
    Sanitize user input.
    
    Args:
        text: Raw user input
        max_length: Maximum allowed length
        
    Returns:
        Sanitized text
    """
    if not text:
        return ""
    
    # Truncate if too long
    if len(text) > max_length:
        text = text[:max_length]
    
    # Remove control characters except newlines
    text = ''.join(c for c in text if c == '\n' or ord(c) >= 32)
    
    # Normalize whitespace
    text = ' '.join(text.split())
    
    return text.strip()


def main():
    """Test security utilities."""
    import logging
    
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    # Test API key masking
    key = "sk-abcdefghijklmnopqrstuvwxyz12345678901234567890"
    print(f"Original: {key}")
    print(f"Masked: {SecureConfig.mask_key(key)}")
    print(f"Redacted: {SecureConfig.redact_sensitive(f'Error with key {key}')}")
    
    # Test rate limiter
    limiter = RateLimiter(max_calls=3, window_sec=10)
    for i in range(5):
        allowed = limiter.is_allowed()
        print(f"Call {i+1}: {'allowed' if allowed else 'blocked'}, remaining={limiter.get_remaining()}")
    
    # Test audit logger
    audit = AuditLogger(logger)
    audit.log_api_call("openai", True, 150.0)
    audit.log_auth_failure("anthropic", "invalid_key")
    
    # Test input sanitization
    dirty = "Hello\x00\x01\x02\nWorld\t\t\t   "
    clean = sanitize_input(dirty)
    print(f"Sanitized: '{clean}'")


if __name__ == "__main__":
    main()
