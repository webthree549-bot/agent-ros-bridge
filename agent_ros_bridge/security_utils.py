"""
Security Utilities for Agent ROS Bridge
v0.6.1 - Week 5 Production Hardening

Provides secure handling of sensitive data like API keys.
"""

import os
import re
import hashlib
import secrets
import time
from typing import Optional, Dict
from cryptography.fernet import Fernet


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
    
    def __init__(self, max_calls: int = 100, window_sec: int = 60, max_requests: int = None, window: int = None):
        """
        Initialize rate limiter.
        
        Args:
            max_calls: Maximum calls per window
            window_sec: Time window in seconds
            max_requests: Alias for max_calls (for compatibility)
            window: Alias for window_sec (for compatibility)
        """
        self.max_calls = max_requests if max_requests is not None else max_calls
        self.window_sec = window if window is not None else window_sec
        self.calls = []
        self.client_calls: Dict[str, list] = {}
    
    def is_allowed(self) -> bool:
        """
        Check if call is allowed under rate limit.
        
        Returns:
            True if call should proceed
        """
        now = time.time()
        
        # Remove old calls outside window
        self.calls = [t for t in self.calls if now - t < self.window_sec]
        
        # Check if under limit
        if len(self.calls) < self.max_calls:
            self.calls.append(now)
            return True
        
        return False
    
    def allow_request(self, client_id: str) -> bool:
        """
        Check if request from client is allowed.
        
        Args:
            client_id: Client identifier
            
        Returns:
            True if request should proceed
        """
        now = time.time()
        
        # Initialize client if not exists
        if client_id not in self.client_calls:
            self.client_calls[client_id] = []
        
        # Remove old calls outside window
        self.client_calls[client_id] = [
            t for t in self.client_calls[client_id] 
            if now - t < self.window_sec
        ]
        
        # Check if under limit
        if len(self.client_calls[client_id]) < self.max_calls:
            self.client_calls[client_id].append(now)
            return True
        
        return False
    
    def get_remaining(self) -> int:
        """Get remaining calls in current window."""
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
    
    # Remove HTML/script tags
    text = re.sub(r'<[^>]+>', '', text)
    
    # Normalize whitespace
    text = ' '.join(text.split())
    
    return text.strip()


def hash_password(password: str) -> str:
    """
    Hash a password using PBKDF2.
    
    Args:
        password: Plain text password
        
    Returns:
        Hashed password string
    """
    salt = secrets.token_hex(16)
    pwdhash = hashlib.pbkdf2_hmac('sha256', password.encode(), salt.encode(), 100000)
    return salt + pwdhash.hex()


def verify_password(password: str, hashed: str) -> bool:
    """
    Verify a password against its hash.
    
    Args:
        password: Plain text password
        hashed: Stored hash
        
    Returns:
        True if password matches
    """
    if len(hashed) < 64:
        return False
    salt = hashed[:32]
    stored_hash = hashed[32:]
    pwdhash = hashlib.pbkdf2_hmac('sha256', password.encode(), salt.encode(), 100000)
    return pwdhash.hex() == stored_hash


def generate_token(length: int = 32) -> str:
    """
    Generate a secure random token.
    
    Args:
        length: Token length (approximate, actual may vary due to base64 encoding)
        
    Returns:
        Random token string
    """
    # token_urlsafe generates base64 encoded bytes, so actual length may differ
    import base64
    # Generate enough bytes to get at least the requested length
    num_bytes = (length * 3) // 4 + 1
    return base64.urlsafe_b64encode(secrets.token_bytes(num_bytes)).decode()[:length]


def generate_api_key(name: str) -> Dict[str, str]:
    """
    Generate an API key with metadata.
    
    Args:
        name: Key name/identifier
        
    Returns:
        Dictionary with key and metadata
    """
    key = "arb_" + secrets.token_urlsafe(32)
    return {
        "key": key,
        "name": name,
        "created": str(time.time())
    }


def hash_api_key(key: str) -> str:
    """
    Hash an API key for storage.
    
    Args:
        key: API key
        
    Returns:
        Hashed key
    """
    return hashlib.sha256(key.encode()).hexdigest()


def validate_api_key(key: str, stored_hash: str) -> bool:
    """
    Validate an API key against stored hash.
    
    Args:
        key: API key to validate
        stored_hash: Stored hash
        
    Returns:
        True if valid
    """
    return hashlib.sha256(key.encode()).hexdigest() == stored_hash


def encrypt(data: str, key: bytes) -> bytes:
    """
    Encrypt data using Fernet.
    
    Args:
        data: Data to encrypt
        key: Encryption key (32 bytes, will be base64 encoded)
        
    Returns:
        Encrypted bytes
    """
    import base64
    # Fernet requires base64-encoded 32-byte key
    if len(key) == 32:
        key = base64.urlsafe_b64encode(key)
    f = Fernet(key)
    return f.encrypt(data.encode())


def decrypt(encrypted: bytes, key: bytes) -> str:
    """
    Decrypt data using Fernet.
    
    Args:
        encrypted: Encrypted bytes
        key: Encryption key (will be base64 encoded if needed)
        
    Returns:
        Decrypted string
    """
    import base64
    # Fernet requires base64-encoded 32-byte key
    if len(key) == 32:
        key = base64.urlsafe_b64encode(key)
    f = Fernet(key)
    return f.decrypt(encrypted).decode()


def validate_robot_id(robot_id: str) -> bool:
    """
    Validate robot ID format.
    
    Args:
        robot_id: Robot identifier
        
    Returns:
        True if valid
    """
    if not robot_id or len(robot_id) > 64:
        return False
    # Only allow alphanumeric, underscore, hyphen
    if not re.match(r'^[a-zA-Z0-9_-]+$', robot_id):
        return False
    # Check for dangerous patterns
    if '<' in robot_id or '>' in robot_id or '"' in robot_id:
        return False
    return True


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
