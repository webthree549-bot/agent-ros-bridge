"""Tests for security utilities."""

import os
from unittest.mock import Mock, patch

import pytest

from agent_ros_bridge.security_utils import (
    AuditLogger,
    RateLimiter,
    SecureConfig,
    decrypt,
    encrypt,
    generate_api_key,
    generate_token,
    hash_api_key,
    hash_password,
    sanitize_input,
    validate_api_key,
    validate_robot_id,
    verify_password,
)


class TestSecureConfig:
    """Test SecureConfig class."""

    def test_get_api_key_openai(self):
        """Test getting OpenAI API key."""
        with patch.dict(os.environ, {"OPENAI_API_KEY": "sk-test123"}):
            key = SecureConfig.get_api_key("openai")
            assert key == "sk-test123"

    def test_get_api_key_anthropic(self):
        """Test getting Anthropic API key."""
        with patch.dict(os.environ, {"ANTHROPIC_API_KEY": "sk-ant-test123"}):
            key = SecureConfig.get_api_key("anthropic")
            assert key == "sk-ant-test123"

    def test_get_api_key_not_found(self):
        """Test getting non-existent API key."""
        with patch.dict(os.environ, {}, clear=True):
            key = SecureConfig.get_api_key("unknown")
            assert key is None

    def test_redact_sensitive_openai(self):
        """Test redacting OpenAI key."""
        text = "Error with sk-abcdefghijklmnopqrstuvwxyz123456789012345678901234"
        redacted = SecureConfig.redact_sensitive(text)
        assert "sk-***" in redacted
        assert "abcdefghijklmnopqrstuvwxyz" not in redacted

    def test_redact_sensitive_anthropic(self):
        """Test redacting Anthropic key."""
        text = "Error with sk-ant-abcdefghijklmnopqrstuvwxyz1234"
        redacted = SecureConfig.redact_sensitive(text)
        assert "sk-ant-***" in redacted

    def test_validate_api_key_openai_valid(self):
        """Test validating valid OpenAI key."""
        key = "sk-" + "a" * 48
        assert SecureConfig.validate_api_key(key, "openai") is True

    def test_validate_api_key_openai_invalid(self):
        """Test validating invalid OpenAI key."""
        assert SecureConfig.validate_api_key("invalid", "openai") is False
        assert SecureConfig.validate_api_key("", "openai") is False

    def test_validate_api_key_anthropic_valid(self):
        """Test validating valid Anthropic key."""
        assert SecureConfig.validate_api_key("sk-ant-test123", "anthropic") is True

    def test_validate_api_key_anthropic_invalid(self):
        """Test validating invalid Anthropic key."""
        assert SecureConfig.validate_api_key("sk-test", "anthropic") is False

    def test_mask_key_long(self):
        """Test masking long key."""
        key = "sk-abcdefghijklmnopqrstuvwxyz1234567890"
        masked = SecureConfig.mask_key(key)
        assert masked.startswith("sk-a")
        assert masked.endswith("7890")
        assert "..." in masked

    def test_mask_key_short(self):
        """Test masking short key."""
        assert SecureConfig.mask_key("abc") == "***"
        assert SecureConfig.mask_key("") == "***"


class TestRateLimiter:
    """Test RateLimiter class."""

    def test_init(self):
        """Test initialization."""
        limiter = RateLimiter(max_calls=10, window_sec=60)
        assert limiter.max_calls == 10
        assert limiter.window_sec == 60

    def test_is_allowed_under_limit(self):
        """Test calls under limit are allowed."""
        limiter = RateLimiter(max_calls=5, window_sec=60)
        for _ in range(3):
            assert limiter.is_allowed() is True

    def test_is_allowed_over_limit(self):
        """Test calls over limit are blocked."""
        limiter = RateLimiter(max_calls=2, window_sec=60)
        assert limiter.is_allowed() is True
        assert limiter.is_allowed() is True
        assert limiter.is_allowed() is False

    def test_get_remaining(self):
        """Test getting remaining calls."""
        limiter = RateLimiter(max_calls=5, window_sec=60)
        assert limiter.get_remaining() == 5
        limiter.is_allowed()
        assert limiter.get_remaining() == 4

    def test_allow_request_client(self):
        """Test per-client rate limiting."""
        limiter = RateLimiter(max_calls=2, window_sec=60)
        assert limiter.allow_request("client1") is True
        assert limiter.allow_request("client1") is True
        assert limiter.allow_request("client1") is False
        # Different client should still be allowed
        assert limiter.allow_request("client2") is True


class TestAuditLogger:
    """Test AuditLogger class."""

    def test_log_api_call(self):
        """Test logging API call."""
        mock_logger = Mock()
        audit = AuditLogger(mock_logger)
        audit.log_api_call("openai", True, 150.0)
        mock_logger.info.assert_called_once()
        assert "openai" in mock_logger.info.call_args[0][0]

    def test_log_auth_failure(self):
        """Test logging auth failure."""
        mock_logger = Mock()
        audit = AuditLogger(mock_logger)
        audit.log_auth_failure("anthropic", "invalid_key")
        mock_logger.warning.assert_called_once()
        assert "Auth failure" in mock_logger.warning.call_args[0][0]

    def test_log_rate_limit(self):
        """Test logging rate limit."""
        mock_logger = Mock()
        audit = AuditLogger(mock_logger)
        audit.log_rate_limit("openai", 0)
        mock_logger.warning.assert_called_once()
        assert "Rate limit" in mock_logger.warning.call_args[0][0]

    def test_log_suspicious_input(self):
        """Test logging suspicious input."""
        mock_logger = Mock()
        audit = AuditLogger(mock_logger)
        audit.log_suspicious_input("command", "rm -rf /")
        mock_logger.warning.assert_called_once()
        assert "Suspicious input" in mock_logger.warning.call_args[0][0]


class TestSanitizeInput:
    """Test input sanitization."""

    def test_sanitize_empty(self):
        """Test sanitizing empty input."""
        assert sanitize_input("") == ""
        assert sanitize_input(None) == ""

    def test_sanitize_control_chars(self):
        """Test removing control characters."""
        text = "Hello\x00\x01\x02World"
        assert "\x00" not in sanitize_input(text)
        assert "\x01" not in sanitize_input(text)

    def test_sanitize_html(self):
        """Test removing HTML tags."""
        text = "Hello <script>alert('xss')</script> World"
        sanitized = sanitize_input(text)
        assert "<script>" not in sanitized
        assert "</script>" not in sanitized

    def test_sanitize_whitespace(self):
        """Test normalizing whitespace."""
        text = "Hello    World\t\t\tTest"
        sanitized = sanitize_input(text)
        assert "    " not in sanitized

    def test_sanitize_max_length(self):
        """Test max length truncation."""
        text = "a" * 2000
        sanitized = sanitize_input(text, max_length=100)
        assert len(sanitized) == 100


class TestPasswordHashing:
    """Test password hashing."""

    def test_hash_password(self):
        """Test password hashing."""
        password = "mysecretpassword"
        hashed = hash_password(password)
        assert len(hashed) > 32  # salt + hash

    def test_verify_password_correct(self):
        """Test verifying correct password."""
        password = "mysecretpassword"
        hashed = hash_password(password)
        assert verify_password(password, hashed) is True

    def test_verify_password_incorrect(self):
        """Test verifying incorrect password."""
        password = "mysecretpassword"
        hashed = hash_password(password)
        assert verify_password("wrongpassword", hashed) is False

    def test_verify_password_invalid_hash(self):
        """Test verifying with invalid hash."""
        assert verify_password("password", "short") is False


class TestTokenGeneration:
    """Test token generation."""

    def test_generate_token(self):
        """Test generating token."""
        token = generate_token(32)
        assert len(token) == 32

    def test_generate_token_different(self):
        """Test that tokens are different."""
        token1 = generate_token(32)
        token2 = generate_token(32)
        assert token1 != token2


class TestApiKeyGeneration:
    """Test API key generation."""

    def test_generate_api_key(self):
        """Test generating API key."""
        result = generate_api_key("test_key")
        assert "key" in result
        assert "name" in result
        assert "created" in result
        assert result["name"] == "test_key"
        assert result["key"].startswith("arb_")

    def test_hash_api_key(self):
        """Test hashing API key."""
        key = "test_key_123"
        hashed = hash_api_key(key)
        assert len(hashed) == 64  # SHA-256 hex

    def test_validate_api_key(self):
        """Test validating API key."""
        key = "test_key_123"
        hashed = hash_api_key(key)
        assert validate_api_key(key, hashed) is True
        assert validate_api_key("wrong_key", hashed) is False


class TestEncryption:
    """Test encryption/decryption."""

    def test_encrypt_decrypt(self):
        """Test encrypting and decrypting."""
        data = "secret message"
        key = b"a" * 32  # 32 bytes for Fernet
        encrypted = encrypt(data, key)
        decrypted = decrypt(encrypted, key)
        assert decrypted == data

    def test_encrypt_different_keys(self):
        """Test that different keys produce different results."""
        data = "secret message"
        key1 = b"a" * 32
        key2 = b"b" * 32
        encrypted1 = encrypt(data, key1)
        encrypted2 = encrypt(data, key2)
        assert encrypted1 != encrypted2


class TestValidateRobotId:
    """Test robot ID validation."""

    def test_valid_robot_id(self):
        """Test valid robot IDs."""
        assert validate_robot_id("robot1") is True
        assert validate_robot_id("robot_1") is True
        assert validate_robot_id("robot-1") is True

    def test_invalid_robot_id_empty(self):
        """Test empty robot ID."""
        assert validate_robot_id("") is False
        assert validate_robot_id(None) is False

    def test_invalid_robot_id_too_long(self):
        """Test too long robot ID."""
        assert validate_robot_id("a" * 100) is False

    def test_invalid_robot_id_chars(self):
        """Test invalid characters."""
        assert validate_robot_id("robot<1>") is False
        assert validate_robot_id('robot"1"') is False
        assert validate_robot_id("robot 1") is False  # space
