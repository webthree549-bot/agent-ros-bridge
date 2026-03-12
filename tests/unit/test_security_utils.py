"""Tests for security_utils module - TDD approach.

This module currently has 0% coverage. Following TDD:
1. Write tests first (this file)
2. Implement minimal code to pass
3. Refactor
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import hashlib
import secrets

from agent_ros_bridge import security_utils


class TestPasswordHashing:
    """Test password hashing functionality."""

    def test_hash_password_returns_string(self):
        """hash_password returns a hashed string."""
        password = "mysecretpassword"

        hashed = security_utils.hash_password(password)

        assert isinstance(hashed, str)
        assert hashed != password

    def test_hash_password_same_input_different_output(self):
        """Same password produces different hashes (due to salt)."""
        password = "mysecretpassword"

        hash1 = security_utils.hash_password(password)
        hash2 = security_utils.hash_password(password)

        assert hash1 != hash2

    def test_verify_password_correct(self):
        """verify_password returns True for correct password."""
        password = "mysecretpassword"
        hashed = security_utils.hash_password(password)

        result = security_utils.verify_password(password, hashed)

        assert result is True

    def test_verify_password_incorrect(self):
        """verify_password returns False for incorrect password."""
        password = "mysecretpassword"
        wrong_password = "wrongpassword"
        hashed = security_utils.hash_password(password)

        result = security_utils.verify_password(wrong_password, hashed)

        assert result is False


class TestTokenGeneration:
    """Test secure token generation."""

    def test_generate_token_returns_string(self):
        """generate_token returns a string token."""
        token = security_utils.generate_token()

        assert isinstance(token, str)
        assert len(token) > 0

    def test_generate_token_unique(self):
        """Each token is unique."""
        token1 = security_utils.generate_token()
        token2 = security_utils.generate_token()

        assert token1 != token2

    def test_generate_token_with_length(self):
        """Token can be generated with specific length."""
        token = security_utils.generate_token(length=32)

        assert len(token) == 32


class TestAPIKeyManagement:
    """Test API key management."""

    def test_generate_api_key_returns_dict(self):
        """generate_api_key returns key metadata."""
        key_data = security_utils.generate_api_key("test_key")

        assert isinstance(key_data, dict)
        assert "key" in key_data
        assert "name" in key_data
        assert key_data["name"] == "test_key"

    def test_generate_api_key_has_prefix(self):
        """API key has proper prefix."""
        key_data = security_utils.generate_api_key("test")

        assert key_data["key"].startswith("arb_")  # agent_ros_bridge prefix

    def test_validate_api_key_valid(self):
        """validate_api_key returns True for valid key."""
        key_data = security_utils.generate_api_key("test")
        stored_hash = security_utils.hash_api_key(key_data["key"])

        result = security_utils.validate_api_key(key_data["key"], stored_hash)

        assert result is True

    def test_validate_api_key_invalid(self):
        """validate_api_key returns False for invalid key."""
        result = security_utils.validate_api_key("invalid_key", "stored_hash")

        assert result is False


class TestEncryption:
    """Test encryption/decryption functionality."""

    def test_encrypt_returns_bytes(self):
        """encrypt returns encrypted bytes."""
        data = "sensitive data"
        key = secrets.token_bytes(32)

        encrypted = security_utils.encrypt(data, key)

        assert isinstance(encrypted, bytes)
        assert encrypted != data.encode()

    def test_decrypt_returns_original(self):
        """decrypt returns original data."""
        data = "sensitive data"
        key = secrets.token_bytes(32)

        encrypted = security_utils.encrypt(data, key)
        decrypted = security_utils.decrypt(encrypted, key)

        assert decrypted == data

    def test_decrypt_wrong_key_fails(self):
        """decrypt fails with wrong key."""
        data = "sensitive data"
        key1 = secrets.token_bytes(32)
        key2 = secrets.token_bytes(32)

        encrypted = security_utils.encrypt(data, key1)

        with pytest.raises(Exception):
            security_utils.decrypt(encrypted, key2)


class TestInputSanitization:
    """Test input sanitization."""

    def test_sanitize_input_removes_dangerous_chars(self):
        """sanitize_input removes dangerous characters."""
        input_str = "hello<script>world</script>"

        sanitized = security_utils.sanitize_input(input_str)

        assert "<script>" not in sanitized

    def test_sanitize_input_allows_safe_chars(self):
        """sanitize_input preserves safe characters."""
        input_str = "hello world 123"

        sanitized = security_utils.sanitize_input(input_str)

        assert sanitized == input_str

    def test_validate_robot_id_valid(self):
        """validate_robot_id accepts valid IDs."""
        assert security_utils.validate_robot_id("robot_123") is True
        assert security_utils.validate_robot_id("turtlebot3") is True

    def test_validate_robot_id_invalid(self):
        """validate_robot_id rejects invalid IDs."""
        assert security_utils.validate_robot_id("") is False
        assert security_utils.validate_robot_id("robot<script>") is False
        assert security_utils.validate_robot_id("a" * 100) is False  # Too long


class TestRateLimiting:
    """Test rate limiting functionality."""

    def test_rate_limiter_allows_first_request(self):
        """Rate limiter allows first request."""
        limiter = security_utils.RateLimiter(max_requests=10, window=60)

        result = limiter.allow_request("client_1")

        assert result is True

    def test_rate_limiter_blocks_excessive_requests(self):
        """Rate limiter blocks excessive requests."""
        limiter = security_utils.RateLimiter(max_requests=2, window=60)

        limiter.allow_request("client_1")
        limiter.allow_request("client_1")
        result = limiter.allow_request("client_1")  # Third request

        assert result is False

    def test_rate_limiter_tracks_clients_separately(self):
        """Rate limiter tracks clients separately."""
        limiter = security_utils.RateLimiter(max_requests=2, window=60)

        limiter.allow_request("client_1")
        limiter.allow_request("client_1")
        result = limiter.allow_request("client_2")  # Different client

        assert result is True
