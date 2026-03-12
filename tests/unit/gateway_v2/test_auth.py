"""Tests for gateway_v2/auth.py - TDD approach.

Improving coverage from 38% to 80%+.
"""
import pytest
from unittest.mock import Mock, patch
from datetime import datetime, timezone, timedelta

import jwt

from agent_ros_bridge.gateway_v2.auth import (
    AuthConfig,
    Authenticator,
)


class TestAuthConfig:
    """Test AuthConfig dataclass."""
    
    def test_default_config(self):
        """Config has secure defaults."""
        config = AuthConfig()
        
        assert config.enabled is True
        assert config.jwt_secret is None
        assert config.jwt_algorithm == "HS256"
        assert config.jwt_expiry_hours == 24
        assert config.api_keys == {}
        assert config.allowed_origins == ["127.0.0.1", "localhost"]
    
    def test_config_with_secret(self):
        """Config can have JWT secret."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        
        assert config.jwt_secret == "test_secret_32_chars_long!!"
    
    def test_config_with_api_keys(self):
        """Config can have API keys."""
        config = AuthConfig(
            api_keys={"key123": {"name": "test", "roles": ["admin"]}}
        )
        
        assert "key123" in config.api_keys
        assert config.api_keys["key123"]["name"] == "test"
    
    def test_config_with_allowed_origins(self):
        """Config can have allowed origins."""
        config = AuthConfig(
            allowed_origins=["https://example.com", "https://app.com"]
        )
        
        assert "https://example.com" in config.allowed_origins


class TestAuthenticatorInit:
    """Test Authenticator initialization."""
    
    def test_init_requires_secret(self):
        """Authenticator requires JWT secret."""
        config = AuthConfig(jwt_secret=None)
        
        with pytest.raises(ValueError) as exc_info:
            Authenticator(config)
        
        assert "JWT_SECRET is required" in str(exc_info.value)
    
    def test_init_with_valid_secret(self):
        """Authenticator can be created with valid secret."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        
        auth = Authenticator(config)
        
        assert auth.config == config


class TestTokenCreation:
    """Test JWT token creation."""
    
    @pytest.fixture
    def authenticator(self):
        """Create authenticator with test secret."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        return Authenticator(config)
    
    def test_create_token_basic(self, authenticator):
        """Can create basic token."""
        token = authenticator.create_token("user123")
        
        assert token is not None
        assert isinstance(token, str)
        assert len(token) > 0
    
    def test_create_token_with_roles(self, authenticator):
        """Can create token with roles."""
        token = authenticator.create_token(
            "user123",
            roles=["admin", "operator"]
        )
        
        # Verify by decoding
        payload = jwt.decode(
            token,
            "test_secret_32_chars_long!!",
            algorithms=["HS256"]
        )
        
        assert payload["sub"] == "user123"
        assert "admin" in payload["roles"]
        assert "operator" in payload["roles"]
    
    def test_create_token_with_metadata(self, authenticator):
        """Can create token with metadata."""
        token = authenticator.create_token(
            "user123",
            metadata={"team": "robotics", "level": 5}
        )
        
        payload = jwt.decode(
            token,
            "test_secret_32_chars_long!!",
            algorithms=["HS256"]
        )
        
        assert payload["metadata"]["team"] == "robotics"
        assert payload["metadata"]["level"] == 5
    
    def test_create_token_fails_when_disabled(self, authenticator):
        """Cannot create token when auth disabled."""
        authenticator.config.enabled = False
        
        with pytest.raises(RuntimeError) as exc_info:
            authenticator.create_token("user123")
        
        assert "Authentication is disabled" in str(exc_info.value)


class TestTokenVerification:
    """Test JWT token verification."""
    
    @pytest.fixture
    def authenticator(self):
        """Create authenticator with test secret."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        return Authenticator(config)
    
    def test_verify_valid_token(self, authenticator):
        """Can verify valid token."""
        token = authenticator.create_token("user123", roles=["admin"])
        
        payload = authenticator.verify_token(token)
        
        assert payload is not None
        assert payload["sub"] == "user123"
        assert "admin" in payload["roles"]
    
    def test_verify_expired_token(self, authenticator):
        """Expired token returns None."""
        # Create token that expires immediately
        authenticator.config.jwt_expiry_hours = -1
        token = authenticator.create_token("user123")
        
        payload = authenticator.verify_token(token)
        
        assert payload is None
    
    def test_verify_invalid_token(self, authenticator):
        """Invalid token returns None."""
        payload = authenticator.verify_token("invalid_token")
        
        assert payload is None
    
    def test_verify_token_wrong_secret(self, authenticator):
        """Token with wrong secret returns None."""
        # Create token with different secret
        wrong_auth = Authenticator(
            AuthConfig(jwt_secret="different_secret_32_chars!!")
        )
        token = wrong_auth.create_token("user123")
        
        payload = authenticator.verify_token(token)
        
        assert payload is None
    
    def test_verify_token_when_disabled(self, authenticator):
        """When disabled, returns anonymous payload."""
        authenticator.config.enabled = False
        
        payload = authenticator.verify_token("any_token")
        
        assert payload is not None
        assert payload["sub"] == "anonymous"
        assert "admin" in payload["roles"]


class TestAPIKeyVerification:
    """Test API key verification."""
    
    @pytest.fixture
    def authenticator_with_keys(self):
        """Create authenticator with API keys."""
        config = AuthConfig(
            jwt_secret="test_secret_32_chars_long!!",
            api_keys={
                "key123": {
                    "name": "test_key",
                    "roles": ["operator"],
                    "metadata": {"env": "test"}
                }
            }
        )
        return Authenticator(config)
    
    def test_verify_valid_api_key(self, authenticator_with_keys):
        """Can verify valid API key."""
        payload = authenticator_with_keys.verify_api_key("key123")
        
        assert payload is not None
        # API key uses "api_user" as subject
        assert payload["sub"] == "api_user"
        assert "operator" in payload["roles"]
        assert payload["metadata"]["env"] == "test"
    
    def test_verify_invalid_api_key(self, authenticator_with_keys):
        """Invalid API key returns None."""
        payload = authenticator_with_keys.verify_api_key("invalid_key")
        
        assert payload is None
    
    def test_verify_api_key_when_disabled(self, authenticator_with_keys):
        """When disabled, returns anonymous payload."""
        authenticator_with_keys.config.enabled = False
        
        payload = authenticator_with_keys.verify_api_key("any_key")
        
        assert payload is not None
        assert payload["sub"] == "anonymous"


class TestOriginValidation:
    """Test origin validation - SKIPPED (method not implemented)."""
    
    @pytest.mark.skip(reason="validate_origin method not implemented")
    def test_validate_allowed_origin(self):
        """Allowed origin is valid."""
        pass
    
    @pytest.mark.skip(reason="validate_origin method not implemented")
    def test_validate_disallowed_origin(self):
        """Disallowed origin is rejected."""
        pass
    
    @pytest.mark.skip(reason="validate_origin method not implemented")
    def test_validate_origin_with_wildcard(self):
        """Wildcard allows any origin."""
        pass


class TestTokenRefresh:
    """Test token refresh functionality."""
    
    @pytest.fixture
    def authenticator(self):
        """Create authenticator."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        return Authenticator(config)
    
    def test_refresh_valid_token(self, authenticator):
        """Can refresh valid token."""
        old_token = authenticator.create_token("user123", roles=["admin"])
        
        new_token = authenticator.refresh_token(old_token)
        
        assert new_token is not None
        assert new_token != old_token
        
        # Verify new token works
        payload = authenticator.verify_token(new_token)
        assert payload["sub"] == "user123"
        assert "admin" in payload["roles"]
    
    def test_refresh_invalid_token(self, authenticator):
        """Refreshing invalid token returns None."""
        new_token = authenticator.refresh_token("invalid_token")
        
        assert new_token is None


class TestSecurityFeatures:
    """Test security features."""
    
    def test_token_contains_expiry(self):
        """Token contains expiry timestamp."""
        config = AuthConfig(
            jwt_secret="test_secret_32_chars_long!!",
            jwt_expiry_hours=1
        )
        auth = Authenticator(config)
        
        token = auth.create_token("user123")
        payload = jwt.decode(token, "test_secret_32_chars_long!!", algorithms=["HS256"])
        
        assert "exp" in payload
        assert "iat" in payload
        
        # Check expiry is roughly 1 hour from now
        exp = datetime.fromtimestamp(payload["exp"], tz=timezone.utc)
        now = datetime.now(timezone.utc)
        diff = exp - now
        assert 3500 < diff.total_seconds() < 3700  # ~1 hour
    
    def test_token_contains_issued_at(self):
        """Token contains issued at timestamp."""
        config = AuthConfig(jwt_secret="test_secret_32_chars_long!!")
        auth = Authenticator(config)
        
        before = datetime.now(timezone.utc)
        token = auth.create_token("user123")
        after = datetime.now(timezone.utc)
        
        payload = jwt.decode(token, "test_secret_32_chars_long!!", algorithms=["HS256"])
        
        iat = datetime.fromtimestamp(payload["iat"], tz=timezone.utc)
        assert before <= iat <= after
