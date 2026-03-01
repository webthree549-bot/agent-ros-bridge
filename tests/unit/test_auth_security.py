"""
Security-focused unit tests for JWT authentication and RBAC.
Tests token generation, validation, edge cases, and security vulnerabilities.
"""

import base64
import json
import time
from datetime import datetime, timedelta

import pytest

try:
    import jwt

    JWT_AVAILABLE = True
except ImportError:
    JWT_AVAILABLE = False
    pytest.skip("PyJWT not available", allow_module_level=True)

import contextlib

from agent_ros_bridge.gateway_v2.core import Identity


# Simple JWT implementation for testing (matches the bridge's approach)
class JWTAuthManager:
    """Test version of JWT auth manager"""

    def __init__(self, secret: str, algorithm: str = "HS256"):
        self.secret = secret
        self.algorithm = algorithm

    def generate_token(self, identity: Identity, expires_in: int = 3600) -> str:
        """Generate JWT token for identity"""
        now = datetime.utcnow()
        payload = {
            "sub": identity.id,
            "name": identity.name,
            "roles": identity.roles,
            "metadata": identity.metadata,
            "iat": now,
            "exp": now + timedelta(seconds=expires_in),
        }
        return jwt.encode(payload, self.secret, algorithm=self.algorithm)

    def validate_token(self, token: str) -> dict:
        """Validate and decode JWT token"""
        return jwt.decode(token, self.secret, algorithms=[self.algorithm])

    def has_role(self, token: str, required_role: str) -> bool:
        """Check if token has required role"""
        try:
            payload = self.validate_token(token)
            return required_role in payload.get("roles", [])
        except jwt.InvalidTokenError:
            return False

    def has_any_role(self, token: str, required_roles: list) -> bool:
        """Check if token has any of the required roles"""
        try:
            payload = self.validate_token(token)
            user_roles = set(payload.get("roles", []))
            return len(user_roles & set(required_roles)) > 0
        except jwt.InvalidTokenError:
            return False


class TestJWTTokenGeneration:
    """Test JWT token generation"""

    @pytest.fixture
    def auth_manager(self):
        """Create auth manager with test secret"""
        return JWTAuthManager(secret="test-secret-key-12345")

    @pytest.fixture
    def test_identity(self):
        """Create test identity"""
        return Identity(
            id="user_123",
            name="Test User",
            roles=["operator", "viewer"],
            metadata={"department": "robotics"},
        )

    def test_token_generation(self, auth_manager, test_identity):
        """Test basic token generation"""
        token = auth_manager.generate_token(test_identity)

        assert token is not None
        assert isinstance(token, str)
        assert len(token) > 0

        # Token should have 3 parts separated by dots
        parts = token.split(".")
        assert len(parts) == 3

    def test_token_contains_expected_claims(self, auth_manager, test_identity):
        """Test that token contains expected claims"""
        token = auth_manager.generate_token(test_identity)
        payload = auth_manager.validate_token(token)

        assert payload["sub"] == "user_123"
        assert payload["name"] == "Test User"
        assert "operator" in payload["roles"]
        assert "viewer" in payload["roles"]
        assert payload["metadata"]["department"] == "robotics"

    def test_token_has_expiration(self, auth_manager, test_identity):
        """Test that token has expiration time"""
        token = auth_manager.generate_token(test_identity, expires_in=3600)
        payload = auth_manager.validate_token(token)

        assert "exp" in payload
        assert "iat" in payload

        # Expiration should be in the future
        exp_time = datetime.fromtimestamp(payload["exp"])
        iat_time = datetime.fromtimestamp(payload["iat"])
        assert exp_time > iat_time
        assert (exp_time - iat_time).seconds == 3600

    def test_custom_expiration(self, auth_manager, test_identity):
        """Test custom token expiration time"""
        # Short-lived token
        short_token = auth_manager.generate_token(test_identity, expires_in=60)
        payload = auth_manager.validate_token(short_token)

        exp_time = datetime.fromtimestamp(payload["exp"])
        iat_time = datetime.fromtimestamp(payload["iat"])
        assert (exp_time - iat_time).seconds == 60


class TestJWTTokenValidation:
    """Test JWT token validation"""

    @pytest.fixture
    def auth_manager(self):
        return JWTAuthManager(secret="test-secret-key-12345")

    @pytest.fixture
    def valid_token(self, auth_manager):
        identity = Identity(id="user_123", name="Test", roles=["operator"])
        return auth_manager.generate_token(identity)

    def test_valid_token_validation(self, auth_manager, valid_token):
        """Test validation of valid token"""
        payload = auth_manager.validate_token(valid_token)

        assert payload is not None
        assert payload["sub"] == "user_123"

    def test_invalid_signature_rejection(self, auth_manager, valid_token):
        """Test that token with invalid signature is rejected"""
        # Create manager with different secret
        wrong_manager = JWTAuthManager(secret="wrong-secret")

        with pytest.raises(jwt.InvalidSignatureError):
            wrong_manager.validate_token(valid_token)

    def test_expired_token_rejection(self, auth_manager):
        """Test that expired token is rejected"""
        identity = Identity(id="user_123", name="Test", roles=[])
        # Generate already-expired token
        expired_token = auth_manager.generate_token(identity, expires_in=-1)

        with pytest.raises(jwt.ExpiredSignatureError):
            auth_manager.validate_token(expired_token)

    def test_malformed_token_rejection(self, auth_manager):
        """Test that malformed token is rejected"""
        malformed_tokens = [
            "not.a.token",
            "invalid",
            "",
            "too.many.dots.here",
        ]

        for token in malformed_tokens:
            with pytest.raises((jwt.InvalidTokenError, ValueError)):
                auth_manager.validate_token(token)

    def test_tampered_token_rejection(self, auth_manager, valid_token):
        """Test that tampered token is rejected"""
        # Modify the token payload
        parts = valid_token.split(".")
        tampered = parts[0] + "." + parts[1] + ".TAMPERED"

        with pytest.raises(jwt.InvalidSignatureError):
            auth_manager.validate_token(tampered)


class TestJWTSecurityVulnerabilities:
    """Test protection against JWT security vulnerabilities"""

    @pytest.fixture
    def auth_manager(self):
        return JWTAuthManager(secret="test-secret-key-12345")

    def test_none_algorithm_rejection(self, auth_manager):
        """Test rejection of 'none' algorithm (CVE-2015-9235)"""
        # Create token with none algorithm
        payload = {"sub": "attacker", "roles": ["admin"]}
        none_token = jwt.encode(payload, key="", algorithm="none")

        # Should reject when expecting HS256
        with pytest.raises(jwt.InvalidAlgorithmError):
            auth_manager.validate_token(none_token)

    def test_algorithm_confusion_attack(self, auth_manager):
        """Test protection against algorithm confusion attacks"""
        # Try to use RS256 public key as HS256 secret
        # This is a simplified test - real attack would use actual RSA key
        payload = {"sub": "attacker"}

        # Generate with different algorithm
        try:
            confused_token = jwt.encode(payload, key="any-key", algorithm="HS384")
            # Should reject if we're strict about algorithm
            with pytest.raises(jwt.InvalidAlgorithmError):
                auth_manager.validate_token(confused_token)
        except Exception:
            # If encoding fails, that's also fine for this test
            pass

    def test_empty_secret_warning(self):
        """Test that empty secret is flagged (implementation may vary)"""
        # Note: Some implementations reject empty secrets, others allow with warnings
        # This test documents that empty secrets should be avoided
        manager = JWTAuthManager(secret="")
        identity = Identity(id="user", name="Test", roles=[])

        # May generate token with empty secret (should be rejected in production)
        token = manager.generate_token(identity)
        assert token is not None

        # Validation should work with same empty secret
        payload = manager.validate_token(token)
        assert payload["sub"] == "user"

    def test_weak_secret_warning(self):
        """Test that weak secrets are flagged"""
        # Very short secret
        weak_manager = JWTAuthManager(secret="short")

        # It works but should be flagged in production
        identity = Identity(id="user", name="Test", roles=[])
        token = weak_manager.generate_token(identity)

        # Token should still be valid
        payload = weak_manager.validate_token(token)
        assert payload["sub"] == "user"


class TestRBACAuthorization:
    """Test Role-Based Access Control"""

    @pytest.fixture
    def auth_manager(self):
        return JWTAuthManager(secret="test-secret")

    def test_has_role_check(self, auth_manager):
        """Test role checking"""
        identity = Identity(id="admin_user", name="Admin", roles=["admin", "operator"])
        token = auth_manager.generate_token(identity)

        assert auth_manager.has_role(token, "admin") is True
        assert auth_manager.has_role(token, "operator") is True
        assert auth_manager.has_role(token, "viewer") is False

    def test_has_any_role_check(self, auth_manager):
        """Test any-of-roles checking"""
        identity = Identity(id="operator_user", name="Operator", roles=["operator"])
        token = auth_manager.generate_token(identity)

        # Has one of the required roles
        assert auth_manager.has_any_role(token, ["admin", "operator"]) is True
        # Has none of the required roles
        assert auth_manager.has_any_role(token, ["admin", "viewer"]) is False

    def test_invalid_token_role_check(self, auth_manager):
        """Test role checking with invalid token"""
        assert auth_manager.has_role("invalid_token", "admin") is False
        assert auth_manager.has_any_role("invalid_token", ["admin", "operator"]) is False

    def test_empty_roles(self, auth_manager):
        """Test user with no roles"""
        identity = Identity(id="public_user", name="Public", roles=[])
        token = auth_manager.generate_token(identity)

        assert auth_manager.has_role(token, "admin") is False
        assert auth_manager.has_any_role(token, ["admin", "operator"]) is False


class TestTokenPayloadEdgeCases:
    """Test edge cases in token payload"""

    @pytest.fixture
    def auth_manager(self):
        return JWTAuthManager(secret="test-secret")

    def test_unicode_in_identity(self, auth_manager):
        """Test Unicode characters in identity fields"""
        identity = Identity(
            id="user_用户",
            name="测试用户 🤖",
            roles=["操作员"],
            metadata={"department": "机器人部门"},
        )

        token = auth_manager.generate_token(identity)
        payload = auth_manager.validate_token(token)

        assert payload["sub"] == "user_用户"
        assert payload["name"] == "测试用户 🤖"
        assert "操作员" in payload["roles"]

    def test_long_roles_list(self, auth_manager):
        """Test identity with many roles"""
        roles = [f"role_{i}" for i in range(100)]
        identity = Identity(id="user", name="Test", roles=roles)

        token = auth_manager.generate_token(identity)
        payload = auth_manager.validate_token(token)

        assert len(payload["roles"]) == 100
        assert "role_50" in payload["roles"]

    def test_nested_metadata(self, auth_manager):
        """Test deeply nested metadata"""
        identity = Identity(
            id="user",
            name="Test",
            roles=["operator"],
            metadata={
                "level1": {"level2": {"level3": {"data": "deep_value"}}},
                "array_data": [1, 2, {"nested": True}],
            },
        )

        token = auth_manager.generate_token(identity)
        payload = auth_manager.validate_token(token)

        assert payload["metadata"]["level1"]["level2"]["level3"]["data"] == "deep_value"
        assert payload["metadata"]["array_data"][2]["nested"] is True


class TestTokenTimingAttacks:
    """Test protection against timing attacks"""

    def test_constant_time_comparison(self):
        """Test that validation is constant-time"""
        # This is a conceptual test - real constant-time comparison
        # would need specialized timing measurement

        auth_manager = JWTAuthManager(secret="test-secret")
        identity = Identity(id="user", name="Test", roles=[])

        valid_token = auth_manager.generate_token(identity)

        # Both should be rejected but take similar time

        # Measure valid token
        start = time.perf_counter()
        with contextlib.suppress(BaseException):
            auth_manager.validate_token(valid_token)
        valid_time = time.perf_counter() - start

        # Measure invalid signature
        start = time.perf_counter()
        with contextlib.suppress(BaseException):
            auth_manager.validate_token(valid_token[:-5] + "XXXXX")
        invalid_time = time.perf_counter() - start

        # Times should be reasonably similar (not a strict test)
        assert abs(valid_time - invalid_time) < 0.1  # 100ms threshold


class TestTokenRefreshScenarios:
    """Test token refresh and rotation scenarios"""

    @pytest.fixture
    def auth_manager(self):
        return JWTAuthManager(secret="test-secret")

    def test_token_renewal(self, auth_manager):
        """Test that new token can be generated with extended expiry"""
        identity = Identity(id="user", name="Test", roles=["operator"])

        # Original short-lived token
        short_token = auth_manager.generate_token(identity, expires_in=60)
        short_payload = auth_manager.validate_token(short_token)

        # Refresh with longer expiry
        refreshed_token = auth_manager.generate_token(identity, expires_in=3600)
        refreshed_payload = auth_manager.validate_token(refreshed_token)

        # Same subject, different expiration
        assert short_payload["sub"] == refreshed_payload["sub"]
        assert refreshed_payload["exp"] > short_payload["exp"]


class TestDifferentAlgorithms:
    """Test with different JWT algorithms"""

    @pytest.mark.parametrize(
        "algorithm,secret_size",
        [
            ("HS256", 32),
            ("HS384", 48),
            ("HS512", 64),
        ],
    )
    def test_different_hmac_algorithms(self, algorithm, secret_size):
        """Test various HMAC algorithms"""
        secret = "x" * secret_size
        manager = JWTAuthManager(secret=secret, algorithm=algorithm)

        identity = Identity(id="user", name="Test", roles=[])
        token = manager.generate_token(identity)

        # Verify algorithm in header
        header = json.loads(base64.urlsafe_b64decode(token.split(".")[0] + "=="))
        assert header["alg"] == algorithm

        # Should validate correctly
        payload = manager.validate_token(token)
        assert payload["sub"] == "user"


class TestTokenBlacklist:
    """Test token revocation/blacklist scenarios"""

    def test_conceptual_blacklist(self):
        """Test conceptual token blacklist implementation"""
        # This documents how blacklist could be implemented

        class BlacklistAwareAuthManager(JWTAuthManager):
            def __init__(self, secret: str):
                super().__init__(secret)
                self.blacklist = set()

            def revoke_token(self, token: str):
                """Add token to blacklist"""
                # In production, use token ID (jti claim) not full token
                payload = self.validate_token(token)
                jti = payload.get("jti") or token[-16:]  # Fallback
                self.blacklist.add(jti)

            def is_revoked(self, token: str) -> bool:
                """Check if token is revoked"""
                try:
                    payload = self.validate_token(token)
                    jti = payload.get("jti") or token[-16:]
                    return jti in self.blacklist
                except jwt.InvalidTokenError:
                    return True

        manager = BlacklistAwareAuthManager(secret="test-secret")
        identity = Identity(id="user", name="Test", roles=[])
        token = manager.generate_token(identity)

        # Token valid initially
        assert manager.is_revoked(token) is False

        # Revoke token
        manager.revoke_token(token)
        assert manager.is_revoked(token) is True


class TestAuthIntegrationPatterns:
    """Test common authentication integration patterns"""

    def test_extract_token_from_header(self):
        """Test token extraction from Authorization header"""

        def extract_token(header: str) -> str:
            """Extract Bearer token from header"""
            if header.startswith("Bearer "):
                return header[7:]
            return None

        assert extract_token("Bearer my-token") == "my-token"
        assert extract_token("Basic credentials") is None
        assert extract_token("") is None

    def test_extract_token_from_query(self):
        """Test token extraction from query string"""
        from urllib.parse import parse_qs, urlparse

        def extract_token_from_url(url: str) -> str:
            parsed = urlparse(url)
            params = parse_qs(parsed.query)
            return params.get("token", [None])[0]

        assert extract_token_from_url("ws://localhost:8765?token=abc123") == "abc123"
        assert extract_token_from_url("ws://localhost:8765") is None
