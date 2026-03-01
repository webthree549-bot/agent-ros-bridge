#!/usr/bin/env python3
"""Authentication and authorization for Agent ROS Bridge.

Supports JWT tokens and API keys for securing WebSocket connections.
"""

import logging
import secrets
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

import jwt

logger = logging.getLogger("auth")


@dataclass
class AuthConfig:
    """Authentication configuration - SECURITY: Always enabled, JWT_SECRET required."""

    enabled: bool = True  # Always enabled, no disable option
    jwt_secret: Optional[str] = None
    jwt_algorithm: str = "HS256"
    jwt_expiry_hours: int = 24
    api_keys: Dict[str, Dict[str, Any]] = None
    allowed_origins: List[str] = None

    def __post_init__(self):
        if self.api_keys is None:
            self.api_keys = {}
        if self.allowed_origins is None:
            self.allowed_origins = ["127.0.0.1", "localhost"]  # SECURITY: Localhost only by default


class Authenticator:
    """JWT and API key authenticator."""

    def __init__(self, config: AuthConfig):
        self.config = config
        self._ensure_secret()

    def _ensure_secret(self):
        """Ensure JWT secret exists - SECURITY: Always required."""
        if not self.config.jwt_secret:
            raise ValueError(
                "JWT_SECRET is required. "
                "Set a strong secret via JWT_SECRET environment variable or config file. "
                "Example: export JWT_SECRET=$(openssl rand -base64 32)"
            )

    def create_token(
        self, user_id: str, roles: List[str] = None, metadata: Dict[str, Any] = None
    ) -> str:
        """Create a new JWT token."""
        if not self.config.enabled:
            raise RuntimeError("Authentication is disabled")

        payload = {
            "sub": user_id,
            "iat": datetime.utcnow(),
            "exp": datetime.utcnow() + timedelta(hours=self.config.jwt_expiry_hours),
            "roles": roles or ["user"],
            "metadata": metadata or {},
        }

        token = jwt.encode(payload, self.config.jwt_secret, algorithm=self.config.jwt_algorithm)
        return token

    def verify_token(self, token: str) -> Optional[Dict[str, Any]]:
        """Verify a JWT token and return payload."""
        if not self.config.enabled:
            return {"sub": "anonymous", "roles": ["admin"]}  # Allow all if disabled

        try:
            payload = jwt.decode(
                token, self.config.jwt_secret, algorithms=[self.config.jwt_algorithm]
            )
            return payload
        except jwt.ExpiredSignatureError:
            logger.warning("JWT token expired")
            return None
        except jwt.InvalidTokenError as e:
            logger.warning(f"Invalid JWT token: {e}")
            return None

    def verify_api_key(self, api_key: str) -> Optional[Dict[str, Any]]:
        """Verify an API key."""
        if not self.config.enabled:
            return {"sub": "anonymous", "roles": ["admin"]}

        if api_key in self.config.api_keys:
            key_data = self.config.api_keys[api_key]
            return {
                "sub": key_data.get("user_id", "api_user"),
                "roles": key_data.get("roles", ["user"]),
                "metadata": key_data.get("metadata", {}),
            }
        return None

    def extract_token_from_query(self, query_string: str) -> Optional[str]:
        """Extract JWT token from query string (token=xxx)."""
        import urllib.parse

        params = urllib.parse.parse_qs(query_string)
        if "token" in params:
            return params["token"][0]
        return None

    def extract_api_key_from_headers(self, headers: Dict[str, str]) -> Optional[str]:
        """Extract API key from headers (X-API-Key)."""
        return headers.get("X-API-Key") or headers.get("x-api-key")


class RoleBasedAccessControl:
    """Role-based access control for robot commands."""

    def __init__(self):
        # Define permission matrix
        self.permissions = {
            "admin": ["*"],  # All actions
            "operator": [
                "list_robots",
                "get_topics",
                "get_robot_state",
                "publish",
                "move",
                "rotate",
                "subscribe",
            ],
            "viewer": ["list_robots", "get_topics", "get_robot_state", "subscribe"],
            "anonymous": [
                "list_robots"  # Minimal info only
            ],
        }

    def can_execute(self, roles: List[str], action: str) -> bool:
        """Check if any role allows the action."""
        for role in roles:
            allowed = self.permissions.get(role, [])
            if "*" in allowed or action in allowed:
                return True
        return False

    def filter_response(self, roles: List[str], response: Dict[str, Any]) -> Dict[str, Any]:
        """Filter response based on role (e.g., hide sensitive data)."""
        if "admin" not in roles and "operator" not in roles:
            # Remove sensitive fields for viewers
            if "robots" in response:
                for robot in response["robots"]:
                    robot.pop("internal_ip", None)
                    robot.pop("serial_number", None)
        return response


# CLI for generating tokens
if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(description="Authentication CLI")
    parser.add_argument("--generate-secret", action="store_true", help="Generate JWT secret")
    parser.add_argument("--create-token", metavar="USER_ID", help="Create JWT token for user")
    parser.add_argument("--roles", default="user", help="Comma-separated roles")
    parser.add_argument("--secret", default=os.environ.get("JWT_SECRET"), help="JWT secret")

    args = parser.parse_args()

    if args.generate_secret:
        print(f"Generated JWT secret:\n{secrets.token_urlsafe(32)}")

    elif args.create_token:
        if not args.secret:
            print("Error: JWT secret required (use --secret or JWT_SECRET env var)")
            exit(1)

        config = AuthConfig(enabled=True, jwt_secret=args.secret)
        auth = Authenticator(config)
        roles = args.roles.split(",")
        token = auth.create_token(args.create_token, roles=roles)
        print(f"Token for {args.create_token}:\n{token}")
