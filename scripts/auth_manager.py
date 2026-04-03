#!/usr/bin/env python3
"""Authentication system for Agent ROS Bridge.

Provides JWT-based authentication for dashboard access.
"""

import hashlib
import hmac
import json
import logging
import secrets
import time
from datetime import datetime, timedelta
from typing import Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("auth")


class AuthManager:
    """JWT-based authentication manager."""
    
    def __init__(self, secret_key: str | None = None):
        """
        Initialize auth manager.
        
        Args:
            secret_key: JWT secret key (auto-generated if None)
        """
        self.secret_key = secret_key or secrets.token_hex(32)
        self.users: dict[str, dict] = {}
        self.sessions: dict[str, dict] = {}  # token -> session data
        self.token_expiry = 3600  # 1 hour
        
        # Default admin user (change in production!)
        self.add_user("admin", "admin123", role="admin")
        
        logger.info("Auth manager initialized")
    
    def hash_password(self, password: str, salt: str | None = None) -> tuple[str, str]:
        """Hash password with salt."""
        if salt is None:
            salt = secrets.token_hex(16)
        
        key = hashlib.pbkdf2_hmac(
            'sha256',
            password.encode('utf-8'),
            salt.encode('utf-8'),
            100000  # iterations
        )
        return key.hex(), salt
    
    def add_user(self, username: str, password: str, role: str = "user"):
        """Add a new user."""
        if username in self.users:
            logger.warning(f"User {username} already exists")
            return False
        
        password_hash, salt = self.hash_password(password)
        
        self.users[username] = {
            'password_hash': password_hash,
            'salt': salt,
            'role': role,
            'created_at': datetime.now().isoformat(),
            'last_login': None
        }
        
        logger.info(f"User {username} added with role {role}")
        return True
    
    def verify_password(self, username: str, password: str) -> bool:
        """Verify user password."""
        if username not in self.users:
            return False
        
        user = self.users[username]
        password_hash, _ = self.hash_password(password, user['salt'])
        
        return hmac.compare_digest(password_hash, user['password_hash'])
    
    def create_token(self, username: str) -> str:
        """Create JWT-like token for user."""
        if username not in self.users:
            raise ValueError(f"User {username} not found")
        
        # Simple token (not full JWT for simplicity)
        token = secrets.token_urlsafe(32)
        
        self.sessions[token] = {
            'username': username,
            'role': self.users[username]['role'],
            'created_at': time.time(),
            'expires_at': time.time() + self.token_expiry
        }
        
        self.users[username]['last_login'] = datetime.now().isoformat()
        
        logger.info(f"Token created for {username}")
        return token
    
    def verify_token(self, token: str) -> Optional[dict]:
        """Verify token and return session data."""
        if token not in self.sessions:
            return None
        
        session = self.sessions[token]
        
        if time.time() > session['expires_at']:
            del self.sessions[token]
            logger.info("Token expired")
            return None
        
        return session
    
    def revoke_token(self, token: str):
        """Revoke token."""
        if token in self.sessions:
            del self.sessions[token]
            logger.info("Token revoked")
    
    def login(self, username: str, password: str) -> Optional[str]:
        """Authenticate user and return token."""
        if not self.verify_password(username, password):
            logger.warning(f"Failed login attempt for {username}")
            return None
        
        return self.create_token(username)
    
    def logout(self, token: str):
        """Logout user."""
        self.revoke_token(token)
    
    def check_permission(self, token: str, permission: str) -> bool:
        """Check if user has permission."""
        session = self.verify_token(token)
        if not session:
            return False
        
        role = session['role']
        
        # Simple RBAC
        permissions = {
            'user': ['view', 'subscribe'],
            'operator': ['view', 'subscribe', 'publish', 'control'],
            'admin': ['view', 'subscribe', 'publish', 'control', 'admin']
        }
        
        return permission in permissions.get(role, [])
    
    def get_user_info(self, token: str) -> Optional[dict]:
        """Get user info from token."""
        session = self.verify_token(token)
        if not session:
            return None
        
        username = session['username']
        return {
            'username': username,
            'role': session['role'],
            'last_login': self.users[username].get('last_login')
        }
    
    def list_users(self) -> list[dict]:
        """List all users (admin only)."""
        return [
            {
                'username': u,
                'role': data['role'],
                'last_login': data.get('last_login'),
                'created_at': data['created_at']
            }
            for u, data in self.users.items()
        ]
    
    def delete_user(self, username: str) -> bool:
        """Delete user."""
        if username not in self.users:
            return False
        
        # Revoke all tokens for this user
        tokens_to_revoke = [
            t for t, s in self.sessions.items()
            if s['username'] == username
        ]
        for t in tokens_to_revoke:
            self.revoke_token(t)
        
        del self.users[username]
        logger.info(f"User {username} deleted")
        return True


# Global auth instance
_auth_manager: AuthManager | None = None


def get_auth_manager() -> AuthManager:
    """Get or create global auth manager."""
    global _auth_manager
    if _auth_manager is None:
        _auth_manager = AuthManager()
    return _auth_manager


if __name__ == '__main__':
    # Test auth
    auth = AuthManager()
    
    # Test login
    token = auth.login("admin", "admin123")
    print(f"Login token: {token}")
    
    # Test verify
    session = auth.verify_token(token)
    print(f"Session: {session}")
    
    # Test permission
    can_control = auth.check_permission(token, "control")
    print(f"Can control: {can_control}")
    
    # Test logout
    auth.logout(token)
    print(f"After logout: {auth.verify_token(token)}")
