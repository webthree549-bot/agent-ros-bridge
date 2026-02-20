#!/usr/bin/env python3
"""Token generator CLI for Agent ROS Bridge

Usage:
    python scripts/generate_token.py --user admin --roles admin,operator
    python scripts/generate_token.py --generate-secret
"""

import argparse
import secrets
import os
import sys

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge.gateway_v2.auth import Authenticator, AuthConfig


def main():
    parser = argparse.ArgumentParser(
        description="Generate JWT tokens and secrets for Agent ROS Bridge"
    )
    parser.add_argument(
        "--generate-secret",
        action="store_true",
        help="Generate a new JWT secret"
    )
    parser.add_argument(
        "--user",
        default="admin",
        help="User ID for token (default: admin)"
    )
    parser.add_argument(
        "--roles",
        default="admin",
        help="Comma-separated roles (default: admin)"
    )
    parser.add_argument(
        "--secret",
        default=os.environ.get("JWT_SECRET"),
        help="JWT secret (or set JWT_SECRET env var)"
    )
    parser.add_argument(
        "--expiry",
        type=int,
        default=24,
        help="Token expiry in hours (default: 24)"
    )
    
    args = parser.parse_args()
    
    if args.generate_secret:
        secret = secrets.token_urlsafe(32)
        # SECURITY: Print full secret to stderr to avoid logging sensitive data
        print("=" * 60, file=sys.stderr)
        print("Generated JWT Secret", file=sys.stderr)
        print("=" * 60, file=sys.stderr)
        print(secret, file=sys.stderr)
        print("=" * 60, file=sys.stderr)
        print("\nAdd to your environment:", file=sys.stderr)
        print(f'  export JWT_SECRET="{secret}"', file=sys.stderr)
        print("\nOr add to config/bridge.yaml:", file=sys.stderr)
        print("  transports:", file=sys.stderr)
        print("    websocket:", file=sys.stderr)
        print("      auth:", file=sys.stderr)
        print("        enabled: true", file=sys.stderr)
        print(f"        jwt_secret: \"{secret}\"", file=sys.stderr)
        print("\n[Secret generated - see stderr output above]")
        return
    
    if not args.secret:
        print("Error: JWT secret required")
        print("Use --secret or set JWT_SECRET environment variable")
        print("Or run with --generate-secret to create one")
        sys.exit(1)
    
    # Create authenticator
    config = AuthConfig(
        enabled=True,
        jwt_secret=args.secret,
        jwt_expiry_hours=args.expiry
    )
    auth = Authenticator(config)
    
    # Generate token
    roles = args.roles.split(",")
    token = auth.create_token(args.user, roles=roles)
    
    print("=" * 60)
    print("JWT Token Generated")
    print("=" * 60)
    print(f"User: {args.user}")
    print(f"Roles: {', '.join(roles)}")
    print(f"Expires: {args.expiry} hours")
    print("=" * 60)
    # SECURITY: Print full token to stderr to avoid logging sensitive data
    # Only the masked token goes to stdout
    print(f"{token[:20]}... [see stderr for full token]")
    print("=" * 60, file=sys.stderr)
    print("FULL TOKEN (copy this):", file=sys.stderr)
    print(token, file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print("\nUse with WebSocket connection:")
    print(f'  wscat -c "ws://localhost:8766?token=<TOKEN_FROM_STDERR>"')
    print("\nOr in Python:")
    print(f'  headers = {{"Authorization": "Bearer {token[:20]}..."}}')


if __name__ == "__main__":
    main()
