#!/usr/bin/env python3
"""Token generator CLI for Agent ROS Bridge.

Usage:
    python scripts/generate_token.py --user admin --roles admin,operator
    python scripts/generate_token.py --generate-secret
"""

import argparse
import os
import secrets
import sys

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from agent_ros_bridge.gateway_v2.auth import AuthConfig, Authenticator


def main():
    parser = argparse.ArgumentParser(
        description="Generate JWT tokens and secrets for Agent ROS Bridge"
    )
    parser.add_argument("--generate-secret", action="store_true", help="Generate a new JWT secret")
    parser.add_argument("--user", default="admin", help="User ID for token (default: admin)")
    parser.add_argument("--roles", default="admin", help="Comma-separated roles (default: admin)")
    parser.add_argument(
        "--secret",
        default=os.environ.get("JWT_SECRET"),
        help="JWT secret (or set JWT_SECRET env var)",
    )
    parser.add_argument(
        "--expiry", type=int, default=24, help="Token expiry in hours (default: 24)"
    )

    args = parser.parse_args()

    if args.generate_secret:
        secret = secrets.token_urlsafe(32)
        # SECURITY: Write full secret to stderr to avoid logging sensitive data
        sys.stderr.write("=" * 60 + "\n")
        sys.stderr.write("Generated JWT Secret\n")
        sys.stderr.write("=" * 60 + "\n")
        sys.stderr.write(secret + "\n")
        sys.stderr.write("=" * 60 + "\n")
        sys.stderr.write("\nAdd to your environment:\n")
        sys.stderr.write(f'  export JWT_SECRET="{secret}"\n')
        sys.stderr.write("\nOr add to config/bridge.yaml:\n")
        sys.stderr.write("  transports:\n")
        sys.stderr.write("    websocket:\n")
        sys.stderr.write("      auth:\n")
        sys.stderr.write("        enabled: true\n")
        sys.stderr.write(f'        jwt_secret: "{secret}"\n')
        print("\n[Secret generated - see stderr output above]")
        return

    if not args.secret:
        print("Error: JWT secret required")
        print("Use --secret or set JWT_SECRET environment variable")
        print("Or run with --generate-secret to create one")
        sys.exit(1)

    # Create authenticator
    config = AuthConfig(enabled=True, jwt_secret=args.secret, jwt_expiry_hours=args.expiry)
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
    # SECURITY: Write full token to stderr to avoid logging sensitive data
    # Only the masked token goes to stdout
    print(f"{token[:20]}... [see stderr for full token]")
    sys.stderr.write("=" * 60 + "\n")
    sys.stderr.write("FULL TOKEN (copy this):\n")
    sys.stderr.write(token + "\n")
    sys.stderr.write("=" * 60 + "\n")
    print("\nUse with WebSocket connection:")
    print('  wscat -c "ws://localhost:8766?token=<TOKEN_FROM_STDERR>"')
    print("\nOr in Python:")
    print(f'  headers = {{"Authorization": "Bearer {token[:20]}..."}}')


if __name__ == "__main__":
    main()
