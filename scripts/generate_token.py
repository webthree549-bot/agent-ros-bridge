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
        # SECURITY: Write secret to a file instead of stdout/stderr
        # This prevents accidental logging of sensitive data
        secret_file = ".jwt_secret"
        with open(secret_file, "w") as f:
            f.write(secret)
        os.chmod(secret_file, 0o600)  # Restrict permissions

        print("=" * 60)
        print("Generated JWT Secret")
        print("=" * 60)
        print(f"Secret saved to: {secret_file}")
        print("(File permissions set to 0o600)")
        print("\nTo use the secret:")
        print(f"  export JWT_SECRET=$(cat {secret_file})")
        print("\nOr add to config/bridge.yaml:")
        print("  transports:")
        print("    websocket:")
        print("      auth:")
        print("        enabled: true")
        print(f'        jwt_secret_file: "{secret_file}"')
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
    # SECURITY: Write token to a file instead of stdout/stderr
    # This prevents accidental logging of sensitive data
    token_file = ".jwt_token"
    with open(token_file, "w") as f:
        f.write(token)
    os.chmod(token_file, 0o600)  # Restrict permissions

    print(f"Token saved to: {token_file}")
    print("(File permissions set to 0o600)")
    print(f"\nMasked token: {token[:20]}...")
    print("\nUse with WebSocket connection:")
    print(f'  wscat -c "ws://localhost:8766?token=$(cat {token_file})"')
    print("\nOr in Python:")
    print(f'  with open("{token_file}") as f:')
    print("      token = f.read().strip()")
    print('  headers = {"Authorization": f"Bearer {token}"}')


if __name__ == "__main__":
    main()
