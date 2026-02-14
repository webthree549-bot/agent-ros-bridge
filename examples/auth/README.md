# Authentication Example

**JWT token-based security demo.**

## What It Does

Runs the bridge with JWT authentication enabled. Shows how to secure robot access.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh
```

## Test

```bash
# Generate a token (in another terminal)
python ../../scripts/generate_token.py --secret my-secret-key --role operator

# Connect with token
wscat -c "ws://localhost:8768?token=YOUR_TOKEN_HERE"

# Without token (will fail)
wscat -c ws://localhost:8768
```

## What's Happening

This demonstrates:
- **JWT Authentication**: Tokens required for connection
- **Role-Based Access**: Admin, operator, viewer roles
- **Token Generation**: How to create valid tokens

## Roles

| Role | Permissions |
|------|-------------|
| `admin` | Full control |
| `operator` | Control robots |
| `viewer` | Read-only |

## Next Steps

- Read [User Manual - Security](../../docs/USER_MANUAL.md#security-hardening)
- Integrate with your auth provider
