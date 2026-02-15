# Security Policy

## Security-First Design

Agent ROS Bridge implements **mandatory JWT authentication** with **no bypass option**.

## Required Security Configuration

### 1. JWT_SECRET (Always Required)

The bridge **will fail to start** without a JWT secret. No exceptions.

```bash
# Generate a secure secret (REQUIRED)
export JWT_SECRET=$(openssl rand -base64 32)

# Start bridge
agent-ros-bridge
```

**There is no way to disable authentication.** This is by design.

### 2. Docker Examples (Isolated Testing)

All examples run in Docker containers for isolation:

```bash
# Example runs in container with JWT auth enforced
cd examples/quickstart
docker-compose up
```

The Docker containers:
- Run in isolated network namespaces
- Bind to container-internal interfaces
- Require JWT_SECRET to function
- Cannot disable authentication

### 3. Network Security

**Default Bind Address:** 127.0.0.1 (localhost only)

```bash
# Only localhost can connect (secure default)
export BRIDGE_HOST=127.0.0.1
agent-ros-bridge
```

**Production Deployment:**
- Use TLS/WSS (WebSocket Secure)
- Place behind reverse proxy (nginx, traefik)
- Firewall: Restrict to known IPs
- VPN: Require VPN access

## Authentication

### Token Generation

```bash
# Generate operator token
python scripts/generate_token.py --secret $JWT_SECRET --role operator

# Generate admin token
python scripts/generate_token.py --secret $JWT_SECRET --role admin
```

### Role-Based Access

| Role | Permissions |
|------|-------------|
| `admin` | Full control |
| `operator` | Control robots, view state |
| `viewer` | Read-only access |

## Reporting Security Issues

Email: security@agent-ros-bridge.org

## Security Checklist

Before production deployment:

- [ ] JWT_SECRET set to strong random value (32+ bytes)
- [ ] Authentication cannot be disabled (verified)
- [ ] TLS/WSS configured
- [ ] Firewall rules applied
- [ ] Rate limiting enabled
- [ ] Audit logging configured
- [ ] Access restricted to authorized users
- [ ] Regular security updates applied

## No Mock Mode

Unlike previous versions, **v0.3.0+ does not have a mock mode**. All deployments require authentication. Docker containers provide isolated testing environments without compromising security.

## Version History

| Version | Security Changes |
|---------|-----------------|
| v0.3.0+ | Mock mode removed, auth always required |
| v0.2.5 | Auth enabled by default, JWT_SECRET required |
| v0.2.0 | Auth disabled by default (deprecated) |
| v0.1.0 | Auth disabled by default (deprecated) |
