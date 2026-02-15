# Security Policy

## Security-First Defaults

Agent ROS Bridge controls physical robots and exposes network endpoints. **Security is not optional.**

## Required Security Configuration

### 1. JWT_SECRET (Required)

Before running the bridge, you **MUST** set a JWT secret:

```bash
# Generate a secure secret
export JWT_SECRET=$(openssl rand -base64 32)

# Or use Python
export JWT_SECRET=$(python3 -c "import secrets; print(secrets.token_urlsafe(32))")
```

**The bridge will FAIL to start without JWT_SECRET when authentication is enabled.**

### 2. Authentication Enabled by Default

Authentication is **enabled by default** in v0.2.1+. To connect:

1. Generate a token:
   ```bash
   python scripts/generate_token.py --secret $JWT_SECRET --role operator
   ```

2. Use token in WebSocket URL:
   ```
   ws://localhost:8765?token=YOUR_TOKEN_HERE
   ```

### 3. Role-Based Access Control

Default roles:
- `admin` — Full control
- `operator` — Control robots, view state
- `viewer` — Read-only access
- `anonymous` — No access (when auth enabled)

## Network Security

### Development/Testing
```bash
# Bind to localhost only (safe for development)
export BRIDGE_HOST=127.0.0.1
```

### Production
- Use TLS/WSS (WebSocket Secure)
- Place behind reverse proxy (nginx, traefik)
- Firewall: Restrict ports to known IPs
- VPN: Require VPN access for robot control

## Reporting Security Issues

Email: security@agent-ros-bridge.org

Please include:
- Version affected
- Description of vulnerability
- Steps to reproduce
- Suggested fix (if any)

## Security Checklist

Before deploying to production:

- [ ] JWT_SECRET set to strong random value
- [ ] Authentication enabled (default)
- [ ] TLS/WSS configured
- [ ] Firewall rules applied
- [ ] Rate limiting enabled
- [ ] Audit logging configured
- [ ] Access restricted to authorized users
- [ ] Regular security updates applied

## Known Limitations

- Mock mode disables auth (intentional for testing)
- Auto-generated secrets in v0.1.0-v0.2.0 (fixed in v0.2.1)
- Plain HTTP in default config (use TLS in production)

## Version History

| Version | Security Changes |
|---------|-----------------|
| v0.2.1+ | Auth enabled by default, JWT_SECRET required |
| v0.2.0 | Auth disabled by default (deprecated) |
| v0.1.0 | Auth disabled by default (deprecated) |
