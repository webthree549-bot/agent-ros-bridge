# ADR-0002: JWT Authentication for All Transports

## Status

**Accepted**

## Context

Agent ROS Bridge controls physical robots that can cause real-world damage if accessed by unauthorized users. Security requirements included:

1. **Authentication** — Verify identity of connecting agents
2. **Authorization** — Control what actions each agent can perform
3. **Transport Security** — Protect data in transit
4. **Auditability** — Log who did what for accountability

Initially, authentication was optional (`auth_enabled=False` by default), which contradicted security best practices and project documentation claiming "JWT always required."

## Decision

We will implement **mandatory JWT authentication** with the following design:

### Core Principles

1. **Always On** — JWT authentication cannot be disabled in production
2. **Per-Transport** — Each transport (WebSocket, MQTT, gRPC) implements auth
3. **RBAC** — Role-based access control for fine-grained permissions
4. **Token-Based** — Stateless JWT tokens with configurable expiration
5. **Secret Required** — Bridge refuses to start without `JWT_SECRET` set

### Implementation

```python
# Auth configuration
auth_config = {
    "enabled": True,  # Always True in production
    "jwt_secret": os.environ["JWT_SECRET"],  # Required
    "jwt_expiry_hours": 24,
    "algorithm": "HS256",
}

# Token payload
{
    "sub": "user_id",
    "roles": ["operator", "admin"],
    "iat": 1700000000,
    "exp": 1700086400,
    "metadata": {}
}
```

### Transport-Specific Implementation

| Transport | Auth Method | Location |
|-----------|-------------|----------|
| WebSocket | Query parameter | `?token=<jwt>` |
| MQTT | Username/Password | Username: token |
| gRPC | Metadata | `authorization: Bearer <jwt>` |

### RBAC Roles

- `anonymous` — Unauthenticated (limited read-only)
- `viewer` — Can view robot state
- `operator` — Can send non-dangerous commands
- `admin` — Full access including emergency stop

## Consequences

### Positive

- **Security** — Strong authentication across all entry points
- **Consistency** — Same auth model regardless of transport
- **Auditability** — All actions tied to authenticated identity
- **Flexibility** — RBAC allows fine-grained access control
- **Standards-Based** — JWT is widely adopted and understood

### Negative

- **Complexity** — Clients must implement token management
- **Token Expiration** — Requires token refresh mechanism
- **Secret Management** — Operators must securely manage JWT_SECRET
- **Debugging** — Harder to debug with auth enabled (can't just `curl`)

### Neutral

- **Performance** — JWT verification adds ~1-2ms per request
- **Statelessness** — No server-side session storage needed

## Alternatives Considered

### Optional Authentication

**Rejected:** Would allow insecure deployments. Security should not be opt-in for robot control systems.

### API Keys Only

**Rejected:** API keys don't support expiration or fine-grained permissions. JWTs provide more flexibility.

### mTLS (Mutual TLS) Only

**Rejected:** While we support mTLS for transport security, it's not practical for client authentication in all scenarios (e.g., browser-based dashboards).

### OAuth 2.0 / OIDC

**Rejected:** Too complex for initial implementation. JWT design allows migration to OAuth later if needed.

## Implementation History

- **v0.4.0** — JWT auth added as optional feature
- **v0.5.0** — Made mandatory, RBAC implemented
- **v0.5.1** — gRPC JWT validation completed

## References

- [Security Documentation](../SECURITY.md) (if exists)
- [Auth Implementation](../../agent_ros_bridge/gateway_v2/auth.py)
- Related: [ADR-0001: Gateway V2 Architecture](0001-gateway-v2-architecture.md)
