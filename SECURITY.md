# Security Policy

## Agent ROS Bridge Security

This document outlines the security practices and policies for the Agent ROS Bridge project.

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.6.x   | :white_check_mark: |
| 0.5.x   | :x:                |
| < 0.5   | :x:                |

## Security Practices

### Dependency Management

- **Automated Scanning**: Dependabot monitors for vulnerable dependencies
- **Update Policy**: Security patches applied within 7 days of release
- **Pinning**: Production dependencies are version-pinned in `pyproject.toml`

### Code Security

- **Static Analysis**: Bandit runs on all code changes
- **Secrets Scanning**: GitHub secret scanning enabled
- **Code Review**: All changes require PR review

### Network Security

- **Binding**: Gateway binds to `0.0.0.0` by design (required for remote robot access)
- **Authentication**: JWT-based auth available for all transports
- **TLS**: Optional TLS support for encrypted connections

### Safety-First Design

- **Human-in-the-Loop**: All AI proposals require human approval by default
- **Emergency Stop**: <100ms response time for safety-critical stops
- **Shadow Mode**: Collects data without executing actions during validation

## Reporting Security Issues

If you discover a security vulnerability:

1. **DO NOT** open a public issue
2. Email security concerns to: [project maintainer]
3. Include:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if any)

We will:
- Acknowledge receipt within 48 hours
- Investigate and provide updates within 7 days
- Release patches for confirmed vulnerabilities within 30 days

## Known Security Considerations

### Subprocess Usage (B607/B603)

**Location**: `agent_ros_bridge/simulation/gazebo_*.py`

The simulation modules use subprocess calls to interact with Gazebo/ROS2:
- `gz` CLI for Gazebo Harmonic integration
- `pgrep` for process detection

**Mitigation**:
- All commands use explicit paths and timeout limits
- No user input is passed to subprocess calls
- Required for ROS2/Gazebo integration functionality

### Network Binding (B104)

**Location**: Multiple transport files

The gateway binds to `0.0.0.0` to accept connections from remote robots.

**Mitigation**:
- JWT authentication enforced when `security.enabled=true`
- TLS encryption available
- Rate limiting on all endpoints
- `# nosec` annotation documents intentional design

### Random Number Generation (B311)

**Location**: `scenario_*.py`, `agentic.py`, `error_handling.py`

Standard `random` module used for:
- Scenario generation (simulation testing)
- Gradual rollout percentages (safety feature)
- Error injection testing (test code only)

**Mitigation**:
- NOT used for cryptographic purposes
- `secrets` module used where crypto-grade randomness required
- Documented as non-security use cases

## Security Scanning

### Bandit

```bash
bandit -r agent_ros_bridge/ -f screen
```

Current status: **No HIGH severity issues**

### Dependencies

```bash
pip-audit
```

Monitored via GitHub Dependabot with automatic PRs for security updates.

## Compliance

### Standards

- **CWE**: Issues tracked against CWE taxonomy
- **NIST**: Aligned with cybersecurity framework
- **ROS2 Security**: Compatible with ROS2 security features (SROS2)

### HIPAA Considerations (Healthcare Use Case)

For healthcare deployments:
- All PHI (Protected Health Information) must be encrypted at rest and in transit
- Audit logs are immutable and tamper-evident
- Access controls via role-based authentication
- Data retention: 7 years minimum

## Security Checklist for Releases

- [ ] All HIGH/MEDIUM Bandit issues reviewed
- [ ] Dependencies updated to latest secure versions
- [ ] No hardcoded secrets in code
- [ ] Authentication tested on all transports
- [ ] Emergency stop functionality verified
- [ ] Security documentation updated

---

*Last updated: 2026-04-05*
