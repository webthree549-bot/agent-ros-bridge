# Security Policy

## Supported Versions

The following versions of OpenClaw ROS Bridge are currently supported with security updates:

| Version | Supported          |
| ------- | ------------------ |
| 2.x.x   | :white_check_mark: |
| 1.x.x   | :white_check_mark: |
| < 1.0   | :x:                |

## Reporting a Vulnerability

We take security seriously. If you discover a security vulnerability, please follow these steps:

### 1. Do Not Open a Public Issue

Please **DO NOT** open a public issue or pull request describing the vulnerability. This helps us coordinate a fix before the vulnerability is publicly disclosed.

### 2. Report Privately

Instead, please report the vulnerability via GitHub's private vulnerability reporting feature:

1. Go to the [Security tab](https://github.com/webthree549-bot/openclaw-ros-bridge/security) of the repository
2. Click "Report a vulnerability"
3. Fill out the vulnerability report form

Or email us directly at: **security@openclaw-ros.org**

### 3. What to Include

Please include the following information in your report:

- **Description**: A clear description of the vulnerability
- **Impact**: What could an attacker do with this vulnerability?
- **Steps to Reproduce**: Detailed steps to reproduce the issue
- **Affected Versions**: Which versions are affected?
- **Possible Mitigations**: Any workarounds or fixes you've identified
- **Your Contact**: How we can reach you for follow-up questions

### 4. Response Timeline

We will respond to security reports within **48 hours** and aim to provide:

- **Initial response**: Within 48 hours of receiving the report
- **Assessment**: Within 5 business days with an initial assessment
- **Fix timeline**: Estimated timeline for a fix
- **Resolution**: Notification when the fix is released

### 5. Disclosure Policy

We follow a **responsible disclosure** policy:

1. You report the vulnerability privately
2. We investigate and confirm the issue
3. We develop and test a fix
4. We release the fix in a new version
5. We publicly disclose the vulnerability (with credit to you, if desired)

We ask that you:
- Give us reasonable time to address the issue before public disclosure
- Do not exploit the vulnerability beyond what is necessary for verification
- Do not access, modify, or delete data belonging to others

## Security Best Practices

### For Users

1. **Keep Updated**: Always use the latest version of the software
2. **Network Security**: 
   - Use TLS/SSL for production deployments
   - Restrict network access to trusted sources
   - Use VPN or private networks for robot communication
3. **Authentication**:
   - Enable authentication in production
   - Use strong API keys/tokens
   - Rotate credentials regularly
4. **Docker Security**:
   - Run containers as non-root user
   - Use read-only filesystems where possible
   - Keep base images updated

### For Developers

1. **Dependency Management**:
   - Pin dependencies in production
   - Regularly audit dependencies (`pip-audit`, `safety`)
   - Automate dependency updates with Dependabot
2. **Code Review**:
   - Require code review for all changes
   - Use pre-commit hooks for security checks
   - Run security linters (Bandit, Semgrep)
3. **Secrets Management**:
   - Never commit secrets to version control
   - Use environment variables or secret management systems
   - Rotate secrets regularly
4. **Input Validation**:
   - Validate all user inputs
   - Sanitize data before processing
   - Use parameterized queries

## Security Features

### Implemented

- [x] TLS/SSL support for WebSocket and gRPC transports
- [x] API key authentication
- [x] JWT token authentication
- [x] Rate limiting
- [x] Input validation and sanitization
- [x] Audit logging
- [x] Security scanning in CI/CD

### Planned

- [ ] mTLS (mutual TLS) support
- [ ] Role-based access control (RBAC)
- [ ] OAuth2/OIDC integration
- [ ] Request signing (HMAC)
- [ ] Automatic security updates

## Acknowledgments

We thank the following security researchers who have responsibly disclosed vulnerabilities:

- None yet - be the first!

## Contact

- Security Team: security@openclaw-ros.org
- General Inquiries: dev@openclaw-ros.org

---

This security policy is adapted from the [GitHub Security Policy template](https://github.com/github/security-policy-templates).
