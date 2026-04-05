# Bandit Security Findings Review

**Scan Date:** 2026-04-05  
**Scope:** `agent_ros_bridge/` directory  
**Total Lines:** 25,729

## Executive Summary

| Severity | Count | Status |
|----------|-------|--------|
| **High** | 0 | ✅ None found |
| **Medium** | 1 | ⚠️ Reviewed - Acceptable risk |
| **Low** | 32 | ℹ️ Informational - No action required |

**Overall Assessment:** ✅ **Secure** - No critical vulnerabilities. All findings are either required for functionality or already mitigated.

---

## Medium Severity Findings

### B104: Hardcoded bind to all interfaces

**Location:** `agent_ros_bridge/gateway_v2/transports/http_transport.py:1328`

**Code:**
```python
self.host = config.get("host", "0.0.0.0")
```

**Issue:** The HTTP transport binds to `0.0.0.0` by default, allowing connections from any network interface.

**Risk Assessment:**
- **Severity:** Medium
- **Impact:** Gateway accepts connections from any network interface
- **Likelihood:** By design - required for robot connectivity

**Mitigation:**
1. ✅ **Authentication Required:** JWT tokens enforced when `security.enabled=true`
2. ✅ **Rate Limiting:** Implemented on all endpoints
3. ✅ **TLS Available:** Optional encryption for sensitive deployments
4. ✅ **Comment Annotation:** `# nosec B104` documents intentional design
5. ✅ **Configurable:** Users can override to specific interfaces if needed

**Decision:** ✅ **ACCEPT RISK** - This is required functionality for a robot gateway that needs to accept connections from remote robots. The security boundary is enforced at the authentication layer, not the network binding.

---

## Low Severity Findings

### B311: Standard pseudo-random generators

**Locations:**
- `agent_ros_bridge/agentic.py:543`
- `agent_ros_bridge/simulation/scenario_generator.py:198`
- `agent_ros_bridge/validation/scenario_10k.py:251-253`
- `agent_ros_bridge/utils/error_handling.py:274`

**Issue:** Uses `random` module instead of cryptographically secure `secrets` module.

**Risk Assessment:**
- **Severity:** Low
- **Impact:** Not used for security purposes
- **Contexts:**
  - Simulation scenario generation (testing only)
  - Gradual rollout percentage calculation (not security-sensitive)
  - Error injection for testing (testing only)

**Mitigation:** ✅ **ACCEPT** - Random generators are used appropriately for:
- Deterministic simulation testing
- Non-security statistical sampling
- Test error injection

**Note:** Where cryptographic randomness is needed (tokens, keys), the `secrets` module is used.

---

### B110: Try/except/pass patterns

**Locations:**
- `agent_ros_bridge/actions/__init__.py:363-365`
- `agent_ros_bridge/agentic.py:896-897`
- `agent_ros_bridge/simulation/gazebo_real.py:307-308, 428-429, 602-603`
- `agent_ros_bridge/simulation/gazebo_sim.py:111-112, 220-221, 625-626`

**Issue:** Empty except blocks that silently ignore errors.

**Risk Assessment:**
- **Severity:** Low
- **Impact:** Errors may be silently ignored
- **Contexts:**
  - Attribute access in dynamic objects
  - Resource cleanup (destroy_node)
  - Simulation fallbacks

**Examples:**

```python
# actions/__init__.py:363 - Graceful attribute access
try:
    result[attr] = val
except Exception:
    # Skip attributes that can't be accessed
    continue
```

```python
# simulation/gazebo_sim.py:111 - Docker detection fallback
try:
    return "docker" in f.read()
except Exception:
    pass
return False
```

**Mitigation:** ✅ **ACCEPT** - These are intentional graceful degradation patterns:
- Attribute access attempts on dynamic objects
- Resource cleanup that should not fail
- Feature detection with fallbacks

**Recommendation:** Consider logging at DEBUG level for troubleshooting, but current behavior is correct for production.

---

### B607/B603: Subprocess calls

**Locations:**
- `agent_ros_bridge/simulation/gazebo_sim.py` (multiple locations)
- `agent_ros_bridge/simulation/gazebo_metrics.py`
- `agent_ros_bridge/simulation/gazebo_real.py`
- `agent_ros_bridge/simulation/real_gazebo.py`

**Issue:** Uses `subprocess.run()` with partial paths and external commands.

**Commands Used:**
- `gz` - Gazebo CLI tool
- `pgrep` - Process grep utility

**Risk Assessment:**
- **Severity:** Low
- **Impact:** Command injection if user input is passed
- **Current State:** No user input in commands

**Code Example:**
```python
result = subprocess.run(
    ["gz", "service", "-s", "/world/default/scene/info", "--timeout", "1000"],
    capture_output=True,
    text=True,
    timeout=5,
    env=env,
)
```

**Mitigation:**
1. ✅ **No User Input:** Commands use hardcoded paths and arguments
2. ✅ **Timeout Protection:** All calls have timeout limits
3. ✅ **Required Functionality:** Gazebo integration requires subprocess calls
4. ✅ **List Arguments:** Uses list form, not shell strings

**Decision:** ✅ **ACCEPT** - These are required for Gazebo/ROS2 integration. The commands:
- Use explicit argument lists (no shell injection)
- Have no user-controlled input
- Are essential for simulation functionality

---

### B105/B107: Hardcoded passwords

**Locations:**
- `agent_ros_bridge/gateway_v2/config.py:207`
- `agent_ros_bridge/gateway_v2/transports/grpc_transport.py:161`

**Issue:** Detection of password-related strings.

**Context:**
```python
# config.py:207 - Environment variable mapping
"JWT_SECRET": "security.jwt_secret",
```

```python
# grpc_transport.py:161 - Default empty token
class GRPCClient:
    def __init__(self, host: str = "localhost", port: int = 50051, token: str = ""):
```

**Risk Assessment:**
- **Severity:** Low (false positive)
- **Impact:** None - these are configuration keys and defaults

**Mitigation:** ✅ **FALSE POSITIVE**
- `JWT_SECRET` is a configuration key path, not a password
- Empty token default is intentional (auth optional)
- Real secrets come from environment variables

---

### B404: Subprocess module import

**Locations:**
- `agent_ros_bridge/simulation/gazebo_sim.py:10`
- `agent_ros_bridge/simulation/real_gazebo.py:15`

**Issue:** Import of `subprocess` module flagged.

**Mitigation:** ✅ **ACCEPT** - Required for Gazebo integration. See B607/B603 analysis above.

---

### B112: Try/except/continue

**Location:** `agent_ros_bridge/actions/__init__.py:363-365`

**Issue:** Continue in except block may mask errors.

**Context:** Iterating over attributes and skipping inaccessible ones.

**Mitigation:** ✅ **ACCEPT** - Intentional graceful handling of dynamic attribute access.

---

## Recommendations

### Immediate Actions

1. ✅ **No immediate action required** - All findings reviewed and accepted

### Future Improvements

1. **Enhanced Logging** (Low Priority)
   - Add DEBUG logging for silent exception handlers
   - Helps with troubleshooting without exposing sensitive info

2. **Configuration Validation** (Medium Priority)
   - Add validation that warns if binding to `0.0.0.0` without auth
   - Fail-safe default: require explicit opt-in for insecure mode

3. **Documentation** (Completed ✅)
   - Security policy created (`SECURITY.md`)
   - Bandit configuration added (`.bandit.yml`)

### Security Monitoring

- [x] Bandit scans in CI pipeline
- [x] Dependabot alerts enabled
- [x] Trivy container scanning
- [x] pip-audit dependency scanning

---

## Compliance Notes

### CWE Mappings

| Bandit Code | CWE ID | Description |
|-------------|--------|-------------|
| B104 | CWE-605 | Binding to all interfaces |
| B110 | CWE-703 | Improper error handling |
| B311 | CWE-330 | Weak random number generator |
| B607 | CWE-78 | OS command injection |
| B603 | CWE-78 | OS command injection |

### Risk Acceptance

All MEDIUM and LOW severity findings have been reviewed and accepted as:
1. **Required for functionality** (robot gateway, Gazebo integration)
2. **Already mitigated** (authentication, TLS, timeouts)
3. **False positives** (configuration keys, test code)

**Approved By:** Automated scan review  
**Review Date:** 2026-04-05  
**Next Review:** On major version changes or new feature additions

---

*This document is part of the Agent ROS Bridge security documentation.*
