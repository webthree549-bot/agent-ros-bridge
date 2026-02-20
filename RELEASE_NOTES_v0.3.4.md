# Release Notes — v0.3.4 (Security Patch)

**Release Date:** February 20, 2026  
**Priority:** HIGH — Security fix recommended for all users

---

## 🚨 Security Fix (CRITICAL)

**Fixed:** Clear-text logging of sensitive JWT credentials (CodeQL alert #83)

**Issue:** `scripts/generate_token.py` printed full JWT tokens and secrets to stdout, potentially exposing them in:
- Shell history
- Log files
- CI/CD pipeline logs
- Docker logs

**Fix:** Tokens and secrets now print to **stderr** instead of stdout:
- Stdout: Shows masked preview (`token[:20]}...`)
- Stderr: Full token for copying (separate stream, not logged)

**Impact:** Prevents accidental credential exposure in production environments.

**Action Required:** Users should upgrade immediately if they use token generation scripts in automated environments.

---

## Other Changes Since v0.3.3

### Code Quality
- Fixed WebSocket type hints (string annotations)
- GitHub repository optimization for growth

---

## Upgrade

```bash
pip install --upgrade agent-ros-bridge
```

---

## Verification

After upgrade, verify the fix:
```bash
python scripts/generate_token.py --generate-secret 2>/dev/null
# Should show: [Secret generated - see stderr output above]
# Full secret only visible without stderr redirection
```

---

**All users are encouraged to upgrade to v0.3.4 immediately.**
