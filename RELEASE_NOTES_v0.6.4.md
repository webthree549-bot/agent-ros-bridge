# Release Notes - v0.6.4

**Release Date:** March 24, 2026  
**Version:** 0.6.4  
**Previous:** v0.6.3

---

## 🎉 Highlights

### Audit Response
This release addresses findings from the comprehensive project audit (2026-03-24):
- ✅ Fixed version mismatch (`__init__.py` now 0.6.4)
- ✅ Fixed Python 3.16 deprecation warnings
- ✅ Added dependency upper bounds for stability
- ✅ Documented Gazebo TODO roadmap
- ✅ Added fleet management tutorial

### Documentation
- **PROPOSAL_GREENFIELD_ADOPTION.md** - Technical proposal for new deployments
- **ROI_SUMMARY.md** - Executive ROI summary
- **GAZEBO_TODOS_ROADMAP.md** - Implementation guide for simulation features
- **TUTORIAL_FLEET_MANAGEMENT.md** - Complete fleet management tutorial
- **AUDIT_REPORT_2026-03-24.md** - Full project audit

---

## 🔧 Bug Fixes

### Critical
- **Version mismatch** - `__init__.py` version now matches `pyproject.toml` (0.6.4)

### High Priority
- **Python 3.16 compatibility** - Replaced deprecated `asyncio.iscoroutinefunction()` with `inspect.iscoroutinefunction()` in:
  - `gateway_v2/blueprint.py`
  - `gateway_v2/module.py`
- **Type annotation fix** - Changed `callable` to `Callable` in `discovery_hardened.py` (fixes CI on Python 3.11/3.12)

---

## 📦 Dependencies

### Upper Bounds Added
Added upper version bounds for improved stability:
- `pydantic>=2.0.0,<3.0.0`
- `pyyaml>=6.0,<7.0`
- `websockets>=11.0,<17.0`
- `cryptography>=41.0.0,<47.0.0`
- `aiohttp>=3.8.0,<4.0.0`
- `aiosqlite>=0.19.0,<1.0.0`
- `redis>=4.5.0,<6.0.0`
- `pyjwt>=2.8.0,<3.0.0`

---

## 📚 Documentation

### New Tutorials
- **Fleet Management** - Complete API tutorial with 10 patterns
  - Robot registration
  - Task assignment strategies
  - Sequential/parallel/cooperative patterns
  - Error handling & recovery

### New Planning Documents
- **Gazebo TODO Roadmap** - Priority matrix for 10 simulation TODOs
  - P0: Core navigation, collision detection (2-3 days)
  - P1: Transport integration, state queries (1-2 days)
  - P2: Physics validation, pose sampling (1 day)
  - P3: Visualization features (2-3 days)

### Business Documents
- **Greenfield Adoption Proposal** - 12-page technical proposal
- **ROI Summary** - $393K savings, 15 months faster

---

## 🧪 Test Status

- **Unit Tests:** 1,948 collected (99%+ passing)
- **E2E Tests:** 44 passed, 11 skipped (ROS-dependent)
- **Coverage:** ~63% (target: 60%)
- **Gate 2 Validation:** ✅ PASSED (95.93% success, 0 violations)

---

## 🚀 Upgrade Instructions

```bash
# Upgrade to v0.6.4
pip install --upgrade agent-ros-bridge==0.6.4

# Verify installation
python -c "import agent_ros_bridge; print(agent_ros_bridge.__version__)"
# Output: 0.6.4
```

### Breaking Changes
None - this is a backward-compatible maintenance release.

### Deprecations
None.

---

## 🙏 Acknowledgments

- Project audit conducted by OpenClaw Agent
- CI improvements based on GitHub Actions best practices
- Documentation improvements for new users

---

## 📊 Statistics

- **Files Changed:** 10+
- **Commits:** 4 since v0.6.3
- **New Documentation:** 6 files
- **Bug Fixes:** 3
- **Lines Added:** ~2,000

---

*Full changelog: See CHANGELOG.md*
