# MEMORY.md - Curated Long-Term Memory

_Last updated: 2026-03-15_

---

## Active Project: Agent ROS Bridge

**Repository:** https://github.com/webthree549-bot/agent-ros-bridge  
**PyPI:** https://pypi.org/project/agent-ros-bridge/  
**Current Version:** v0.6.1 (production ready)

### Status Summary
- **Tests:** 997 passing, 11 failing (E2E Docker-dependent)
- **Coverage:** 40.44% (target: 60%)
- **CI/CD:** 9-stage pipeline operational
- **ROS2:** Docker-based (`ros2_humble` container)

### Key Decisions
1. **All tests run in Docker** — No local ROS2 fallback
2. **No mock implementations** — Examples use real ROS
3. **Fail fast** — Clear errors instead of silent skips

### Performance Targets (Met)
- Intent parsing: ~0.01ms (target <10ms) ✅
- Safety validation: ~0.1ms cached (target <10ms) ✅
- Motion planning: ~70ms (target <100ms) ✅

### Next Priorities
- Reach 60% test coverage
- Install Nav2 in Docker for navigation E2E tests
- Fix remaining 11 E2E test failures

---

## Historical Milestones

| Date | Milestone | Tests | Version |
|------|-----------|-------|---------|
| 2026-03-12 | CI fixes, 997 tests, 40% coverage | 997 | v0.6.1 |
| 2026-03-09 | Docker-based ROS2 setup complete | 691 | v0.6.1 |
| 2026-03-04 | Week 1 TODO complete (gRPC, Redis, rate limiting) | - | v0.6.0 |
| 2026-03-03 | v0.5.1 released, 92/100 health score | 269 | v0.5.1 |
| 2026-03-02 | ROS2 connector complete, TDD compliance | 233 | v0.5.0+ |
| 2026-02-28 | Project audit, CI restored | 222 | v0.5.0 |

---

## Daily Logs Location

- **Recent:** `memory/YYYY-MM-DD.md` (root)
- **Archive:** `memory/daily/YYYY-MM-DD.md` (older)

---

## Identity

See `IDENTITY.md` and `USER.md` for assistant/user preferences.
