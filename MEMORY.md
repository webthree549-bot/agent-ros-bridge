# MEMORY.md - Curated Long-Term Memory

_Last updated: 2026-03-16_

---

## Active Project: Agent ROS Bridge

**Repository:** https://github.com/webthree549-bot/agent-ros-bridge  
**PyPI:** https://pypi.org/project/agent-ros-bridge/  
**Current Version:** v0.6.1 (production ready)

### Status Summary
- **Tests:** 997 unit tests passing, 43 E2E tests passing
- **E2E Tests:** 43 passed, 12 skipped (OpenClaw integration tests)
- **Coverage:** 40.44% (target: 60%)
- **CI/CD:** 9-stage pipeline operational
- **ROS2:** Docker-based (`ros2_humble` container with Jazzy + Nav2)

### Recent Fixes (2026-03-16)
1. **WebSocketTransport API** - Fixed breaking change in `__init__` signature
2. **Docker Strategy** - E2E tests skip gracefully in CI, run locally
3. **ROS Version Detection** - Auto-detects Humble/Jazzy in container
4. **Demo Nodes** - Added to Docker image for E2E tests
5. **Dependency Workflow** - Fixed invalid GitHub Action reference

### Docker Strategy
- **Image:** `agent-ros-bridge:ros2-humble` (ROS2 Jazzy + demo nodes + Nav2, 5GB)
- **E2E Tests:** Run locally with `./scripts/test-e2e.sh`
- **CI Behavior:** E2E tests skip if container not running

### Key Decisions
1. **Docker-first testing** — No local ROS2 fallback
2. **Graceful degradation** — Tests skip when dependencies unavailable
3. **Fixed base image** — Reproducible builds with version-pinned packages

### Performance Targets (Met)
- Intent parsing: ~0.01ms (target <10ms) ✅
- Safety validation: ~0.1ms cached (target <10ms) ✅
- Motion planning: ~70ms (target <100ms) ✅

### Next Priorities
- Reach 60% test coverage
- Optimize CI pipeline speed
- Fix remaining 12 skipped E2E tests (OpenClaw integration)

---

## Historical Milestones

| Date | Milestone | Tests | Version |
|------|-----------|-------|---------|
| 2026-03-16 | Nav2 Docker image complete, 43 E2E tests passing | 1040 | v0.6.1 |
| 2026-03-12 | CI fixes, 997 tests, 40% coverage | 997 | v0.6.1 |
| 2026-03-09 | Docker-based ROS2 setup complete | 691 | v0.6.1 |
| 2026-03-04 | Week 1 TODO complete (gRPC, Redis, rate limiting) | - | v0.6.0 |
| 2026-03-03 | v0.5.1 released, 92/100 health score | 269 | v0.5.1 |
| 2026-03-02 | ROS2 connector complete, TDD compliance | 233 | v0.5.0+ |
| 2026-02-28 | Project audit, CI restored | 222 | v0.5.0 |

---

## Quick Reference

### Docker Commands
```bash
./scripts/docker-manager.sh start    # Start ROS2 container
./scripts/docker-manager.sh stop     # Stop container
./scripts/docker-manager.sh shell    # Interactive shell
./scripts/test-e2e.sh                # Run E2E tests
```

### Test Commands
```bash
pytest tests/unit/ -v                # Unit tests
pytest tests/e2e/ -v                 # E2E tests (requires Docker)
pytest tests/ -v -k "not e2e"        # Exclude E2E tests
```

---

## Identity

See `IDENTITY.md` and `USER.md` for assistant/user preferences.
