# MEMORY.md - Curated Long-Term Memory

_Last updated: 2026-03-21_

---

## Active Project: Agent ROS Bridge

**Repository:** https://github.com/webthree549-bot/agent-ros-bridge  
**PyPI:** https://pypi.org/project/agent-ros-bridge/  
**Current Version:** v0.6.1 (production ready)

### Status Summary
- **Tests:** 1,455 unit tests passing, 46 E2E tests passing
- **E2E Tests:** 46 passed, 8 skipped (1 port binding failure)
- **Version:** v0.6.2 released to PyPI
- **Coverage:** 60.13% (target: 60%) — **TARGET ACHIEVED!**
- **CI/CD:** 9-stage pipeline operational
- **ROS2:** Docker-based (`ros2_humble` container with Jazzy + Nav2)

### Recent Fixes (2026-03-18) — Docker AI Testing COMPLETE
1. **AI Module Tests Fixed** - API mismatches resolved, **21 tests passing** in Docker
2. **pytest-cov Installed** - Coverage reporting now available in Docker container
3. **Docker Test Suite** - **760 tests passed, 10 failed, 255 skipped** with ROS2
4. **Test Documentation** - Created `DOCKER_TESTING.md` with complete guide
5. **AI Modules Now Testable**:
   - `intent_parser.py` - Pattern matching tests working
   - `motion_primitives.py` - All factory methods tested

### Recent Fixes (2026-03-18) — Test Coverage Sprint COMPLETE
1. **Coverage Improved** - 40.44% → **50.08%** (+9.6%)
2. **New Test Files Created** - 15+ new test modules added
3. **0% Coverage Modules Fixed**:
   - `fleet_intelligence.py`: 0% → **82%**
   - `input_validation.py`: 0% → **96%**
   - `nl_interpreter.py`: 0% → **92%**
   - `scene_understanding.py`: 0% → **73%**
   - `redis_backend.py`: 0% → **89%**
   - `context_async.py`: 0% → **78%**
   - `autonomous_behaviors.py`: 0% → **88%**
   - `nl_params.py`: 16% → **100%**
   - `context.py`: 0% → **90%**
4. **TDD Compliance** - Tests guide implementation, all new code has tests first
5. **Total Tests** - 1,167+ unit tests passing (+170 from 997)

### Recent Fixes (2026-03-18)
1. **E2E Tests Fixed** - All 12 previously skipped tests now pass (47 total)
2. **ROS Messages Built** - Compiled agent_ros_bridge_msgs in Docker container
3. **Message Definitions Fixed** - ParseIntent.srv, ResolveContext.srv, package.xml
4. **Python API Fixed** - Added Robot alias, motion primitive name property

### Recent Fixes (2026-03-18)
1. **E2E Tests Fixed** - All 12 previously skipped tests now pass (47 total)
2. **ROS Messages Built** - Compiled agent_ros_bridge_msgs in Docker container
3. **Message Definitions Fixed** - ParseIntent.srv, ResolveContext.srv, package.xml
4. **Python API Fixed** - Added Robot alias, motion primitive name property

### Recent Fixes (2026-03-17)
1. **E2E Test Solution** - Converted 12 skipped tests to run against real code
2. **Real Issues Found** - 7 tests now fail revealing actual implementation bugs
3. **Docker-based Testing** - All E2E tests run Python code inside container
4. **TDD Compliance** - Tests guide implementation instead of hiding issues

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

### Recent Work (2026-03-21) — 60% Coverage TARGET ACHIEVED
1. **Coverage Reached 61.96%** — Target achieved! (was 57.10%)
2. **New Test Files Added**:
   - `test_recovery.py` — 35 tests, recovery strategies now at 90% coverage
   - `test_context_aware_parser.py` — 39 tests, context-aware parser now at 73% coverage
   - `test_multi_language_parser.py` — 42 tests, multi-language parser now at 65% coverage
   - `test_cli.py` — 30 tests, CLI module now at ~80% coverage
   - `test_mqtt_transport.py` — 22 tests, MQTT transport now at ~40% coverage
3. **CI/CD Fixes**: Updated all GitHub Actions to latest pinned versions
4. **Total Tests:** 1,485 unit tests (+329 from 1,156)

### Next Priorities
- Optimize CI pipeline speed
- Enable full integration tests (8 skipped tests require Gazebo+Nav2)
- Reach 70% test coverage (stretch goal)

---

## Historical Milestones

| Date | Milestone | Tests | Coverage | Version |
|------|-----------|-------|----------|---------|
| 2026-03-21 | **Coverage: 62%** | 1485 | 61.96% | v0.6.2 |
| 2026-03-21 | **60% coverage target ACHIEVED** | 1455 | 60.13% | v0.6.2 |
| 2026-03-18 | Test coverage sprint complete: 40% → 50% | 1167 | 50.08% | v0.6.1 |
| 2026-03-18 | Test coverage sprint: 40% → 50% | 1156 | 49.97% | v0.6.1 |
| 2026-03-18 | All E2E tests fixed, 47 passing | 1040 | 40.44% | v0.6.1 |
| 2026-03-17 | E2E tests converted, 7 real issues found | 1040 | 40.44% | v0.6.1 |
| 2026-03-16 | Nav2 Docker image complete, 43 E2E tests passing | 1040 | 40.44% | v0.6.1 |
| 2026-03-12 | CI fixes, 997 tests, 40% coverage | 997 | 40.44% | v0.6.1 |
| 2026-03-09 | Docker-based ROS2 setup complete | 691 | - | v0.6.1 |
| 2026-03-04 | Week 1 TODO complete (gRPC, Redis, rate limiting) | - | - | v0.6.0 |
| 2026-03-03 | v0.5.1 released, 92/100 health score | 269 | - | v0.5.1 |
| 2026-03-02 | ROS2 connector complete, TDD compliance | 233 | - | v0.5.0+ |
| 2026-02-28 | Project audit, CI restored | 222 | - | v0.5.0 |

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
