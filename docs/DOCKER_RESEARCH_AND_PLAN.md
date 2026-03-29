# Docker Infrastructure Research & Fix Plan

**Date:** March 24, 2026  
**Researcher:** OpenClaw Agent  
**Status:** Critical Issues Identified

---

## Executive Summary

The project has a **fragmented and inconsistent Docker infrastructure** with:
- Multiple naming conventions (humble vs jazzy)
- Overlapping but different Dockerfiles
- Scripts that don't match the actual container setup
- Documentation referencing both versions inconsistently

**Impact:** Confusion for developers, potential deployment errors, maintenance burden.

---

## Research Findings

### 1. Container Naming Chaos

| What | Name | ROS Version Inside | Status |
|------|------|-------------------|--------|
| Running container | `ros2_humble` | **Jazzy** | ❌ Mismatch |
| Image tag | `agent-ros-bridge:ros2-humble` | Jazzy | ❌ Mismatch |
| Scripts reference | `ros2_humble` / `ros2-jazzy` | Mixed | ❌ Inconsistent |
| docker-compose | `ros2-humble-bridge` / `ros2-jazzy-bridge` | Both exist | ⚠️ Duplication |

### 2. Multiple Dockerfiles (8 total)

```
docker/
├── Dockerfile                          # Production Jazzy
├── Dockerfile.dev                      # Minimal dev
├── Dockerfile.mock                     # Mock mode
├── Dockerfile.ros1                     # ROS1 Noetic
├── Dockerfile.ros2                     # Generic ROS2 (builds humble)
├── Dockerfile.ros2.humble              # Humble desktop-full
├── Dockerfile.ros2.jazzy               # Jazzy desktop-full ⭐ CURRENT
└── Dockerfile.simulation               # Humble + Gazebo + Nav2 + TB3
```

**Problem:** `Dockerfile.simulation` uses Humble, but project has moved to Jazzy.

### 3. Script Inconsistencies

| Script | Container Name | Image Name | ROS Version |
|--------|---------------|------------|-------------|
| `scripts/docker-manager.sh` (root) | `ros2_humble` | `osrf/ros:humble-desktop` | Humble |
| `scripts/docker/start-ros2.sh` | `ros2_jazzy` (was `ros2_humble`) | `agent-ros-bridge:ros2-jazzy` | Jazzy |
| `scripts/docker/build-ros2-image.sh` | — | `agent-ros-bridge:ros2-humble` | Humble |
| `scripts/docker-manager.sh` (just fixed) | `ros2_jazzy` | `agent-ros-bridge:ros2-jazzy` | Jazzy |

### 4. Documentation Drift

Documents referencing **Humble**:
- `DEPLOYMENT.md` - "ROS2 Humble (optional)"
- `DOCKER_STRATEGY.md` - `ros2_humble` container name
- `NATIVE_ROS.md` - `ros-humble-desktop`
- `SIMULATION_FIRST_STRATEGY.md` - `osrf/ros:humble-desktop`

Documents referencing **Jazzy**:
- `ROS2_JAZZY_COMPATIBILITY.md` - Full Jazzy support
- `MACOS_SETUP_GUIDE.md` - `ros2-jazzy-bridge`
- `adr/0003-multi-ros-support.md` - Both versions

### 5. Root Cause Analysis

**How did this happen?**
1. Project started with ROS2 Humble (Ubuntu 22.04)
2. Upgraded to Jazzy (Ubuntu 24.04) for newer features
3. Partial migration - some files updated, others not
4. Running container was manually built with Jazzy packages but kept "humble" name
5. Scripts evolved independently with different naming

**Current State:**
- ✅ Code works with Jazzy
- ✅ Running container has Jazzy
- ❌ Naming is wrong
- ❌ Some Dockerfiles still use Humble
- ❌ Documentation inconsistent

---

## Fix Plan

### Phase 1: Standardize on Jazzy (Immediate - 1 day)

**Rationale:** Jazzy is newer, has better Nav2 support, and is what the code actually uses.

#### 1.1 Update All Dockerfiles to Jazzy
- [ ] `Dockerfile.simulation` - Change FROM to `osrf/ros:jazzy-desktop-full`
- [ ] `Dockerfile.ros2` - Change FROM to `ros:jazzy-ros-base`
- [ ] `scripts/docker/build-ros2-image.sh` - Update to jazzy
- [ ] Verify all apt packages use `ros-jazzy-*` prefix

#### 1.2 Standardize Container/Image Names
- [ ] Single container name: `ros2-jazzy-bridge`
- [ ] Single image name: `agent-ros-bridge:jazzy`
- [ ] Update all scripts consistently

#### 1.3 Update docker-compose.yml
- [ ] Remove `ros2-humble-bridge` service
- [ ] Keep only `ros2-jazzy-bridge`
- [ ] Update all environment variables

### Phase 2: Script Consolidation (2-3 days)

#### 2.1 Merge Duplicate Scripts
**Current:**
- `docker-manager.sh` (root) - uses Humble
- `scripts/docker-manager.sh` - uses Jazzy
- `scripts/docker/start-ros2.sh` - uses Jazzy

**Target:**
- Single `scripts/docker-manager.sh` (Jazzy)
- Deprecate root version

#### 2.2 Create Unified Build Script
```bash
scripts/docker/
├── build.sh          # Build image
├── start.sh          # Start container
├── stop.sh           # Stop container
├── shell.sh          # Enter container
└── logs.sh           # View logs
```

#### 2.3 Add Environment Detection
```bash
# Auto-detect if running in Docker
if [ -f /.dockerenv ]; then
    echo "Running inside container"
fi
```

### Phase 3: Documentation Update (1-2 days)

#### 3.1 Update All References
- [ ] `DEPLOYMENT.md` - Remove Humble references
- [ ] `DOCKER_STRATEGY.md` - Update to Jazzy
- [ ] `NATIVE_ROS.md` - Primary Jazzy, Humble as legacy
- [ ] `README.md` - Update Docker section

#### 3.2 Create Migration Guide
```markdown
## Migrating from Humble to Jazzy

### For Users
1. Stop old container: `docker stop ros2_humble`
2. Remove old container: `docker rm ros2_humble`
3. Start new container: `./scripts/docker-manager.sh start`

### For Developers
1. Update local scripts
2. Rebuild image: `./scripts/docker/build-ros2-image.sh`
```

### Phase 4: Testing & Validation (2 days)

#### 4.1 Build New Image
```bash
docker build -f docker/Dockerfile.ros2.jazzy -t agent-ros-bridge:jazzy .
```

#### 4.2 Run Integration Tests
- [ ] Test Gazebo P0 features in new container
- [ ] Verify Nav2 works
- [ ] Test agent-ros-bridge imports
- [ ] Run E2E tests

#### 4.3 Document Test Results

### Phase 5: Cleanup (1 day)

#### 5.1 Deprecate Old Files
- [ ] Move `Dockerfile.ros2.humble` to `docker/legacy/`
- [ ] Add deprecation notices
- [ ] Update CI to build only Jazzy

#### 5.2 Update CI/CD
- [ ] `.github/workflows/ci.yml` - Use Jazzy only
- [ ] Remove Humble matrix entries

#### 5.3 Final Verification
- [ ] All tests pass
- [ ] Documentation consistent
- [ ] Scripts work end-to-end

---

## Implementation Priority

| Priority | Phase | Effort | Impact |
|----------|-------|--------|--------|
| **P0 - Critical** | Phase 1 | 1 day | Fixes immediate confusion |
| **P1 - High** | Phase 2 | 2-3 days | Long-term maintainability |
| **P2 - Medium** | Phase 3 | 1-2 days | User experience |
| **P3 - Low** | Phase 4-5 | 3 days | Technical debt cleanup |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Breaking existing workflows | Medium | High | Migration guide, keep old container running |
| CI failures | Low | Medium | Test in feature branch first |
| User confusion | High | Medium | Clear documentation, deprecation notices |
| Humble users left behind | Low | Low | Document Humble as legacy, still works |

---

## Recommendation

**Proceed with Phase 1 immediately** - it fixes the immediate confusion with minimal risk.

**Phase 2-3** can be done incrementally over the next week.

**Phase 4-5** can wait until next sprint.

---

*Research completed: March 24, 2026*
