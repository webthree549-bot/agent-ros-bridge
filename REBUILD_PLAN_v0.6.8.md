# Rebuild Plan - v0.6.8

**Date:** 2026-04-10  
**Base Version:** v0.6.7  
**Target Version:** v0.6.8  
**Goal:** Strip dead code, consolidate architecture, complete missing features

---

## Phase 1: Strip Dead Code (In Progress)

### 1.1 Duplicates to Remove

| File | Reason | Action |
|------|--------|--------|
| `discovery_hardened.py` | Duplicate of `discovery.py` with minor changes | Consolidate into `discovery.py` |
| `gateway_v2/` vs old `gateway/` | v2 exists but references may remain | Ensure only v2 is used |
| `validation/` vs `safety/` | Overlapping concerns | Consolidate safety validation |

### 1.2 Unused/Deprecated Modules

**Candidates for removal:**
- `middleware/` - Check if actually used
- `proto/` - Check if protobuf is used vs generated files
- `learning/` - Check usage vs `ai/` module
- Old transport implementations in `transports/` root

### 1.3 Stale Configuration

- Review `config/` for unused YAML files
- Consolidate config loading logic

---

## Phase 2: Consolidate Architecture ✅ COMPLETE

### 2.1 Single Gateway Entry Point ✅

**Delivered:** `agent_ros_bridge/main.py` - Unified entry point with subcommands

```bash
agent-ros-bridge start              # Main gateway (default)
agent-ros-bridge dashboard          # Dashboard only
agent-ros-bridge robot --device-id  # Robot API mode
agent-ros-bridge status             # Check status
agent-ros-bridge config --init      # Manage config
```

### 2.2 Completed Cleanups ✅

- ✅ Discovery module consolidation (merged discovery_hardened.py)
- ✅ Transport stub removal (deleted old transports/)
- ✅ Unified CLI entry point

### 2.3 Remaining (Optional)

- Clean module structure reorganization (major refactor - defer to v0.7.0)
- Remove Bridge v1 artifacts (if any remain)

---

## Phase 3: Complete Missing Features (Planned)

### 3.1 From Critique: Hardware Timing Validation

**Issue:** Safety timing claims not validated
**Implementation:**
- [x] Add timing measurement hooks in safety nodes
- [x] Create `scripts/validate_safety_timing.py`
- [x] Document RT-PREEMPT requirements
- [ ] Add timing metrics to dashboard (pending)

### 3.2 From Critique: Production Config Generator

**Issue:** Config examples look "toy"
**Implementation:**
- [x] Create `scripts/generate_production_config.py`
- [x] Interactive robot specification input
- [x] ISO 10218 compliance checks
- [x] Safety zone visualization guidance

### 3.3 From Critique: Fleet Coordination Algorithms

**Issue:** No actual multi-robot task allocation
**Implementation:**
- [ ] Auction-based task allocation
- [ ] Distributed consensus for coordination
- [ ] Path conflict detection
- [ ] Fleet state machine

### 3.4 LLM Integration (Current Gap)

**Issue:** LLM processing happens upstream
**Implementation:**
- [ ] Complete `llm_parser.py` implementation
- [ ] Add local LLM option (ollama)
- [ ] Intent classification
- [ ] Command confidence scoring

---

## Phase 4: Documentation (Planned)

### 4.1 API Reference Completion

**Current:** `docs/API_REFERENCE.md` exists but incomplete
**Tasks:**
- [ ] Complete WebSocket message reference
- [ ] Add gRPC service definitions
- [ ] Document all error codes
- [ ] Add code examples for each endpoint

### 4.2 Architecture Documentation

**Tasks:**
- [ ] Update `docs/ARCHITECTURE_V2.md` to current implementation
- [ ] Add sequence diagrams for common flows
- [ ] Document safety layer integration
- [ ] Add deployment architecture diagrams

### 4.3 Deployment Guide

**Tasks:**
- [ ] Production deployment checklist
- [ ] Docker deployment guide
- [ ] Kubernetes deployment manifests
- [ ] Security hardening guide

### 4.4 Missing Guides

| Guide | Status | Priority |
|-------|--------|----------|
| Fleet Management | ✅ Complete | High |
| Safety Configuration | ✅ Complete | High |
| Troubleshooting | ✅ Complete | Medium |
| Performance Tuning | ❌ Missing | Medium |

---

## Execution Checklist

### Phase 1: Strip (Target: -20% code volume)
- [ ] Identify all duplicate files
- [ ] Identify unused imports/functions
- [ ] Create deprecation list
- [ ] Execute removal
- [ ] Verify tests still pass

### Phase 2: Consolidate ✅
- [x] Merge discovery files
- [x] Unify entry point (main.py)
- [x] Clean up module exports
- [x] Update imports throughout

### Phase 3: Features ✅
- [x] Hardware timing validation script
- [x] Production config generator
- [x] Fleet coordination v1 (auction-based)
- [x] LLM integration completion (local + cloud)

### Phase 4: Docs
- [ ] API reference completion
- [ ] Architecture update
- [ ] Deployment guide
- [ ] Troubleshooting guide

---

## Metrics

**Current:**
- 217 Python files
- 3.8M package size
- ~35,483 source LOC
- 2,231+ tests

**Target (v0.6.8):**
- ~180 Python files (-17%)
- <3.0M package size (-21%)
- ~30,000 source LOC (-15%)
- 2,500+ tests (+12%)
- 100% API documentation coverage

---

## Progress Log

| Date | Phase | Task | Status |
|------|-------|------|--------|
| 2026-04-10 | 1 | Create rebuild plan | ✅ Complete |
| 2026-04-10 | 1 | Analyze dead code | ✅ Complete |
| 2026-04-10 | 1 | Consolidate discovery modules | ✅ Complete |
| 2026-04-10 | 1 | Remove old stub transports | ✅ Complete |
| 2026-04-10 | 2 | Unified entry point | ✅ Complete - main.py with 5 subcommands |
| 2026-04-10 | 3 | Safety timing validation script | ✅ Complete |
| 2026-04-10 | 3 | Production config generator | ✅ Complete |
| 2026-04-10 | 3 | Dead code finder tool | ✅ Complete |
| 2026-04-10 | 3 | Auction-based task allocation | ✅ Complete |
| 2026-04-10 | 3 | Local LLM support | ✅ Complete |
| 2026-04-10 | 3 | Path conflict detection | ✅ Complete |
| 2026-04-10 | 4 | Fleet Management guide | ✅ Complete |
| 2026-04-10 | 4 | Troubleshooting guide | ✅ Complete |
| 2026-04-10 | - | Push to remote | ✅ Complete |
