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

## Phase 2: Consolidate Architecture (Planned)

### 2.1 Single Gateway Entry Point

**Current:** Multiple entry points (cli.py, __main__.py, dashboard.py)
**Target:** Unified entry with subcommands

```python
agent-ros-bridge --mode=gateway    # Main gateway
agent-ros-bridge --mode=dashboard  # Dashboard only
agent-ros-bridge --mode=robot      # Robot API mode
```

### 2.2 Clean Module Structure

```
agent_ros_bridge/
├── core/              # Bridge, transport, messages
├── connectors/        # ROS1, ROS2, MQTT connectors
├── safety/           # Validation, limits, emergency stop
├── fleet/            # Multi-robot management
├── transports/       # WebSocket, gRPC, MQTT implementations
├── api/              # Robot API, actions
├── simulation/       # Gazebo, mock robots
└── utils/            # Shared utilities
```

### 2.3 Remove Bridge v1 Artifacts

- Ensure no imports from old gateway
- Remove compatibility shims
- Clean up `__init__.py` exports

---

## Phase 3: Complete Missing Features (Planned)

### 3.1 From Critique: Hardware Timing Validation

**Issue:** Safety timing claims not validated
**Implementation:**
- [ ] Add timing measurement hooks in safety nodes
- [ ] Create `scripts/validate_safety_timing.py`
- [ ] Document RT-PREEMPT requirements
- [ ] Add timing metrics to dashboard

### 3.2 From Critique: Production Config Generator

**Issue:** Config examples look "toy"
**Implementation:**
- [ ] Create `scripts/generate_production_config.py`
- [ ] Interactive robot specification input
- [ ] ISO 10218 compliance checks
- [ ] Safety zone visualization

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
| Fleet Management | ❌ Missing | High |
| Safety Configuration | ⚠️ Partial | High |
| Troubleshooting | ⚠️ Partial | Medium |
| Performance Tuning | ❌ Missing | Medium |

---

## Execution Checklist

### Phase 1: Strip (Target: -20% code volume)
- [ ] Identify all duplicate files
- [ ] Identify unused imports/functions
- [ ] Create deprecation list
- [ ] Execute removal
- [ ] Verify tests still pass

### Phase 2: Consolidate
- [ ] Merge discovery files
- [ ] Unify config loading
- [ ] Clean up module exports
- [ ] Update imports throughout

### Phase 3: Features
- [ ] Hardware timing validation script
- [ ] Production config generator
- [ ] Fleet coordination v1
- [ ] LLM integration completion

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
| 2026-04-10 | 1 | Analyze dead code | 🔄 In Progress |
