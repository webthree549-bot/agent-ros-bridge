# Phase 2: Dashboard & Integration (TDD)

**Goal:** Real-time dashboard and system integration
**Timeline:** 1 week
**Target:** v0.6.4 release

---

## TDD Approach

```
Week 2:
Day 1: Write failing tests for dashboard API (RED)
Day 2: Implement dashboard API (GREEN)
Day 3: Write failing tests for integration hooks (RED)
Day 4: Implement integration hooks (GREEN)
Day 5: Refactor, add web UI, documentation
```

---

## Components

### 1. Dashboard API

**Purpose:** HTTP API for real-time metrics

**Endpoints:**
- `GET /api/metrics` - Current agreement rate, totals
- `GET /api/decisions` - Recent decisions (paginated)
- `GET /api/robots/{id}` - Per-robot metrics
- `WebSocket /ws/metrics` - Real-time updates

**Files:**
- `agent_ros_bridge/shadow/dashboard.py`
- `tests/unit/shadow/test_dashboard.py`

### 2. Integration Hooks

**Purpose:** Connect shadow mode to existing systems

**Integration Points:**
- `intent_parser.py` - Log AI proposals
- `gateway_v2/` - Log human commands
- `robot_api.py` - Log outcomes

**Files:**
- `agent_ros_bridge/shadow/integration.py`
- `tests/unit/shadow/test_integration.py`

### 3. Web Dashboard

**Purpose:** Visual interface for operators

**Features:**
- Live agreement rate display
- Decision history table
- Robot status cards
- Alert notifications

**Files:**
- `agent_ros_bridge/shadow/static/index.html`
- `agent_ros_bridge/shadow/static/dashboard.js`

---

## Success Criteria

| Metric | Target |
|--------|--------|
| Dashboard API | All endpoints working |
| WebSocket | Real-time updates <1s latency |
| Integration | All commands logged |
| Test Coverage | >90% |
| UI | Functional in browser |

---

## Day 1: Dashboard API Tests (RED)

Start with failing tests for:
1. Metrics endpoint returns correct data
2. Decisions endpoint supports pagination
3. WebSocket broadcasts updates
4. Error handling works

---

Ready to write failing tests?
