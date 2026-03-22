# Phase 1 Implementation: Shadow Mode Framework

**Goal:** Build the foundation for AI-human decision comparison
**Timeline:** 2 weeks
**Target:** v0.6.4 release

---

## Week 1: Core Framework

### Day 1-2: Decision Logger

**Files to create:**
- `agent_ros_bridge/shadow/__init__.py`
- `agent_ros_bridge/shadow/decision_logger.py`
- `agent_ros_bridge/shadow/models.py`

**Requirements:**
```python
@dataclass
class DecisionRecord:
    timestamp: datetime
    robot_id: str
    context: dict  # Robot state, environment, task
    ai_proposal: dict  # What AI suggested
    human_action: dict  # What human did
    outcome: dict  # Success/failure, timing, issues
    agreement: bool  # Did AI and human match?
```

**Storage:**
- SQLite for local logging
- JSONL for easy analysis
- Optional: Redis for real-time streaming

### Day 3-4: Comparison Engine

**Files to create:**
- `agent_ros_bridge/shadow/comparator.py`

**Metrics to track:**
- Agreement rate (AI vs human)
- Latency (AI decision time)
- Confidence calibration
- Safety violations
- Task success rate

### Day 5: Integration Points

**Modify:**
- `agent_ros_bridge/ai/intent_parser.py` - Log AI proposals
- `agent_ros_bridge/gateway_v2/` - Log human commands
- `agent_ros_bridge/robot_api.py` - Log actual outcomes

---

## Week 2: Dashboard & Validation

### Day 6-7: Metrics Dashboard

**Files to create:**
- `agent_ros_bridge/shadow/dashboard.py`
- Web UI for real-time metrics

**Display:**
- Live agreement rate
- Recent decisions
- Trend charts
- Alert thresholds

### Day 8-9: Validation & Tests

**Files to create:**
- `tests/unit/shadow/test_decision_logger.py`
- `tests/unit/shadow/test_comparator.py`

**Test coverage target:** 90%

### Day 10: Documentation & Release Prep

**Update:**
- README.md with shadow mode usage
- CHANGELOG.md for v0.6.4
- docs/SHADOW_MODE.md guide

---

## Implementation Order

```
Priority 1 (Must have):
├── Decision logger with SQLite backend
├── Basic comparison metrics
└── Integration with intent parser

Priority 2 (Should have):
├── Real-time dashboard
├── Redis streaming option
└── Comprehensive tests

Priority 3 (Nice to have):
├── Advanced analytics
├── Alert system
└── Export to external tools
```

---

## Success Criteria

| Metric | Target | Measurement |
|--------|--------|-------------|
| Decision logging | 100% | All commands logged |
| Agreement detection | Working | AI vs human comparison |
| Dashboard | Functional | Real-time metrics display |
| Test coverage | >90% | Unit tests passing |
| Documentation | Complete | Usage guide written |

---

## Files Checklist

### New Files
- [ ] `agent_ros_bridge/shadow/__init__.py`
- [ ] `agent_ros_bridge/shadow/models.py`
- [ ] `agent_ros_bridge/shadow/decision_logger.py`
- [ ] `agent_ros_bridge/shadow/comparator.py`
- [ ] `agent_ros_bridge/shadow/dashboard.py`
- [ ] `tests/unit/shadow/test_decision_logger.py`
- [ ] `tests/unit/shadow/test_comparator.py`
- [ ] `docs/SHADOW_MODE.md`
- [ ] `examples/demo_shadow_mode.py`

### Modified Files
- [ ] `agent_ros_bridge/ai/intent_parser.py` (add logging hooks)
- [ ] `agent_ros_bridge/gateway_v2/` (add logging hooks)
- [ ] `pyproject.toml` (add shadow dependencies)
- [ ] `CHANGELOG.md`

---

## Daily Standup Template

**Yesterday:**
- What was completed?

**Today:**
- What will be worked on?

**Blockers:**
- Any issues or dependencies?

---

Ready to start Day 1?
