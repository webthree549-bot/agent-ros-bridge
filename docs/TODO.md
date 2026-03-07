# Agent ROS Bridge: Unified Roadmap v0.6.1-v0.7.0

## Executive Summary

This roadmap consolidates all architecture analysis into a **unified, feasible implementation plan** based on **Option D: ROS-Native AI Architecture**.

**Key Principles:**
1. **Safety-First:** Hardware-enforced safety independent from AI
2. **ROS-Native:** All AI components as ROS nodes (observable, debuggable)
3. **Phased Rollout:** Validate at each stage before proceeding
4. **Dual-Track:** Agent and ROS engineering in parallel

---

## Legend

- 🔴 **Critical** - Must have for safety/production
- 🟡 **High** - Important for adoption
- 🟢 **Medium** - Nice to have
- ⚪ **Low** - Future enhancement
- ✅ **Done** - Completed
- 🚧 **In Progress** - Currently being implemented

---

## Architecture Overview (Option D)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    AGENT ROS BRIDGE v0.6.1+ ARCHITECTURE               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  AI REASONING LAYER (ROS Nodes)                                 │   │
│  │                                                                  │   │
│  │  /ai/intent_parser        → Rule-based + bounded LLM fallback   │   │
│  │  /ai/context_manager      → ROS topology integration            │   │
│  │  /ai/motion_planner       → SMT-based formal verification       │   │
│  │  /ai/execution_monitor    → Progress tracking, anomaly detect   │   │
│  │                                                                  │   │
│  │  Characteristics:                                                │   │
│  │  - Deterministic fast path (rule-based)                         │   │
│  │  - Bounded non-determinism (LLM with timeout/schema)            │   │
│  │  - Full observability (ros2 topic, rviz, bags)                  │   │
│  │  - Latency SLA: <100ms end-to-end                               │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                           ↓                                             │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  SAFETY CRITICAL LAYER (Independent, Hardware-Enforced)         │   │
│  │                                                                  │   │
│  │  /safety/validator        → Formal verification, <10ms response │   │
│  │  /safety/limits           → Hardware-enforced bounds            │   │
│  │  /safety/emergency_stop   → Physical circuit, <50ms             │   │
│  │  /safety/watchdog         → Independent heartbeat monitor       │   │
│  │                                                                  │   │
│  │  Characteristics:                                                │   │
│  │  - CANNOT be overridden by AI layer                             │   │
│  │  - Hard real-time (1kHz monitoring)                             │   │
│  │  - Runs on safety-rated hardware (PLC)                          │   │
│  │  - ISO 10218-1/2 compliant                                      │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                           ↓ (Only if safety layer approves)             │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  EXECUTION LAYER (Standard ROS2)                                │   │
│  │                                                                  │   │
│  │  /nav2/navigator          → Navigation (existing)               │   │
│  │  /moveit/move_group       → Manipulation (existing)             │   │
│  │  /controller_server       → Low-level control (existing)        │   │
│  │                                                                  │   │
│  │  (Leverages existing ROS2 ecosystem)                            │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  TRANSPORT LAYER (Preserved from v0.6.0)                        │   │
│  │                                                                  │   │
│  │  WebSocket, MQTT, gRPC    → AI agent interfaces                 │   │
│  │  ROS1/ROS2 connectors     → Robot interfaces                    │   │
│  │                                                                  │   │
│  │  (Backward compatible with v0.6.0)                              │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

All Analysis Documents Referenced:
- docs/NL2ROS_DEEP_ANALYSIS.md
- docs/ROS_TOPOLOGY_CONTEXT_ANALYSIS.md
- docs/DYNAMIC_SKILL_SYSTEM_ANALYSIS.md
- docs/ROS_NATIVE_AI_ARCHITECTURE.md
- docs/DUAL_TRACK_ENGINEERING.md
- docs/FINAL_DECISION_METHODOLOGY.md
"""

---

## Phase 1: v0.6.1 Foundation (Months 1-2)

**Goal:** Safe, deterministic foundation with ROS-native architecture
**Confidence Target:** 6/10 → 7/10
**Team:** 4 engineers (2 Agent, 2 ROS)

### 1.1 Interface Contracts (Week 1) 🔴

**Deliverable:** Frozen interface definitions

- [ ] Define ROS message types for AI-Safety-Execution communication
- [ ] Specify latency SLAs (Intent: <10ms, Safety: <10ms, E2E: <100ms)
- [ ] Document QoS requirements (reliable, deadline, liveliness)
- [ ] Create interface compatibility tests

**TDD Tests:**
```python
def test_interface_latency_sla():
    response = call_intent_service("test")
    assert response.latency_ms < 10

def test_safety_response_time():
    result = safety_validator.validate(test_command)
    assert result.response_time_ms < 10
```

### 1.2 Safety Foundation (Weeks 1-4) 🔴 CRITICAL

**Deliverable:** Hardware-enforced safety layer

- [ ] **/safety/limits node** — Hardware-enforced bounds
  - Configuration: max_velocity, max_force, workspace_bounds
  - Storage: Safety-rated PLC (not modifiable by software)
  - Validation: Formal verification of limit enforcement
  
- [ ] **/safety/validator node** — Real-time validation
  - Input: Motion plans from AI layer
  - Output: Approve/Reject with certificate
  - Timing: <10ms response (hard real-time)
  - Algorithm: SMT solver + constraint checking (deterministic)
  
- [ ] **/safety/emergency_stop node** — Physical e-stop
  - Hardware: Physical relay circuit
  - Latency: <50ms from trigger to motor stop
  - Independence: Cannot be disabled by software
  
- [ ] **/safety/watchdog node** — Heartbeat monitoring
  - Frequency: 1kHz monitoring
  - Action: Trigger e-stop if AI layer unresponsive (>100ms)

**Safety Gate 1 (End of Week 4):**
- [ ] All unsafe commands rejected
- [ ] Emergency stop <50ms verified
- [ ] Hardware limits cannot be overridden
- [ ] Zero safety violations in 1000+ test scenarios

### 1.3 Intent Parser (Weeks 2-4) 🔴

**Deliverable:** ROS-native intent parsing

- [ ] **/ai/intent_parser node** — Natural language understanding
  - Fast path: Rule-based parser (<5ms, deterministic)
  - Fallback: Bounded LLM (timeout=100ms, temperature=0)
  - Output: Intent message with confidence score
  - Schema validation: Pydantic models (no free-form text)
  
- [ ] **Intent categories:**
  - NAVIGATE: Movement commands
  - MANIPULATE: Arm/gripper commands
  - SENSE: Perception commands
  - QUERY: Status requests
  - SAFETY: Emergency commands
  
- [ ] **Confidence scoring:**
  - >0.95: Auto-execute (if safety validates)
  - 0.80-0.95: Suggest, require confirmation
  - <0.80: Reject, ask for clarification

**TDD Tests:**
```python
def test_intent_parsing_latency():
    result = intent_parser.parse("Go forward")
    assert result.latency_ms < 10
    assert result.confidence > 0.95

def test_low_confidence_rejection():
    result = intent_parser.parse("Do something")
    assert result.confidence < 0.80
    assert result.action == "REQUEST_CLARIFICATION"
```

### 1.4 Context Manager (Weeks 3-4) 🟡

**Deliverable:** ROS topology integration

- [ ] **/ai/context_manager node** — World state tracking
  - Subscribe to: /tf, /robot_state, /world_state
  - Maintain: Current pose, available capabilities, known locations
  - Cache: Recent observations for anaphora resolution
  
- [ ] **Reference resolution:**
  - "Kitchen" → lookup in semantic map
  - "It" → resolve from conversation history
  - "There" → resolve from spatial context

**Integration Tests:**
```python
def test_spatial_reference_resolution():
    context.set_current_location("living_room")
    result = context.resolve("go to the kitchen")
    assert result.target == semantic_map["kitchen"]
```

### 1.5 Simulation Environment Setup (Week 2-3) 🟡

**Deliverable:** Physics simulation for safe development

- [ ] **Gazebo + ROS2 environment**
  - Install and configure Gazebo Ignition/Fortress
  - Setup ROS2 Humble integration
  - Configure physics engine (ODE/Bullet)
  
- [ ] **Robot models**
  - TurtleBot3 Waffle (SDF, meshes, plugins)
  - UR5 arm (SDF, kinematics, controllers)
  - Sensor plugins (camera, lidar, IMU)
  
- [ ] **Test environments**
  - Warehouse world (aisles, shelves, obstacles)
  - Office world (rooms, corridors, elevators)
  - Outdoor world (terrain, lighting variations)
  
- [ ] **CI/CD integration**
  - GitHub Actions workflow for simulation tests
  - Automated scenario execution
  - Results reporting and artifact storage

### 1.6 Simulation-Based Testing (Week 3-4) 🟡

**Deliverable:** Comprehensive simulation test suite

- [ ] **Unit tests with mocked ROS (L1)**
  - Test individual nodes without physics
  - Fast execution (100x real-time)
  - >90% code coverage
  
- [ ] **Physics simulation tests (L2)**
  - Navigation: 100+ scenarios (empty, obstacles, dynamic)
  - Safety: Emergency stop, collision avoidance
  - Performance: Latency under load
  
- [ ] **Parallel simulation**
  - 1000 scenarios in parallel
  - 100 worker processes
  - Complete in <1 hour
  
- [ ] **Safety scenario validation**
  - Human suddenly appears
  - Sensor failure (lidar stops)
  - Unsafe velocity command
  - Network latency spike

### 1.7 Integration & Validation (Week 4) 🔴

**Deliverable:** Working foundation system validated in simulation

- [ ] End-to-end test in simulation: "Go forward" → parse → validate → execute
- [ ] Latency validation: <100ms total pipeline
- [ ] Safety validation: All commands pass through /safety/validator
- [ ] Simulation-to-real correlation: Verify sim matches real robot behavior
- [ ] Documentation: Architecture diagrams, API docs, simulation guide

**Gate 1 Criteria:**
- [ ] Zero safety violations in simulation
- [ ] <100ms end-to-end latency in simulation
- [ ] 100% command validation
- [ ] All tests passing (unit + simulation)
- [ ] Simulation environment documented and reproducible

---

## Phase 2: v0.6.2 Assisted AI (Months 3-4)

**Goal:** AI-assisted commands with human confirmation
**Confidence Target:** 7/10 → 7.5/10
**Team:** 6 engineers (3 Agent, 3 ROS)

### 2.1 Motion Planner (Weeks 5-6) 🔴

**Deliverable:** Formal verification-based planning

- [ ] **/ai/motion_planner node** — Trajectory generation
  - Input: Intent + constraints from context manager
  - Algorithm: Motion primitives + SMT-based verification
  - Output: Validated trajectory with safety certificate
  - Fallback: Pre-verified motion library
  
- [ ] **Planning capabilities:**
  - Navigation: Nav2 integration (existing)
  - Manipulation: MoveIt2 integration (existing)
  - Constraints: Obstacle avoidance, joint limits, dynamics
  
- [ ] **Safety certificate generation:**
  - Formal proof of constraint satisfaction
  - Valid time window (30 seconds)
  - Unique validation ID for traceability

**TDD Tests:**
```python
def test_motion_planning_with_verification():
    plan = motion_planner.plan(goal="kitchen")
    assert plan.has_safety_certificate
    assert plan.verification_proof is not None
```

### 2.2 Execution Monitor (Weeks 6-7) 🟡

**Deliverable:** Runtime monitoring and recovery

- [ ] **/ai/execution_monitor node** — Progress tracking
  - Subscribe to: Robot telemetry, action feedback
  - Detect: Progress, anomalies, failures
  - Trigger: Recovery behaviors, human escalation
  
- [ ] **Monitoring capabilities:**
  - Progress tracking: % complete, ETA
  - Anomaly detection: Deviation from plan, unexpected obstacles
  - Failure recovery: Retry, replan, escalate

### 2.3 Human-in-the-Loop Interface (Weeks 7-8) 🔴

**Deliverable:** Confirmation system for AI suggestions

- [ ] **Confirmation workflow:**
  - AI suggests action → Human reviews → Human approves/rejects
  - Low confidence (<0.95) → Always confirm
  - Novel commands → Always confirm
  - Emergency commands → Never confirm (execute immediately)
  
- [ ] **Explanation generation:**
  - Natural language rationale for AI decisions
  - Visualization of planned trajectory
  - Risk assessment (low/medium/high)

**User Story:**
```
User: "Go to the kitchen"
AI: "I'll navigate to the kitchen (distance: 5.2m, ETA: 45s). 
     This path is clear of obstacles. Approve?"
User: [Approve]
AI: [Executes]
```

### 2.4 Simulation-Based Validation (Weeks 7-8) 🟡

**Deliverable:** Large-scale simulation validation before real-world shadow mode

- [ ] **Scale simulation testing**
  - 10,000 scenarios in parallel
  - Coverage: Navigation, manipulation, safety edge cases
  - Performance: Load testing, stress testing
  
- [ ] **AI training in simulation**
  - RL environment for navigation (NavigationEnv)
  - Domain randomization (friction, mass, sensor noise)
  - Train policies: 1M+ episodes in simulation
  
- [ ] **Sim-to-real validation**
  - Validate simulation matches real robot (correlation >90%)
  - Fine-tune on real robot (minimal data)
  - Verify transfer success rate >90%

### 2.5 Shadow Mode (Week 8) 🟡

**Deliverable:** Parallel AI-human operation for validation

- [ ] **Shadow mode operation:**
  - AI proposes action (based on simulation-validated models)
  - Human executes (manually or via traditional interface)
  - System logs: AI proposal, human action, outcome
  - Compare: AI decision vs human decision
  
- [ ] **Success criteria:**
  - >95% agreement between AI and human
  - Zero safety violations
  - <5% human override rate (indicates AI is reasonable)
  - Correlation with simulation predictions >95%

**Gate 2 Criteria:**
- [ ] >95% AI-human agreement
- [ ] <100ms end-to-end latency
- [ ] Zero safety incidents
- [ ] User satisfaction >4/5

---

## Phase 3: v0.6.3 Supervised Autonomy (Months 5-6)

**Goal:** Supervised autonomy with constrained learning
**Confidence Target:** 7.5/10 → 8/10
**Team:** 6 engineers (3 Agent, 3 ROS)

### 3.1 Learning System (Weeks 9-10) 🟡

**Deliverable:** Constrained parameter optimization

- [ ] **/ai/learning node** — Experience-based optimization
  - Log: Execution outcomes, success/failure, timing
  - Analyze: Patterns, bottlenecks, optimization opportunities
  - Propose: Parameter changes (within safety bounds)
  
- [ ] **Constraints (hard limits):**
  - Max parameter change: 10% per iteration
  - Must stay within /safety/limits (hardware-enforced)
  - Requires human approval for all changes
  - Rollback capability (previous parameters stored)
  
- [ ] **What can be learned:**
  - Speed preferences per environment
  - Path preferences (shortest vs safest)
  - Timing patterns (busy hours, optimal schedules)

**What CANNOT be learned (safety critical):**
- Safety limits (hardware-enforced)
- Emergency stop behavior
- Workspace boundaries

### 3.2 Multi-Robot Coordination (Weeks 10-11) 🟡

**Deliverable:** Fleet-level task allocation

- [ ] **Fleet optimization:**
  - Task allocation based on: position, battery, capabilities
  - Conflict resolution: Resource contention, path conflicts
  - Coordination patterns: Formation, follow, distribute
  
- [ ] **Fleet monitoring:**
  - Real-time status: All robots, all tasks
  - Health monitoring: Battery, errors, maintenance needs
  - Performance analytics: Efficiency, success rates

### 3.3 Natural Language Refinement (Weeks 11-12) 🟢

**Deliverable:** Improved NL understanding

- [ ] **Enhanced parsing:**
  - Multi-turn conversations
  - Contextual references
  - Complex commands ("Go to kitchen, get water, bring to office")
  
- [ ] **Language expansion:**
  - Synonym handling
  - Dialect/terminology adaptation
  - Error correction ("Did you mean...?")

### 3.4 Simulation-First Extended Validation (Weeks 11-12) 🟡

**Deliverable:** 10,000+ hours of simulated operation

- [ ] **Large-scale simulation (Weeks 11-12)**
  - 100,000 scenarios in parallel cloud simulation
  - 10,000+ hours of simulated robot operation
  - Fleet simulation: 100 robots simultaneously
  - Stress testing: Peak load, resource contention
  
- [ ] **Digital twin validation**
  - High-fidelity simulation matching real robots
  - Real sensor noise profiles
  - Accurate physics (friction, mass, inertia)
  - Validate sim-to-real correlation >95%
  
- [ ] **Safety certification in simulation**
  - ISO 10218 scenarios: All passed
  - ISO/TS 15066 collaborative scenarios: All passed
  - Fuzz testing: 100,000 random commands
  - Zero unexpected safety violations

### 3.5 Extended Shadow Mode (v0.7.0 Phase 1) 🟡

**Deliverable:** 1000+ hours of real-world validated operation (cumulative)

- [ ] **Phase 1 (v0.6.3 - Weeks 11-12):** 200+ hours real-world
  - 2 robots, 12 hours/day, 7 days/week
  - Focus: Core functionality validation
  - Validate simulation predictions match reality
  
- [ ] **Phase 2 (v0.7.0 Months 7-8):** 800+ additional hours
  - 5 robots, 16 hours/day, 5 days/week
  - Diverse scenarios: Different robots, environments, tasks
  - Seasonal variations, peak load testing
  
- [ ] **Cumulative target:** 1000+ hours before 50% fleet rollout

**Why 1000 hours real-world after 10,000 hours simulation?**
- Simulation catches 99% of issues safely and quickly
- Real-world validates sim-to-real transfer
- Statistical significance: Detect 1-in-1000 failure modes
- Industry standard: ISO 13849 requires extensive validation

**Gate 3 Criteria (End of v0.6.3):**
- [ ] 200+ hours real-world shadow mode
- [ ] 10,000+ hours simulation validation
- [ ] Zero safety incidents
- [ ] >98% AI-human agreement
- [ ] >95% sim-to-real correlation
- [ ] All edge cases documented with mitigations

**Gate 4 Criteria (Before 50% rollout in v0.7.0):**
- [ ] 1000+ hours cumulative real-world shadow mode
- [ ] External safety audit passed
- [ ] Regulatory compliance review complete

---

## Phase 4: v0.7.0 Production Autonomy (Month 7+)

**Goal:** Production-ready autonomous operation
**Confidence Target:** 8/10 → 9/10
**Team:** 8 engineers (4 Agent, 4 ROS) + Safety Officer

### 4.1 Gradual Rollout (Months 7-12)

**Deliverable:** Phased production deployment with simulation-validated foundation

- [ ] **Pre-rollout simulation validation (Months 7-8)**
  - Digital twin testing: Exact replica of production environment
  - 1000+ hours additional simulation (cumulative 11,000+ hours)
  - Hardware-in-loop testing: Real controllers + simulation
  - Final safety certification in simulation

- [ ] **Pre-rollout real-world validation (Months 7-8): Extended Shadow Mode**
  - Continue shadow mode to reach 1000+ cumulative hours
  - 5 robots, 16 hours/day operation
  - Validate diverse scenarios before any production deployment
  - Correlation check: Sim predictions vs real-world behavior

- [ ] **Rollout plan (Months 9-12):**
  - Month 9: 1 robot (pilot) - AFTER 11,000+ hours simulation + 1000+ hours shadow mode
  - Month 10: 5% of fleet
  - Month 11: 25% of fleet
  - Month 12: 50% of fleet (hold for evaluation)
  - Month 13+: 75% → 100% (if metrics positive)
  
- [ ] **Monitoring:**
  - Real-time metrics: Success rate, latency, safety events
  - Simulation comparison: Real behavior vs predicted
  - Automatic rollback: If error rate >0.1% or any safety event
  - Human oversight: 24/7 monitoring during rollout
  - Weekly review: Go/No-Go decision for next phase

### 4.2 Full Autonomy (Months 13-18)

**Deliverable:** Autonomous operation with minimal oversight

- [ ] **Autonomy levels:**
  - High confidence commands: Auto-execute
  - Medium confidence: Suggest, no confirmation needed
  - Low confidence: Suggest, require confirmation
  - Emergency: Auto-execute (safety critical)
  
- [ ] **Continuous improvement:**
  - Monthly parameter tuning (human-approved)
  - Quarterly model updates (validated in simulation)
  - Annual safety audits

### 4.3 Advanced Features (v0.7.1+) 🟢

**Future enhancements (post-v0.7.0):**

- [ ] Multi-language support
- [ ] Voice interface
- [ ] Predictive maintenance
- [ ] Swarm intelligence
- [ ] Cloud-based learning (aggregated, privacy-preserving)

---

## Resource Requirements

### Team Structure

| Phase | Duration | Team Size | Roles |
|-------|----------|-----------|-------|
| v0.6.1 | 2 months | 4 engineers | 2 Agent, 2 ROS |
| v0.6.2 | 2 months | 6 engineers | 3 Agent, 3 ROS |
| v0.6.3 | 2 months | 6 engineers | 3 Agent, 3 ROS |
| v0.7.0 | 6+ months | 8 engineers | 4 Agent, 4 ROS, 1 Safety Officer |

### Budget Estimate

| Phase | Engineering | Infrastructure | Safety Certification | **Total** |
|-------|-------------|----------------|---------------------|-----------|
| v0.6.1 | $200K | $20K | - | **$220K** |
| v0.6.2 | $300K | $30K | - | **$330K** |
| v0.6.3 | $300K | $30K | $50K | **$380K** |
| v0.7.0 | $900K | $100K | $200K | **$1.2M** |
| **Total** | **$1.7M** | **$180K** | **$250K** | **$2.13M** |

---

## Success Metrics

### Technical Metrics

| Metric | v0.6.1 | v0.6.2 | v0.6.3 | v0.7.0 |
|--------|--------|--------|--------|--------|
| Safety validation | 100% | 100% | 100% | 100% |
| End-to-end latency | <100ms | <100ms | <100ms | <100ms |
| AI-human agreement | N/A | >95% | >98% | >99% |
| Task success rate | 90% | 93% | 96% | 99% |
| Simulation hours | N/A | 1,000 (v0.6.2) | 10,000 (v0.6.3) | 11,000+ (before pilot) |
| Shadow mode hours | N/A | 100 (v0.6.2) | 200+ (v0.6.3) | 1000+ (before 50% rollout) |

### Business Metrics

| Metric | Target |
|--------|--------|
| User productivity gain | 10x vs v0.6.0 |
| Training time reduction | 95% (2 days → 2 hours) |
| Setup time reduction | 95% (3 hours → 10 minutes) |
| Safety incidents | Zero |
| Customer satisfaction | >4.5/5 |

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Safety incident | Low | Critical | Hardware enforcement, independent validation |
| Schedule slip | Medium | High | Parallel tracks, weekly milestones, MVP approach |
| Performance issues | Medium | Medium | Continuous profiling, fallback algorithms |
| User adoption | Low | Medium | Progressive value delivery, extensive training |
| Technical debt | Medium | Medium | Code reviews, refactoring sprints, documentation |

---

## Documentation References

All analysis documents support this roadmap:

| Document | Purpose | Status |
|----------|---------|--------|
| `docs/NL2ROS_SYSTEM.md` | System design | ✅ Referenced |
| `docs/NL2ROS_DEEP_ANALYSIS.md` | Physical execution pipeline | ✅ Referenced |
| `docs/ROS_TOPOLOGY_CONTEXT_ANALYSIS.md` | Context system | ✅ Referenced |
| `docs/DYNAMIC_SKILL_SYSTEM_ANALYSIS.md` | Skill architecture | ✅ Referenced |
| `docs/ROS_NATIVE_AI_ARCHITECTURE.md` | ROS-native implementation | ✅ Referenced |
| `docs/DUAL_TRACK_ENGINEERING.md` | Development process | ✅ Referenced |
| `docs/FINAL_DECISION_METHODOLOGY.md` | Decision rationale | ✅ Referenced |
| `docs/V0_6_1_USER_STORIES.md` | User impact | ✅ Referenced |
| `docs/V0_6_1_UPGRADE_JOURNEY.md` | Migration guide | ✅ Referenced |
| `docs/V0_6_1_EXPERT_CRITICAL_ANALYSIS.md` | Safety review | ✅ Addressed |

---

## Decision

**APPROVED:** Proceed with unified roadmap based on Option D (ROS-Native AI Architecture)

**Rationale:**
- Highest weighted score (8.35/10) from decision methodology
- Best ROI (900%) with manageable risk
- Safety-first approach with hardware enforcement
- Phased rollout enables validation at each stage
- ROS-native architecture provides observability and debuggability

**Confidence Level:** 8/10

**Next Steps:**
1. Week 1: Finalize interface contracts
2. Week 2: Begin dual-track engineering
3. Week 4: Gate 1 (Safety Foundation)
4. Month 2: v0.6.1 release

---

**Roadmap Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Approved for Implementation
