# v0.6.1 Sprint Plan: Foundation Phase

## Executive Summary

**Sprint Duration:** 8 weeks (Month 1-2)  
**Team:** 4 engineers (2 Agent Track, 2 ROS Track)  
**Goal:** Safe, deterministic foundation with ROS-native architecture  
**Deliverable:** Working system validated in simulation

---

## Team Structure & Responsibilities

### Track A: Agent Engineering (2 Engineers)

| Engineer | Role | Primary Focus | Interfaces With |
|----------|------|---------------|-----------------|
| **ENG-1** | Agent AI Lead | Intent parser, context manager | ENG-2 (safety validation), ENG-3 (motion planning) |
| **ENG-3** | Motion Planning | Motion planner, execution monitor | ENG-1 (intent), ENG-2 (safety), ENG-4 (simulation) |

### Track B: ROS Engineering (2 Engineers)

| Engineer | Role | Primary Focus | Interfaces With |
|----------|------|---------------|-----------------|
| **ENG-2** | ROS Safety Lead | Safety validator, emergency stop | ALL (safety validation required) |
| **ENG-4** | Simulation Lead | Gazebo, CI/CD, test environments | ALL (testing infrastructure) |

---

## Week-by-Week Sprint Plan

### Week 1: Interface Definition & Setup

**Theme:** Contracts, environments, foundations

#### ENG-1 (Agent AI): Interface Contracts
- [ ] Define ROS service interfaces:
  - `/ai/parse_intent` (input: utterance, output: Intent)
  - `/ai/resolve_context` (input: reference, output: resolved entity)
- [ ] Create ROS message definitions:
  - `Intent.msg` (type, confidence, entities, constraints)
  - `Entity.msg` (type, value, confidence)
  - `ContextQuery.msg` / `ContextResponse.msg`
- [ ] Document latency SLAs: <10ms for intent parsing
- [ ] Create interface compatibility tests

**Deliverable:** `agent_ros_bridge_msgs/` package with all interfaces

#### ENG-2 (ROS Safety): Safety Architecture Design
- [ ] Define safety-critical ROS services:
  - `/safety/validate_motion` (input: trajectory, output: approval+certificate)
  - `/safety/get_limits` (output: current safety limits)
  - `/safety/emergency_stop` (input: trigger, output: confirmation)
- [ ] Define safety message types:
  - `SafetyCertificate.msg` (validation_id, expiry, constraints_checked)
  - `EmergencyStop.msg` (triggered, source, timestamp)
  - `SafetyLimits.msg` (max_velocity, max_force, workspace_bounds)
- [ ] Document timing requirements: <10ms validation response
- [ ] Create safety requirement specification (SRS)

**Deliverable:** Safety interface specification, hardware requirements list

#### ENG-3 (Motion Planning): Planning Interface Design
- [ ] Define motion planning ROS actions:
  - `/ai/plan_motion` (Action: goal pose → trajectory)
  - `/ai/execute_motion` (Action: trajectory → execution result)
- [ ] Define planning message types:
  - `MotionPlan.msg` (waypoints, timestamps, safety_certificate)
  - `ExecutionStatus.msg` (progress, anomalies, completion)
- [ ] Document integration points with Nav2/MoveIt2
- [ ] Define safety certificate requirements for plans

**Deliverable:** Motion planning interface specification

#### ENG-4 (Simulation): Environment Setup
- [ ] Setup Gazebo Ignition + ROS2 Humble development environment
- [ ] Create Docker development container
- [ ] Setup CI/CD pipeline structure (GitHub Actions)
- [ ] Create basic test world (empty warehouse)
- [ ] Document simulation workflow

**Deliverable:** Development environment, CI/CD skeleton

**Week 1 Sync Meeting:**
- Review all interface definitions
- Identify integration points
- Resolve conflicts
- Confirm hardware procurement list

---

### Week 2: Core Implementation (Part 1)

**Theme:** Basic functionality, safety foundation

#### ENG-1 (Agent AI): Intent Parser Foundation
- [ ] Implement rule-based intent parser (deterministic)
  - Pattern matching for NAVIGATE, MANIPULATE, SENSE, QUERY
  - Regex-based entity extraction
  - Confidence scoring (0.0-1.0)
- [ ] Implement fast path (<5ms target)
- [ ] Unit tests for all intent types
- [ ] Performance benchmarks

**Deliverable:** `/ai/intent_parser` node (basic version)

#### ENG-2 (ROS Safety): Safety Limits Node
- [ ] Implement `/safety/limits` node
  - Load limits from configuration
  - Hardware-enforced bounds (simulation placeholder)
  - Service interface for limit queries
- [ ] Create safety limit configuration format
- [ ] Unit tests for limit enforcement

**Deliverable:** `/safety/limits` node with configuration

#### ENG-3 (Motion Planning): Motion Primitive Library
- [ ] Define motion primitive types:
  - `navigate_to_pose` (using Nav2)
  - `move_arm` (using MoveIt2)
  - `gripper_action` (open/close)
- [ ] Create primitive execution interface
- [ ] Unit tests for each primitive type

**Deliverable:** Motion primitive library (basic)

#### ENG-4 (Simulation): Robot Models
- [ ] Create TurtleBot3 Waffle SDF model
  - Visual meshes
  - Collision geometries
  - Diff drive plugin
  - Camera plugin
  - Lidar plugin
- [ ] Create basic UR5 arm SDF model
  - Joint definitions
  - Controller plugin
- [ ] Test models in empty world

**Deliverable:** `models/` directory with robot SDFs

**Week 2 Sync Meeting:**
- Demo intent parser (rule-based)
- Demo safety limits
- Review robot models
- Plan integration tests

---

### Week 3: Core Implementation (Part 2)

**Theme:** Context, validation, simulation integration

#### ENG-1 (Agent AI): Context Manager
- [ ] Implement `/ai/context_manager` node
  - Subscribe to `/tf`, `/robot_state`
  - Maintain current pose, available capabilities
  - Resolve spatial references ("kitchen" → coordinates)
  - Anaphora resolution ("it", "there")
- [ ] Implement context query service
- [ ] Unit tests for context resolution

**Deliverable:** `/ai/context_manager` node

#### ENG-2 (ROS Safety): Safety Validator
- [ ] Implement `/safety/validator` node
  - Trajectory validation logic
  - Constraint checking (speed, workspace, joints)
  - Safety certificate generation
  - <10ms response time (target)
- [ ] Integration with `/safety/limits`
- [ ] Unit tests for validation logic

**Deliverable:** `/safety/validator` node (basic version)

#### ENG-3 (Motion Planning): Execution Monitor
- [ ] Implement `/ai/execution_monitor` node
  - Subscribe to robot telemetry
  - Progress tracking
  - Anomaly detection (deviation from plan)
  - Basic recovery (stop, retry)
- [ ] Unit tests for monitoring

**Deliverable:** `/ai/execution_monitor` node (basic)

#### ENG-4 (Simulation): Test Environments
- [ ] Create warehouse world
  - Aisles, shelves, loading docks
  - Static obstacles
  - Spawn points
- [ ] Create office world
  - Rooms, corridors, elevators
  - Furniture
- [ ] Create test scenario library (10 basic scenarios)
- [ ] CI/CD integration for automated testing

**Deliverable:** `worlds/` directory with test environments

**Week 3 Sync Meeting:**
- Demo context manager
- Demo safety validator
- Review test environments
- Plan end-to-end integration

---

### Week 4: Integration & Validation

**Theme:** End-to-end, safety, simulation

#### ENG-1 (Agent AI): Integration & LLM Fallback
- [ ] Integrate intent parser with context manager
- [ ] Implement LLM fallback (bounded)
  - OpenAI/Anthropic API integration
  - Timeout handling (100ms)
  - Schema validation (Pydantic)
  - Confidence thresholding
- [ ] End-to-end test: "Go forward" → parse → context → output
- [ ] Simulation tests (100 scenarios)

**Deliverable:** Complete `/ai/intent_parser` with fallback

#### ENG-2 (ROS Safety): Emergency Stop & Watchdog
- [ ] Implement `/safety/emergency_stop` node
  - Physical relay control (simulation placeholder)
  - <50ms response time
  - Cannot be overridden
- [ ] Implement `/safety/watchdog` node
  - 1kHz heartbeat monitoring
  - Auto-trigger e-stop on failure
- [ ] Safety scenario tests (50 scenarios)
  - Human appearance
  - Sensor failure
  - Unsafe commands
  - Network latency

**Deliverable:** Complete safety layer (4 nodes)

#### ENG-3 (Motion Planning): Safety Integration
- [ ] Integrate motion planner with safety validator
  - All plans must pass validation
  - Safety certificate attachment
  - Rejection handling
- [ ] Integrate with execution monitor
  - Real-time monitoring
  - Anomaly response
- [ ] End-to-end test: plan → validate → execute → monitor
- [ ] Simulation tests (50 scenarios)

**Deliverable:** Integrated motion planning with safety

#### ENG-4 (Simulation): Scale Testing & CI/CD
- [ ] Parallel simulation framework
  - 1000 scenarios in parallel
  - 100 worker processes
  - Results aggregation
- [ ] CI/CD pipeline complete
  - Automated testing on PR
  - Simulation test reports
  - Coverage reporting
- [ ] Performance benchmarks
  - Latency measurements
  - Resource utilization

**Deliverable:** Complete simulation infrastructure

**Week 4 Activities:**

**Day 1-2: Integration Testing**
- End-to-end tests: "Go forward", "Navigate to kitchen"
- Safety validation tests
- Performance benchmarks

**Day 3-4: Bug Fixes & Polish**
- Fix integration issues
- Performance optimization
- Documentation updates

**Day 5: Gate 1 Review**
- Demo to stakeholders
- Review success criteria
- Go/No-Go decision

---

## Integration Points & Dependencies

### Critical Path

```
Week 1: Interface definitions (ALL)
    ↓
Week 2: Basic implementation (ALL)
    ↓
Week 3: Core functionality (ALL)
    ↓
Week 4: Integration & validation (ALL)
    ↓
Gate 1: Foundation complete
```

### Cross-Team Dependencies

| Dependency | From | To | Week |
|------------|------|-----|------|
| Safety interface | ENG-2 | ENG-1, ENG-3 | 1 |
| Intent → Context | ENG-1 | ENG-1 (internal) | 3 |
| Plan validation | ENG-2 | ENG-3 | 3 |
| Sim environment | ENG-4 | ALL | 2 |
| E2E testing | ALL | ENG-4 | 4 |

### Interface Contracts

```python
# /ai/parse_intent service
class ParseIntent:
    request: string utterance
    response: 
        Intent intent
        float64 confidence
        string source  # "RULE_BASED" or "LLM_ASSISTED"
        float64 latency_ms

# /safety/validate_motion service
class ValidateMotion:
    request:
        MotionPlan plan
        RobotState current_state
    response:
        bool approved
        SafetyCertificate certificate (if approved)
        string rejection_reason (if not approved)

# /ai/plan_motion action
class PlanMotion:
    goal:
        PoseStamped target_pose
        Constraints constraints
    feedback:
        string stage
        float64 progress_percent
    result:
        MotionPlan plan
        SafetyCertificate certificate
        bool success
```

---

## Success Criteria (Gate 1)

### Technical Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Intent parsing latency | <10ms | 95th percentile |
| Safety validation latency | <10ms | 95th percentile |
| End-to-end latency | <100ms | 95th percentile |
| Test coverage | >90% | Code coverage tool |
| Simulation scenarios | 1000+ | Automated test count |
| Safety scenarios | 50+ | Manual + automated |

### Functional Requirements

- [ ] Intent parser handles 7 intent types (NAVIGATE, MANIPULATE, SENSE, QUERY, CONFIGURE, MISSION, SAFETY)
- [ ] Context manager resolves spatial and anaphoric references
- [ ] Safety validator approves/rejects all motion plans
- [ ] Emergency stop triggers in <50ms
- [ ] All motion plans include safety certificates
- [ ] Simulation environment supports 1000+ parallel tests

### Quality Requirements

- [ ] Zero safety violations in simulation
- [ ] All unit tests passing
- [ ] All integration tests passing
- [ ] Documentation complete (API docs, architecture diagrams)
- [ ] Code review complete (all PRs reviewed)

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Interface mismatch | Medium | High | Week 1 sync meeting, frozen interfaces |
| Safety validation too slow | Medium | Critical | Performance benchmarking in Week 2-3 |
| LLM integration issues | Medium | Medium | Rule-based fallback always available |
| Simulation environment delays | Low | Medium | Parallel work streams, ENG-4 dedicated |
| Integration complexity | Medium | High | Daily sync meetings, incremental integration |

---

## Communication Plan

### Daily Standups (15 min)
- What did you complete yesterday?
- What are you working on today?
- Blockers or dependencies?

### Weekly Sync (1 hour)
- Demo progress
- Review integration points
- Resolve conflicts
- Adjust plan if needed

### Gate 1 Review (2 hours)
- Demo complete system
- Review metrics
- Stakeholder feedback
- Go/No-Go decision

---

## Post-Sprint: v0.6.2 Preparation

### Handoff to v0.6.2

**ENG-1 Deliverables:**
- Intent parser (rule-based + LLM fallback)
- Context manager
- Interface documentation

**ENG-2 Deliverables:**
- Safety layer (4 nodes)
- Safety test suite
- Safety documentation

**ENG-3 Deliverables:**
- Motion primitive library
- Execution monitor (basic)
- Planning interface

**ENG-4 Deliverables:**
- Simulation environment
- CI/CD pipeline
- Test automation

### v0.6.2 Planning Input

- Performance benchmarks (identify bottlenecks)
- Integration pain points (improve interfaces)
- Missing features (prioritize for v0.6.2)
- Technical debt (plan refactoring)

---

## Appendix: Detailed Task Lists

### ENG-1 Detailed Tasks

**Week 1:**
- [ ] Create `agent_ros_bridge_msgs` package
- [ ] Define Intent.msg, Entity.msg
- [ ] Define ParseIntent.srv
- [ ] Define ContextQuery/Response
- [ ] Write interface documentation
- [ ] Create interface tests

**Week 2:**
- [ ] Create `/ai/intent_parser` package
- [ ] Implement rule-based parser
- [ ] Implement entity extraction
- [ ] Implement confidence scoring
- [ ] Write unit tests
- [ ] Performance benchmark

**Week 3:**
- [ ] Create `/ai/context_manager` package
- [ ] Implement ROS topic subscriptions
- [ ] Implement spatial resolution
- [ ] Implement anaphora resolution
- [ ] Write unit tests

**Week 4:**
- [ ] Integrate intent + context
- [ ] Implement LLM fallback
- [ ] Add timeout handling
- [ ] Add schema validation
- [ ] End-to-end tests
- [ ] Simulation tests

### ENG-2 Detailed Tasks

**Week 1:**
- [ ] Define safety message types
- [ ] Define safety services
- [ ] Write SRS document
- [ ] Document timing requirements
- [ ] Create safety test plan

**Week 2:**
- [ ] Create `/safety/limits` node
- [ ] Implement limit configuration
- [ ] Implement limit enforcement
- [ ] Write unit tests
- [ ] Performance benchmark

**Week 3:**
- [ ] Create `/safety/validator` node
- [ ] Implement trajectory validation
- [ ] Implement constraint checking
- [ ] Implement certificate generation
- [ ] Write unit tests

**Week 4:**
- [ ] Create `/safety/emergency_stop` node
- [ ] Create `/safety/watchdog` node
- [ ] Implement e-stop logic
- [ ] Implement watchdog logic
- [ ] Safety scenario tests
- [ ] Integration tests

### ENG-3 Detailed Tasks

**Week 1:**
- [ ] Define motion planning actions
- [ ] Define motion message types
- [ ] Document Nav2 integration
- [ ] Document MoveIt2 integration
- [ ] Create planning interface

**Week 2:**
- [ ] Create motion primitive library
- [ ] Implement navigate primitive
- [ ] Implement manipulate primitive
- [ ] Implement gripper primitive
- [ ] Write unit tests

**Week 3:**
- [ ] Create `/ai/execution_monitor` node
- [ ] Implement telemetry subscription
- [ ] Implement progress tracking
- [ ] Implement anomaly detection
- [ ] Write unit tests

**Week 4:**
- [ ] Integrate with safety validator
- [ ] Add safety certificate handling
- [ ] Integrate with execution monitor
- [ ] End-to-end tests
- [ ] Simulation tests

### ENG-4 Detailed Tasks

**Week 1:**
- [ ] Setup Gazebo Ignition
- [ ] Setup ROS2 Humble
- [ ] Create Docker container
- [ ] Setup CI/CD skeleton
- [ ] Document setup process

**Week 2:**
- [ ] Create TurtleBot3 model
- [ ] Add visual meshes
- [ ] Add collision geometries
- [ ] Add plugins (drive, camera, lidar)
- [ ] Test in empty world

**Week 3:**
- [ ] Create warehouse world
- [ ] Create office world
- [ ] Add static obstacles
- [ ] Create 10 test scenarios
- [ ] CI/CD integration

**Week 4:**
- [ ] Implement parallel runner
- [ ] Setup 100 worker processes
- [ ] Create results aggregation
- [ ] Performance benchmarks
- [ ] Complete CI/CD pipeline

---

**Sprint Plan Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Ready for execution
