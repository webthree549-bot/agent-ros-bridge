# Safety Architecture V1.0
## Agent ROS Bridge v0.6.1

**Document Version:** 1.0  
**Date:** 2026-03-06  
**Author:** ENG-2 (ROS Safety Engineer)  
**Status:** Implementation Complete - Validation Pending

> **Note:** This architecture is implemented in code. Timing targets (10ms validation, 50ms e-stop) are **design specifications**, not yet validated on production hardware. Validation requires RT-PREEMPT kernel, hardware oscilloscope testing, and safety officer sign-off before production deployment.

---

## 1. Executive Summary

This document defines the safety architecture for Agent ROS Bridge v0.6.1. The safety system is designed as a layered defense-in-depth architecture that ensures robot operations remain within safe bounds while providing fast response to hazardous conditions.

### Key Design Principles
- **Fail-Safe:** Any system failure results in a safe state (stopped)
- **Defense in Depth:** Multiple independent safety layers
- **Hardware/Software Separation:** Critical safety functions can operate independently
- **Deterministic Timing:** Guaranteed response times for safety-critical operations
- **Auditability:** All safety events are logged for analysis

---

## 2. Safety Node Architecture

### 2.1 Node Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           SAFETY LAYER                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐    │
│  │ /safety/         │     │ /safety/         │     │ /safety/         │    │
│  │   validator      │◄────│   limits         │◄────│   emergency_stop │    │
│  │                  │     │                  │     │                  │    │
│  │ • Trajectory     │     │ • Load limits    │     │ • E-stop logic   │    │
│  │   validation     │     │ • Bound checks   │     │ • Cutoff control │    │
│  │ • Safety certs   │     │ • Zone checks    │     │ • Status pub     │    │
│  │ • Constraint chk │     │ • Limit caching  │     │ • Reset handling │    │
│  └────────┬─────────┘     └──────────────────┘     └────────▲─────────┘    │
│           │                                                  │              │
│           │         ┌──────────────────┐                     │              │
│           └────────►│ /safety/         │─────────────────────┘              │
│                     │   watchdog       │                                    │
│                     │                  │                                    │
│                     │ • 1kHz heartbeat │                                    │
│                     │ • Timeout detect │                                    │
│                     │ • Auto e-stop    │                                    │
│                     └──────────────────┘                                    │
│                                                                             │
│  ═══════════════════════════════════════════════════════════════════════   │
│                         HARDWARE INTERFACE LAYER                            │
│  ═══════════════════════════════════════════════════════════════════════   │
│                                                                             │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐    │
│  │ Hardware E-Stop  │     │ Motor Controllers│     │ Safety PLC       │    │
│  │   (Optional)     │     │   (Velocity      │     │   (Optional)     │    │
│  │                  │     │    Limits)       │     │                  │    │
│  └──────────────────┘     └──────────────────┘     └──────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         AI / PLANNING LAYER                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────┐     ┌──────────────────┐     ┌──────────────────┐    │
│  │ /ai/nl2ros       │     │ /ai/planner      │     │ /ai/executor     │    │
│  │                  │────►│                  │────►│                  │    │
│  │ • NL parsing     │     │ • Path planning  │     │ • Command exec   │    │
│  │ • Intent extract │     │ • Trajectory gen │     │ • Safety check   │    │
│  └──────────────────┘     └──────────────────┘     └────────┬─────────┘    │
│                                                             │               │
│                                                             ▼               │
│                                                    ┌──────────────────┐    │
│                                                    │ /safety/         │    │
│                                                    │   validator      │    │
│                                                    │   (validate srv) │    │
│                                                    └──────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Node Descriptions

#### 2.2.1 `/safety/validator`
**Purpose:** Validates trajectories against safety constraints before execution

**Responsibilities:**
- Receive trajectory validation requests from AI/planning layer
- Check trajectories against velocity limits, workspace bounds, and restricted zones
- Issue cryptographically-signed safety certificates for approved trajectories
- Reject unsafe trajectories with detailed reasoning
- Maintain validation cache for performance

**Inputs:**
- `TrajectoryValidationRequest` (service)
- `SafetyLimits` (topic, cached)
- `EmergencyStop` (topic, for state awareness)

**Outputs:**
- `TrajectoryValidationResponse` (service)
- `SafetyCertificate` (topic, published on approval)
- Validation logs

**Timing Requirements:**
- Validation must complete within **< 10ms**
- Certificate validity window: **30 seconds**

#### 2.2.2 `/safety/limits`
**Purpose:** Manages and provides safety limits for all robots

**Responsibilities:**
- Load safety limits from configuration (`config/safety_limits.yaml`)
- Serve safety limits via service calls
- Monitor for configuration changes
- Cache limits for fast access

**Inputs:**
- Configuration file changes
- `GetSafetyLimits` service requests

**Outputs:**
- `SafetyLimits` (topic, published on change)
- `GetSafetyLimits` service responses

#### 2.2.3 `/safety/emergency_stop`
**Purpose:** Central emergency stop coordination

**Responsibilities:**
- Monitor emergency stop triggers from multiple sources
- Execute immediate stop commands
- Coordinate hardware and software cutoffs
- Manage emergency stop state machine
- Handle authorized reset procedures

**Inputs:**
- `TriggerEmergencyStop` service
- `ClearEmergencyStop` service
- Watchdog timeout signals
- Hardware e-stop signals (if available)

**Outputs:**
- `EmergencyStop` topic (published on state change)
- Motor stop commands
- Hardware e-stop triggers

**Timing Requirements:**
- E-stop activation latency: **< 50ms** from trigger to motor cutoff

#### 2.2.4 `/safety/watchdog`
**Purpose:** Monitors system health and triggers failsafe actions

**Responsibilities:**
- Generate 1kHz heartbeat signal
- Monitor critical nodes for responsiveness
- Detect timeout conditions
- Auto-trigger emergency stop on failures
- Log watchdog events

**Inputs:**
- Node status heartbeats
- System health metrics

**Outputs:**
- 1kHz heartbeat topic
- Emergency stop triggers
- Watchdog status topic

**Timing Requirements:**
- Heartbeat frequency: **1kHz (1ms period)**
- Timeout threshold: **50ms**

---

## 3. Timing Requirements

### 3.1 Critical Timing Specifications

| Operation | Target | Maximum | Notes |
|-----------|--------|---------|-------|
| Trajectory Validation | 5ms | 10ms | From request to response |
| Emergency Stop Activation | 20ms | 50ms | From trigger to motor cutoff |
| Watchdog Heartbeat | 1ms | 1ms | Fixed 1kHz frequency |
| Watchdog Timeout | N/A | 50ms | Max time between heartbeats |
| Safety Certificate Validity | 30s | 30s | Fixed window |
| Limit Query Response | 1ms | 5ms | From request to response |

### 3.2 Timing Analysis

```
Trajectory Validation Timeline:
─────────────────────────────────────────────────────────►
│←──── 1ms ────→│←──── 3ms ────→│←──── 1ms ────→│
   Receive/Parse    Validate        Sign/Respond
   Request          Constraints     Certificate

Emergency Stop Timeline:
─────────────────────────────────────────────────────────►
│←──── 5ms ────→│←──── 10ms ───→│←──── 15ms ───→│←─ 20ms ─→│
   Trigger         Software        Hardware        Motors
   Detection       Cutoff          Cutoff          Stopped
```

---

## 4. Hardware/Software Separation

### 4.1 Safety Layers

```
┌─────────────────────────────────────────────────────────────┐
│ LAYER 3: Hardware Safety (Independent)                      │
│ ─────────────────────────────────────                       │
│ • Hardware emergency stop buttons                           │
│ • Motor controller hardware limits                          │
│ • Safety PLC (if equipped)                                  │
│ • Physical barriers and interlocks                          │
│                                                             │
│ Characteristics:                                            │
│ - Operates independently of software                        │
│ - Hardwired logic where possible                            │
│ - Highest reliability, cannot be overridden by software     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ LAYER 2: Software Safety (ROS Safety Nodes)                 │
│ ───────────────────────────────────────────                 │
│ • /safety/emergency_stop (coordinated stop)                 │
│ • /safety/validator (trajectory validation)                 │
│ • /safety/watchdog (health monitoring)                      │
│ • /safety/limits (constraint enforcement)                   │
│                                                             │
│ Characteristics:                                            │
│ - Runs on real-time capable ROS2 executor                   │
│ - Can trigger hardware cutoffs                              │
│ - Independent of AI/planning layer                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ LAYER 1: AI/Planning Safety (Application Layer)             │
│ ───────────────────────────────────────────────             │
│ • Trajectory validation requests                            │
│ • Safety certificate verification                           │
│ • Conservative planning defaults                            │
│                                                             │
│ Characteristics:                                            │
│ - Must request validation before execution                  │
│ - Cannot override safety layer decisions                    │
│ - Provides first line of defense                            │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Separation Principles

1. **Independence:** Each layer can function independently
2. **Redundancy:** Multiple layers check the same constraints
3. **Fail-Safe:** Lower layers take precedence over higher layers
4. **Audit:** All layer interactions are logged

---

## 5. Fail-Safe Design Principles

### 5.1 Fail-Safe States

| Component | Normal State | Failure State | Detection |
|-----------|-------------|---------------|-----------|
| Watchdog | Heartbeating | Trigger E-stop | Timeout > 50ms |
| Validator | Validating | Reject all | Exception/Timeout |
| E-Stop Node | Monitoring | Active E-stop | Internal error |
| Limits Node | Serving limits | Use defaults | Config error |
| AI Layer | Planning | Request validation | Safety cert check |

### 5.2 Fail-Safe Behaviors

1. **Communication Loss:** Any safety-critical communication loss triggers e-stop
2. **Node Crash:** Watchdog detects node absence and triggers e-stop
3. **Validation Timeout:** Trajectories exceeding validation time are rejected
4. **Certificate Expiry:** Expired safety certificates are treated as invalid
5. **Configuration Error:** Missing limits use most conservative defaults

### 5.3 Recovery Procedures

1. **E-Stop Recovery:**
   - Requires manual acknowledgment
   - Authorization verification
   - System health check
   - Gradual restart sequence

2. **Watchdog Recovery:**
   - Automatic after heartbeat resumes
   - Requires operator acknowledgment
   - Logs incident for review

---

## 6. Watchdog Design

### 6.1 Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     WATCHDOG SYSTEM                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   Heartbeat │    │   Timeout   │    │   Action    │     │
│  │   Generator │───►│   Detector  │───►│   Executor  │     │
│  │   (1kHz)    │    │   (50ms)    │    │             │     │
│  └─────────────┘    └─────────────┘    └──────┬──────┘     │
│                                                │            │
│                                                ▼            │
│                                        ┌─────────────┐      │
│                                        │  Emergency  │      │
│                                        │    Stop     │      │
│                                        └─────────────┘      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 Heartbeat Protocol

**Frequency:** 1000 Hz (1ms period)
**Message Format:**
```
WatchdogHeartbeat:
  seq: uint32       # Sequence number (monotonic)
  stamp: time       # Timestamp
  source: string    # Node ID
  status: uint8     # 0=OK, 1=WARNING, 2=ERROR
```

**Timeout Logic:**
- Expected heartbeat every 1ms
- Warning after 10ms without heartbeat
- E-stop triggered after 50ms without heartbeat

### 6.3 Monitored Nodes

| Node | Criticality | Timeout Action |
|------|-------------|----------------|
| /safety/validator | Critical | E-stop + log |
| /safety/emergency_stop | Critical | Hardware e-stop |
| /safety/limits | High | Use cached/default limits |
| /ai/executor | High | Pause execution |
| /ai/planner | Medium | Replan required |

---

## 7. Message Flow Examples

### 7.1 Normal Trajectory Validation Flow

```
┌─────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────┐
│  AI/    │     │   Safety    │     │   Safety    │     │  Robot  │
│ Planner │     │  Validator  │     │   Limits    │     │ Control │
└────┬────┘     └──────┬──────┘     └──────┬──────┘     └────┬────┘
     │                 │                   │                 │
     │ 1. ValidateTrajectory Request       │                 │
     │────────────────►│                   │                 │
     │                 │ 2. GetSafetyLimits│                 │
     │                 │──────────────────►│                 │
     │                 │                   │                 │
     │                 │ 3. SafetyLimits   │                 │
     │                 │◄──────────────────│                 │
     │                 │                   │                 │
     │                 │ 4. Validate constraints             │
     │                 │ (velocity, workspace, zones)        │
     │                 │                   │                 │
     │ 5. ValidationResponse + Certificate │                 │
     │◄────────────────│                   │                 │
     │                 │                   │                 │
     │ 6. Execute with certificate         │                 │
     │─────────────────────────────────────────────────────►│
     │                 │                   │                 │
```

### 7.2 Emergency Stop Flow

```
┌─────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────┐
│ Trigger │     │   Safety    │     │   Hardware  │     │  Robot  │
│ Source  │     │  E-Stop     │     │   Cutoff    │     │ Motors  │
└────┬────┘     └──────┬──────┘     └──────┬──────┘     └────┬────┘
     │                 │                   │                 │
     │ 1. TriggerEStop │                   │                 │
     │ (reason/source) │                   │                 │
     │────────────────►│                   │                 │
     │                 │                   │                 │
     │                 │ 2. Publish        │                 │
     │                 │ EmergencyStop     │                 │
     │                 │───────────────────┼────────►        │
     │                 │                   │                 │
     │                 │ 3. Software cutoff│                 │
     │                 │───────────────────┼────────────────►│
     │                 │                   │                 │
     │                 │ 4. Hardware cutoff│                 │
     │                 │──────────────────►│                 │
     │                 │                   │ 5. Cut power    │
     │                 │                   │────────────────►│
     │                 │                   │                 │
     │                 │ 6. Confirm        │                 │
     │◄────────────────│                   │                 │
     │                 │                   │                 │
```

---

## 8. Integration Points

### 8.1 AI Layer Integration (ENG-1)

**Interface:**
- AI layer must call `ValidateTrajectory` service before execution
- AI layer must verify safety certificate is valid and not expired
- AI layer must subscribe to `EmergencyStop` topic and halt on activation

**Contract:**
```python
# Before executing any trajectory
response = validate_trajectory_client.call(request)
if not response.approved:
    log_error(response.rejection_reason)
    return False

# During execution
if emergency_stop_active:
    halt_immediately()
    return False
```

### 8.2 Motion Planning Integration (ENG-3)

**Interface:**
- Planner must request validation for generated trajectories
- Planner must respect workspace bounds and restricted zones
- Planner should use safety limits as constraints during planning

**Contract:**
```python
# Generate trajectory with safety limits as constraints
trajectory = planner.plan(goal, constraints=safety_limits)

# Validate before returning
response = validator.validate(trajectory)
if not response.approved:
    # Replan with more conservative constraints
    trajectory = replan_with_adjusted_constraints()
```

---

## 9. Configuration

### 9.1 Safety Limits Configuration

See `config/safety_limits.yaml` for complete configuration format.

### 9.2 Runtime Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `validation_timeout_ms` | 10 | Maximum validation time |
| `certificate_validity_sec` | 30 | Certificate lifetime |
| `watchdog_heartbeat_hz` | 1000 | Watchdog frequency |
| `watchdog_timeout_ms` | 50 | Watchdog timeout threshold |
| `estop_activation_ms` | 50 | Max e-stop activation time |

---

## 10. Testing and Validation

See `docs/SAFETY_TEST_PLAN.md` for detailed test scenarios.

---

## 11. References

- [1] ISO 10218-1:2011 - Robots and robotic devices — Safety requirements
- [2] ISO/TS 15066:2016 - Collaborative robots safety
- [3] ROS2 Safety Guidelines
- [4] `config/safety_limits.yaml` - Safety configuration
- [5] `docs/SAFETY_TEST_PLAN.md` - Test plan

---

## 12. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-03-06 | ENG-2 | Initial architecture document |

---

## 13. Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Safety Officer | TBD | ___________ | _____ |
| Lead Engineer | ENG-2 | ___________ | _____ |
| AI Lead | ENG-1 | ___________ | _____ |
| Planning Lead | ENG-3 | ___________ | _____ |
