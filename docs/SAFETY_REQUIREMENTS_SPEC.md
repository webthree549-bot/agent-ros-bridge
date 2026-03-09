# Safety Requirements Specification (SRS) v1.0

**Document ID:** SRS-001  
**Version:** 1.0  
**Date:** 2026-03-06  
**Author:** Safety Officer  
**Status:** APPROVED  
**Classification:** SAFETY-CRITICAL

---

## 1. Executive Summary

This Safety Requirements Specification (SRS) defines the safety requirements for the Agent ROS Bridge v0.6.1-v0.7.0 project. This document establishes the safety architecture, hardware-enforced limits, emergency stop requirements, and validation criteria necessary to ensure safe operation of AI-controlled robotic systems.

**Safety Authority:** This document carries VETO authority. Any development that violates these requirements MUST be stopped.

---

## 2. Scope and Applicability

### 2.1 Scope
This SRS applies to:
- All software components in the Agent ROS Bridge system
- Hardware safety interfaces
- AI/ML components with safety implications
- Simulation and testing environments
- Deployment and operational procedures

### 2.2 Applicability by Version
| Version | Applicability |
|---------|---------------|
| v0.6.1 (Foundation) | All Sections 3-5, 7-9 |
| v0.6.2 (Assisted AI) | All sections |
| v0.6.3 (Supervised) | All sections |
| v0.7.0 (Production) | All sections + Section 10 |

---

## 3. Safety Principles (Non-Negotiable)

### 3.1 Principle 1: Independence
**Requirement:** The safety layer MUST be independent from the AI layer.

**Rationale:** AI systems are non-deterministic and may produce unexpected outputs. The safety layer must function correctly regardless of AI behavior.

**Implementation:**
- Safety validator runs as separate process with higher priority
- Safety layer has dedicated CPU core (if available)
- AI layer cannot modify safety configuration
- Safety layer can operate without AI layer running

**Verification:** Demonstrate that disabling AI layer does not affect safety layer operation.

### 3.2 Principle 2: Hardware Enforcement
**Requirement:** Software cannot override safety limits enforced by hardware.

**Rationale:** Software bugs can compromise safety. Hardware-enforced limits provide ultimate protection.

**Implementation:**
- Safety-rated PLC or safety controller for critical limits
- Hardware interlocks for emergency stop
- Hardware velocity/position limits on motor controllers
- Independent safety sensors (not shared with AI)

**Verification:** Attempt to override hardware limits via software - must fail.

### 3.3 Principle 3: Fail-Safe
**Requirement:** Any failure defaults to safe state.

**Rationale:** Unknown failure modes should not lead to unsafe operation.

**Implementation:**
- Loss of communication → Stop motion
- Sensor failure → Conservative safe mode
- Software crash → Hardware e-stop triggers
- Power loss → Brakes engage

**Verification:** Inject failures at each component - verify safe state reached.

### 3.4 Principle 4: Determinism
**Requirement:** Safety validation must be deterministic with <10ms response time.

**Rationale:** Safety decisions cannot depend on non-deterministic AI or variable timing.

**Implementation:**
- Rule-based validation (no ML in safety path)
- Fixed execution time algorithms
- Worst-case execution time (WCET) analysis
- Real-time operating system or scheduler

**Verification:** Statistical timing analysis showing 99.99% of responses <10ms.

### 3.5 Principle 5: Verification
**Requirement:** All safety claims must be provable.

**Rationale:** Safety cannot be assumed; it must be demonstrated.

**Implementation:**
- Formal methods where applicable
- Comprehensive test coverage
- Independent safety audit
- Traceability from requirements to tests

**Verification:** Third-party safety audit report.

---

## 4. Hardware-Enforced Limits Requirements

### 4.1 Velocity Limits

#### REQ-HW-001: Maximum Linear Velocity
**Description:** Hardware must enforce maximum linear velocity.

**Requirement:**
- Mobile robots: ≤ 1.5 m/s (indoor), ≤ 3.0 m/s (outdoor)
- Manipulator end-effector: ≤ 2.0 m/s
- Software request exceeding limit → Hardware rejects

**Hardware Implementation:**
- Motor controller with configurable velocity limit
- Independent encoder feedback
- Hard limit in motor driver firmware

**Verification Method:**
- Command velocity exceeding limit
- Verify hardware enforces limit
- Verify software receives rejection

#### REQ-HW-002: Maximum Angular Velocity
**Description:** Hardware must enforce maximum angular velocity.

**Requirement:**
- Mobile base: ≤ 1.0 rad/s
- Manipulator joints: ≤ 2.0 rad/s (varies by joint)

**Hardware Implementation:**
- Same as REQ-HW-001

**Verification Method:**
- Same as REQ-HW-001

### 4.2 Position/Joint Limits

#### REQ-HW-003: Joint Position Limits
**Description:** Hardware must enforce joint position limits.

**Requirement:**
- Each joint has hardware-enforced min/max position
- Approaching limit triggers soft deceleration
- Exceeding limit triggers hard stop

**Hardware Implementation:**
- Limit switches or hard stops
- Encoder position monitoring in motor controller
- Torque limiting near boundaries

**Verification Method:**
- Command motion beyond joint limit
- Verify soft deceleration activates
- Verify hard stop prevents overtravel

#### REQ-HW-004: Workspace Boundaries
**Description:** Hardware must enforce workspace boundaries.

**Requirement:**
- Define 3D workspace volume
- End-effector cannot exit workspace
- Mobile robot cannot exit defined area (if applicable)

**Hardware Implementation:**
- Virtual walls in safety controller
- Physical barriers where possible
- Geofencing for mobile robots

**Verification Method:**
- Command motion outside workspace
- Verify hardware prevents exit

### 4.3 Force/Torque Limits

#### REQ-HW-005: Maximum Joint Torque
**Description:** Hardware must limit joint torque.

**Requirement:**
- Each joint has maximum torque limit
- Exceeding limit triggers stop
- Limit set below mechanical damage threshold

**Hardware Implementation:**
- Current limiting in motor drivers
- Torque sensors with hardware shutdown

**Verification Method:**
- Apply external force to joint
- Verify torque limiting activates

#### REQ-HW-006: Collision Detection
**Description:** Hardware must detect and respond to collisions.

**Requirement:**
- Collision detected within 10ms
- Motion stopped within 50ms of detection
- Force threshold configurable per application

**Hardware Implementation:**
- Force/torque sensors with hardware threshold
- Current spike detection
- Accelerometer-based collision detection

**Verification Method:**
- Simulate collision with test object
- Measure detection and stop time

---

## 5. Emergency Stop Requirements

### 5.1 Response Time

#### REQ-ESTOP-001: Emergency Stop Latency
**Description:** Emergency stop must activate within 50ms of trigger.

**Requirement:**
- Physical e-stop button press → Motion stop: <50ms
- Software e-stop command → Motion stop: <50ms
- Safety violation detected → Motion stop: <50ms

**Timing Breakdown:**
- Detection: <10ms
- Signal propagation: <20ms
- Hardware response: <20ms
- **Total: <50ms (worst case)**

**Verification Method:**
- Oscilloscope measurement of e-stop signal
- High-speed camera verification of motion stop
- Statistical analysis (n≥1000 samples)

### 5.2 E-Stop Types

#### REQ-ESTOP-002: Physical Emergency Stop
**Description:** Physical e-stop buttons must be provided.

**Requirement:**
- Red mushroom buttons at operator stations
- Wireless e-stop for mobile operation
- E-stop on robot itself (if accessible)
- Category 0 stop (immediate power removal) or Category 1 (controlled stop)

**Hardware Implementation:**
- Safety-rated e-stop buttons (ISO 13850)
- Dual-channel safety relays
- Hardwired to motor power

**Verification Method:**
- Press e-stop, verify immediate stop
- Test all e-stop locations
- Verify latching (requires reset)

#### REQ-ESTOP-003: Software Emergency Stop
**Description:** Software must be able to trigger e-stop.

**Requirement:**
- `/safety/emergency_stop` topic triggers e-stop
- Safety validator can trigger e-stop
- Watchdog can trigger e-stop
- AI layer CANNOT override e-stop

**Implementation:**
- Software command to safety controller
- Safety controller validates and executes
- Log all software e-stop triggers

**Verification Method:**
- Publish to e-stop topic
- Verify e-stop activates
- Verify AI cannot prevent e-stop

#### REQ-ESTOP-004: Automatic Emergency Stop
**Description:** System must auto-trigger e-stop on critical failures.

**Requirement:**
- Safety validation failure → E-stop
- Watchdog timeout → E-stop
- Communication loss >100ms → E-stop
- Sensor failure affecting safety → E-stop

**Implementation:**
- Automatic trigger logic in safety controller
- No software override possible

**Verification Method:**
- Induce each failure mode
- Verify automatic e-stop

### 5.3 E-Stop Reset

#### REQ-ESTOP-005: Reset Requirements
**Description:** E-stop reset must require deliberate human action.

**Requirement:**
- E-stop latches until manually reset
- Reset requires two-step process (release + restart)
- System checks safety before allowing reset
- Cannot reset while hazard present

**Implementation:**
- Twist-to-release e-stop buttons
- Software reset command after physical release
- Safety checks before motion enabled

**Verification Method:**
- Trigger e-stop
- Attempt automatic reset - must fail
- Perform proper reset procedure - must succeed

---

## 6. Safety Validator Requirements

### 6.1 Response Time

#### REQ-VAL-001: Validation Response Time
**Description:** Safety validator must respond within 10ms.

**Requirement:**
- Request received → Response sent: <10ms (99.99%)
- Worst case execution time: <10ms (100%)
- Measured under full system load

**Implementation:**
- Real-time thread or process
- Fixed execution time algorithms
- No dynamic memory allocation in hot path
- WCET analysis and testing

**Verification Method:**
- Statistical timing analysis (n≥10000)
- Load testing with maximum concurrent requests
- Formal WCET analysis if possible

### 6.2 Validation Functions

#### REQ-VAL-002: Trajectory Validation
**Description:** Validator must check trajectories for safety.

**Requirement:**
- Check velocity limits at all points
- Check acceleration limits
- Check joint position limits
- Check workspace boundaries
- Check for collisions with known obstacles

**Implementation:**
- Forward kinematics calculation
- Interpolation between waypoints
- Conservative bounds checking

**Verification Method:**
- Submit valid trajectory - must pass
- Submit invalid trajectory - must reject with reason

#### REQ-VAL-003: Safety Certificate Generation
**Description:** Validator must generate safety certificates.

**Requirement:**
- Certificate contains validation ID
- Certificate lists all constraints checked
- Certificate has expiration time (≤30 seconds)
- Certificate is cryptographically signed (optional v0.7.0)

**Implementation:**
- Structured certificate format
- Timestamp and validation ID
- List of passed checks

**Verification Method:**
- Validate trajectory
- Verify certificate format and contents
- Verify expiration enforced

#### REQ-VAL-004: Constraint Updates
**Description:** Validator must support dynamic constraint updates.

**Requirement:**
- Constraints can be updated at runtime
- Updates take effect within 100ms
- Invalid constraint updates rejected
- Constraint history logged

**Implementation:**
- Parameter or service interface
- Validation of new constraints
- Atomic update mechanism

**Verification Method:**
- Update constraints
- Verify new constraints enforced
- Verify invalid updates rejected

### 6.3 Independence Requirements

#### REQ-VAL-005: AI Independence
**Description:** Validator must not depend on AI layer.

**Requirement:**
- Validator functions without AI running
- AI cannot modify validator logic
- AI cannot bypass validation
- Validator uses independent sensors where possible

**Implementation:**
- Separate process/node
- No shared memory with AI
- Independent configuration

**Verification Method:**
- Stop AI layer
- Verify validator continues operating
- Attempt to bypass validation via AI - must fail

---

## 7. Watchdog Requirements

### 7.1 Heartbeat Monitoring

#### REQ-WD-001: Heartbeat Frequency
**Description:** Watchdog must monitor at 1kHz (1ms period).

**Requirement:**
- Monitor all critical nodes at 1kHz
- Detect missed heartbeat within 2ms
- Trigger e-stop after 3 consecutive missed heartbeats (3ms)

**Implementation:**
- High-priority monitoring thread
- Hardware timer if available
- Circular buffer for heartbeat tracking

**Verification Method:**
- Measure actual monitoring frequency
- Verify detection time for missed heartbeat
- Verify e-stop trigger timing

#### REQ-WD-002: Monitored Nodes
**Description:** Watchdog must monitor all safety-critical nodes.

**Requirement:**
- /safety/validator
- /safety/limits
- /ai/execution_monitor
- /controller/server
- /hardware_interface

**Implementation:**
- Registration mechanism for critical nodes
- Automatic detection of unregistered critical nodes

**Verification Method:**
- Kill each monitored node
- Verify watchdog detects and triggers e-stop

### 7.2 Health Monitoring

#### REQ-WD-003: Health Checks
**Description:** Watchdog must perform health checks beyond heartbeat.

**Requirement:**
- Monitor CPU temperature
- Monitor memory usage
- Monitor communication latency
- Monitor sensor data freshness

**Implementation:**
- System health polling
- Threshold-based alerts
- Graceful degradation before e-stop

**Verification Method:**
- Simulate high temperature
- Verify warning then e-stop

---

## 8. Sensor Safety Requirements

### 8.1 Sensor Redundancy

#### REQ-SENS-001: Safety-Critical Sensor Redundancy
**Description:** Safety-critical sensors must have redundancy.

**Requirement:**
- Position feedback: Minimum 2 independent sources
- Emergency stop circuit: Dual-channel
- Collision detection: Multiple modalities (force + current + accel)

**Implementation:**
- Dual encoders on critical joints
- Multiple sensor types for collision detection
- Voting logic for discrepancies

**Verification Method:**
- Fail one sensor
- Verify system continues safely
- Verify discrepancy detection

### 8.2 Sensor Validation

#### REQ-SENS-002: Sensor Data Validation
**Description:** Sensor data must be validated before use.

**Requirement:**
- Range checking (reject impossible values)
- Rate limiting (reject sudden jumps)
- Cross-validation between redundant sensors
- Timestamp validation (reject stale data)

**Implementation:**
- Input validation filters
- Median filters for noise
- Outlier detection

**Verification Method:**
- Inject invalid sensor data
- Verify rejection and safe response

---

## 9. Software Safety Requirements

### 9.1 Safety-Critical Code

#### REQ-SW-001: Safety Code Standards
**Description:** Safety-critical code must meet coding standards.

**Requirement:**
- MISRA C++ or equivalent for C++ code
- No dynamic memory allocation in safety paths
- No recursion in safety paths
- Bounded loops only
- Defensive programming (check all inputs)

**Implementation:**
- Static analysis tools
- Code reviews with safety focus
- Automated compliance checking

**Verification Method:**
- Static analysis report
- Code review checklist

#### REQ-SW-002: Safety Testing Coverage
**Description:** Safety-critical code must have 100% test coverage.

**Requirement:**
- 100% statement coverage
- 100% branch coverage
- MC/DC coverage for critical decisions
- All safety requirements have test cases

**Implementation:**
- Unit tests for all safety functions
- Hardware-in-the-loop testing
- Fault injection testing

**Verification Method:**
- Coverage reports
- Test execution reports

### 9.2 AI Safety Requirements

#### REQ-AI-001: AI Output Validation
**Description:** All AI outputs must be validated before execution.

**Requirement:**
- Schema validation of AI outputs
- Range checking of numeric values
- Semantic validation where possible
- Confidence threshold enforcement

**Implementation:**
- Deterministic validation layer
- Rejection of invalid outputs
- Fallback to safe behavior

**Verification Method:**
- Inject invalid AI outputs
- Verify validation catches all errors

#### REQ-AI-002: AI Timeout
**Description:** AI operations must have strict timeouts.

**Requirement:**
- Intent parsing: <1000ms
- Motion planning: <5000ms
- Context resolution: <500ms
- Timeout → Fallback to rule-based or human request

**Implementation:**
- Async operations with timeout
- Cancellation support
- Fallback mechanisms

**Verification Method:**
- Induce AI delays
- Verify timeout and fallback

---

## 10. Compliance Requirements

### 10.1 Standards Compliance

#### REQ-COMP-001: ISO 10218 Compliance
**Description:** System must comply with ISO 10218-1/2.

**Requirement:**
- Industrial robot safety standards
- Collaborative robot requirements (if applicable)
- Documentation and marking requirements

**Implementation:**
- Reference ISO 10218 in design
- Compliance checklist
- Third-party assessment

**Verification Method:**
- ISO 10218 compliance audit

#### REQ-COMP-002: Functional Safety
**Description:** System should achieve SIL 2 or PL d.

**Requirement:**
- Safety Integrity Level 2 (IEC 61508)
- Performance Level d (ISO 13849)
- Documented safety function architecture

**Implementation:**
- Safety function analysis
- Diagnostic coverage requirements
- Mean time to dangerous failure calculations

**Verification Method:**
- Functional safety assessment

---

## 11. Safety Gates

### 11.1 Gate 1: Foundation Safety (End of v0.6.1)

**Criteria:**
- [ ] All hardware limits implemented and tested
- [ ] Emergency stop <50ms verified
- [ ] Safety validator <10ms verified
- [ ] Watchdog 1kHz operational
- [ ] 50+ safety scenarios passing
- [ ] Zero safety violations in simulation
- [ ] Safety documentation complete

**Authority:** Safety Officer has VETO power

### 11.2 Gate 2: Assisted AI Safety (End of v0.6.2)

**Criteria:**
- [ ] AI validation layer operational
- [ ] Human confirmation working
- [ ] Safety layer unchanged from Gate 1
- [ ] 100+ safety scenarios passing
- [ ] Zero safety incidents

**Authority:** Safety Officer has VETO power

### 11.3 Gate 3: Supervised Autonomy Safety (End of v0.6.3)

**Criteria:**
- [ ] Shadow mode validation complete
- [ ] 200+ hours shadow mode without incidents
- [ ] 10,000+ simulation hours
- [ ] All edge cases documented
- [ ] Zero safety incidents

**Authority:** Safety Officer has VETO power

### 11.4 Gate 4: Production Readiness (Before v0.7.0)

**Criteria:**
- [ ] 1000+ hours cumulative shadow mode
- [ ] External safety audit passed
- [ ] Regulatory compliance review passed
- [ ] Insurance approval obtained
- [ ] Incident response procedures tested

**Authority:** Safety Officer has VETO power

---

## 12. Traceability Matrix

| Requirement | Design Element | Test Case | Status |
|-------------|----------------|-----------|--------|
| REQ-HW-001 | Motor controller config | TEST-HW-001 | Not Started |
| REQ-HW-002 | Motor controller config | TEST-HW-002 | Not Started |
| REQ-HW-003 | Limit switches | TEST-HW-003 | Not Started |
| REQ-HW-004 | Safety controller | TEST-HW-004 | Not Started |
| REQ-HW-005 | Current limiting | TEST-HW-005 | Not Started |
| REQ-HW-006 | Force sensors | TEST-HW-006 | Not Started |
| REQ-ESTOP-001 | E-stop circuit | TEST-ESTOP-001 | Not Started |
| REQ-ESTOP-002 | E-stop buttons | TEST-ESTOP-002 | Not Started |
| REQ-ESTOP-003 | E-stop topic | TEST-ESTOP-003 | Not Started |
| REQ-ESTOP-004 | Auto-trigger logic | TEST-ESTOP-004 | Not Started |
| REQ-ESTOP-005 | Reset procedure | TEST-ESTOP-005 | Not Started |
| REQ-VAL-001 | Validator timing | TEST-VAL-001 | Not Started |
| REQ-VAL-002 | Trajectory checking | TEST-VAL-002 | Not Started |
| REQ-VAL-003 | Certificate format | TEST-VAL-003 | Not Started |
| REQ-VAL-004 | Constraint service | TEST-VAL-004 | Not Started |
| REQ-VAL-005 | Process separation | TEST-VAL-005 | Not Started |
| REQ-WD-001 | Watchdog timer | TEST-WD-001 | Not Started |
| REQ-WD-002 | Node registration | TEST-WD-002 | Not Started |
| REQ-WD-003 | Health polling | TEST-WD-003 | Not Started |
| REQ-SENS-001 | Dual encoders | TEST-SENS-001 | Not Started |
| REQ-SENS-002 | Input validation | TEST-SENS-002 | Not Started |
| REQ-SW-001 | Static analysis | TEST-SW-001 | Not Started |
| REQ-SW-002 | Coverage reports | TEST-SW-002 | Not Started |
| REQ-AI-001 | Output validation | TEST-AI-001 | Not Started |
| REQ-AI-002 | Timeout handling | TEST-AI-002 | Not Started |

---

## 13. Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Safety Officer | (Subagent) | Digital | 2026-03-06 |
| Engineering Lead | ENG-2 (ROS Safety) | Pending | - |
| Project Manager | Pending | Pending | - |

---

## 14. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-03-06 | Safety Officer | Initial release |

---

**END OF DOCUMENT**

*This document is classified SAFETY-CRITICAL. Changes require Safety Officer approval.*
