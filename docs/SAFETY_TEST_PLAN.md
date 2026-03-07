# Safety Test Plan
## Agent ROS Bridge v0.6.1

**Document Version:** 1.0  
**Date:** 2026-03-06  
**Author:** ENG-2 (ROS Safety Engineer)  
**Status:** Draft - Pending Safety Officer Review

---

## 1. Overview

This document defines the initial test scenarios for validating the safety architecture of Agent ROS Bridge v0.6.1. These tests cover critical safety functions including velocity limits, workspace boundaries, emergency stop, watchdog functionality, and trajectory validation.

### 1.1 Test Categories

| Category | Count | Description |
|----------|-------|-------------|
| Limit Enforcement | 3 | Velocity, workspace, and zone limit tests |
| Emergency Systems | 3 | E-stop activation, clearing, and watchdog |
| Validation Logic | 2 | Trajectory validation and certificate tests |
| Integration | 2 | Multi-node and hardware integration tests |

---

## 2. Test Environment

### 2.1 Required Components

- ROS2 Jazzy or Humble
- Agent ROS Bridge v0.6.1 safety nodes
- Simulated robot (TurtleBot3 or custom URDF)
- Test harness with timing measurement capabilities

### 2.2 Test Configuration

```yaml
# test_config.yaml
test_robot_id: "turtlebot_01"
max_linear_velocity: 0.5
max_angular_velocity: 1.0
workspace_bounds:
  - {x: -5.0, y: -5.0}
  - {x: 5.0, y: -5.0}
  - {x: 5.0, y: 5.0}
  - {x: -5.0, y: 5.0}
timing_requirements:
  validation_ms: 10
  estop_activation_ms: 50
  watchdog_timeout_ms: 50
```

---

## 3. Test Scenarios

### Test 1: Velocity Limit Enforcement

**ID:** SAF-001  
**Priority:** Critical  
**Type:** Automated

#### Objective
Verify that the safety validator correctly rejects trajectories exceeding velocity limits.

#### Preconditions
- Safety nodes are running
- Test robot configuration loaded
- Validator service available

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1.1 | Create trajectory with linear velocity 0.3 m/s (within limit) | Trajectory accepted |
| 1.2 | Create trajectory with linear velocity 0.5 m/s (at limit) | Trajectory accepted |
| 1.3 | Create trajectory with linear velocity 0.6 m/s (exceeds limit) | Trajectory rejected |
| 1.4 | Create trajectory with angular velocity 1.0 rad/s (at limit) | Trajectory accepted |
| 1.5 | Create trajectory with angular velocity 1.5 rad/s (exceeds limit) | Trajectory rejected |
| 1.6 | Measure validation time for each case | All validations < 10ms |

#### Pass Criteria
- All within-limit trajectories are approved
- All exceeding-limit trajectories are rejected with reason "VELOCITY_LIMIT_EXCEEDED"
- Validation completes within 10ms

#### Failure Criteria
- Any unsafe trajectory is approved
- Validation exceeds 10ms

---

### Test 2: Workspace Boundary Violation

**ID:** SAF-002  
**Priority:** Critical  
**Type:** Automated

#### Objective
Verify that trajectories violating workspace boundaries are rejected.

#### Preconditions
- Safety nodes are running
- Workspace bounds configured as 10m x 10m square centered at origin

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 2.1 | Create trajectory entirely within workspace | Trajectory accepted |
| 2.2 | Create trajectory ending at workspace boundary | Trajectory accepted |
| 2.3 | Create trajectory with single point outside boundary | Trajectory rejected |
| 2.4 | Create trajectory crossing workspace boundary | Trajectory rejected |
| 2.5 | Create trajectory with all points outside workspace | Trajectory rejected |
| 2.6 | Test with polygon workspace (non-rectangular) | Same boundary rules apply |

#### Pass Criteria
- All boundary-violating trajectories are rejected with reason "WORKSPACE_VIOLATION"
- Boundary-edge trajectories are handled correctly
- Rejection includes which point violated boundary

#### Failure Criteria
- Trajectory with points outside workspace is approved
- No clear rejection reason provided

---

### Test 3: Emergency Stop Activation

**ID:** SAF-003  
**Priority:** Critical  
**Type:** Automated + Manual

#### Objective
Verify emergency stop activates correctly from all sources and achieves required timing.

#### Preconditions
- All safety nodes running
- Robot in motion (simulated)
- Timing measurement enabled

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 3.1 | Trigger e-stop via service call with source="HUMAN" | E-stop activates within 50ms |
| 3.2 | Verify EmergencyStop message published | Message contains correct source and reason |
| 3.3 | Verify motor stop command sent | Zero velocity commanded |
| 3.4 | Trigger e-stop via watchdog timeout | E-stop activates within 50ms |
| 3.5 | Trigger e-stop via validator critical failure | E-stop activates within 50ms |
| 3.6 | Measure total activation latency | All activations < 50ms |

#### Pass Criteria
- All e-stop triggers result in stopped state within 50ms
- EmergencyStop message correctly identifies source
- Motor commands are zeroed

#### Failure Criteria
- E-stop activation exceeds 50ms
- Robot continues moving after e-stop
- Incorrect source identification

---

### Test 4: Watchdog Timeout

**ID:** SAF-004  
**Priority:** Critical  
**Type:** Automated

#### Objective
Verify watchdog correctly detects node failures and triggers emergency stop.

#### Preconditions
- Watchdog node running
- Test node publishing heartbeats
- Emergency stop node monitoring

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 4.1 | Start test node with 1kHz heartbeat | Watchdog reports healthy |
| 4.2 | Reduce heartbeat to 100Hz (10ms period) | Watchdog reports healthy |
| 4.3 | Stop heartbeat for 25ms | Watchdog warning logged |
| 4.4 | Stop heartbeat for 50ms | Emergency stop triggered |
| 4.5 | Resume heartbeat | E-stop remains active (manual reset required) |
| 4.6 | Verify e-stop reason | "WATCHDOG_TIMEOUT" with node ID |

#### Pass Criteria
- Warning logged at 10ms without heartbeat
- E-stop triggered at 50ms without heartbeat
- E-stop persists after heartbeat resumes

#### Failure Criteria
- E-stop not triggered after 50ms
- False positive e-stop with valid heartbeat
- Auto-clear without authorization

---

### Test 5: Invalid Trajectory Rejection

**ID:** SAF-005  
**Priority:** High  
**Type:** Automated

#### Objective
Verify validator correctly identifies and rejects various types of invalid trajectories.

#### Preconditions
- Validator node running
- Safety limits configured

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 5.1 | Submit empty trajectory (zero points) | Rejected: "EMPTY_TRAJECTORY" |
| 5.2 | Submit trajectory with single point | Rejected: "INSUFFICIENT_POINTS" |
| 5.3 | Submit trajectory with NaN values | Rejected: "INVALID_DATA" |
| 5.4 | Submit trajectory with infinite values | Rejected: "INVALID_DATA" |
| 5.5 | Submit trajectory with non-monotonic timestamps | Rejected: "INVALID_TIMING" |
| 5.6 | Submit trajectory entering restricted zone | Rejected: "RESTRICTED_ZONE_VIOLATION" |
| 5.7 | Submit trajectory with excessive acceleration | Rejected: "ACCELERATION_LIMIT_EXCEEDED" |

#### Pass Criteria
- All invalid trajectories rejected with specific reasons
- Valid trajectories accepted
- No crashes or exceptions

#### Failure Criteria
- Invalid trajectory accepted
- Vague or missing rejection reason
- System crash on invalid input

---

### Test 6: Safety Certificate Validity

**ID:** SAF-006  
**Priority:** High  
**Type:** Automated

#### Objective
Verify safety certificates have correct validity period and are properly invalidated.

#### Preconditions
- Validator node running
- Clock synchronization verified

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 6.1 | Validate trajectory and receive certificate | Certificate issued with 30s expiry |
| 6.2 | Verify certificate immediately | Valid |
| 6.3 | Wait 15 seconds, verify certificate | Valid |
| 6.4 | Wait 30 seconds, verify certificate | Expired |
| 6.5 | Verify certificate hash matches trajectory | Hash verification passes |
| 6.6 | Modify trajectory, verify certificate fails | Hash verification fails |
| 6.7 | Check certificate includes all constraint checks | All constraints listed |

#### Pass Criteria
- Certificates valid for exactly 30 seconds
- Hash correctly binds certificate to trajectory
- Expired certificates rejected

#### Failure Criteria
- Certificate valid longer than 30 seconds
- Hash mismatch not detected
- Wrong constraint check list

---

### Test 7: Emergency Stop Reset Authorization

**ID:** SAF-007  
**Priority:** High  
**Type:** Automated + Manual

#### Objective
Verify emergency stop can only be cleared with proper authorization.

#### Preconditions
- Emergency stop active
- Authorization system configured

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 7.1 | Attempt clear without authorization | Rejected |
| 7.2 | Attempt clear with invalid authorization | Rejected |
| 7.3 | Attempt clear with valid authorization | Success |
| 7.4 | Verify system state after clear | E-stop inactive, ready for operation |
| 7.5 | Trigger e-stop again, verify re-clearable | Can be cleared again |
| 7.6 | Log all authorization attempts | All attempts logged with operator ID |

#### Pass Criteria
- Only valid authorization clears e-stop
- System returns to operational state
- All attempts audited

#### Failure Criteria
- E-stop clears without authorization
- System stuck in e-stop state
- Missing audit logs

---

### Test 8: Multi-Node Failure Handling

**ID:** SAF-008  
**Priority:** High  
**Type:** Automated

#### Objective
Verify system handles simultaneous failures in multiple safety nodes.

#### Preconditions
- All safety nodes running
- Fault injection capability

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 8.1 | Kill validator node while validating | E-stop triggered, error logged |
| 8.2 | Kill limits node during query | Uses cached/default limits |
| 8.3 | Kill emergency_stop node | Hardware e-stop activated |
| 8.4 | Kill watchdog node | Other nodes detect loss, trigger e-stop |
| 8.5 | Restart all nodes | System recovers to safe state |
| 8.6 | Verify no memory leaks after restarts | Memory usage stable |

#### Pass Criteria
- Any node failure results in safe state
- System recovers gracefully after restart
- No resource leaks

#### Failure Criteria
- Node failure leaves system in unsafe state
- Recovery fails
- Memory leaks detected

---

### Test 9: Hardware Emergency Stop Integration

**ID:** SAF-009  
**Priority:** Medium  
**Type:** Manual (Hardware Required)

#### Objective
Verify software safety layer integrates correctly with hardware emergency stop.

#### Preconditions
- Robot with hardware e-stop capability
- Hardware e-stop button accessible
- Oscilloscope or timing measurement equipment

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 9.1 | Press hardware e-stop button | Hardware cutoff within 10ms |
| 9.2 | Verify software detects hardware e-stop | Software e-stop activated |
| 9.3 | Release hardware e-stop | E-stop remains active (latching) |
| 9.4 | Clear software e-stop with auth | System ready |
| 9.5 | Trigger software e-stop | Hardware cutoff activated |
| 9.6 | Measure hardware cutoff latency | < 10ms from trigger to power cutoff |

#### Pass Criteria
- Hardware e-stop triggers software e-stop
- Software e-stop can trigger hardware cutoff
- Hardware cutoff < 10ms

#### Failure Criteria
- Hardware/software e-stop not synchronized
- Hardware cutoff > 10ms
- Non-latching behavior

---

### Test 10: Load and Performance Testing

**ID:** SAF-010  
**Priority:** Medium  
**Type:** Automated

#### Objective
Verify safety system maintains timing requirements under load.

#### Preconditions
- All safety nodes running
- Load generation capability

#### Test Steps

| Step | Action | Expected Result |
|------|--------|-----------------|
| 10.1 | Run 1000 validations sequentially | All complete < 10ms |
| 10.2 | Run 100 validations concurrently | All complete < 10ms |
| 10.3 | Simulate 10 robots simultaneously | Each validated correctly |
| 10.4 | Run continuous validation for 1 hour | No memory leaks, timing stable |
| 10.5 | Measure CPU usage under load | < 50% of one core |
| 10.6 | Test with maximum complexity trajectory | Validation still < 10ms |

#### Pass Criteria
- All timing requirements met under load
- No degradation over extended operation
- Memory usage stable

#### Failure Criteria
- Validation time > 10ms under load
- Memory growth over time
- CPU usage excessive

---

## 4. Test Execution Schedule

| Phase | Tests | Duration | Responsible |
|-------|-------|----------|-------------|
| Phase 1 | SAF-001, SAF-002, SAF-005 | 1 day | ENG-2 |
| Phase 2 | SAF-003, SAF-004, SAF-007 | 1 day | ENG-2 |
| Phase 3 | SAF-006, SAF-008 | 1 day | ENG-2 |
| Phase 4 | SAF-009 (Hardware) | 1 day | ENG-2 + Safety Officer |
| Phase 5 | SAF-010 (Load) | 2 days | ENG-2 |
| Phase 6 | Regression + Integration | 2 days | All ENG |

**Total Estimated Duration:** 8 days

---

## 5. Success Criteria

### 5.1 Critical Tests (Must Pass)
- SAF-001: Velocity Limit Enforcement
- SAF-002: Workspace Boundary Violation
- SAF-003: Emergency Stop Activation
- SAF-004: Watchdog Timeout

### 5.2 High Priority Tests (Must Pass)
- SAF-005: Invalid Trajectory Rejection
- SAF-006: Safety Certificate Validity
- SAF-007: Emergency Stop Reset Authorization
- SAF-008: Multi-Node Failure Handling

### 5.3 Medium Priority Tests (Should Pass)
- SAF-009: Hardware Emergency Stop Integration
- SAF-010: Load and Performance Testing

---

## 6. Test Reporting

### 6.1 Test Report Template

```
Test Report: [Test ID]
Date: [Date]
Tester: [Name]
Result: [PASS/FAIL]

Execution Summary:
- Steps executed: [N]/[N]
- Pass criteria met: [Y/N]
- Timing requirements met: [Y/N]

Issues Found:
[Description]

Logs:
[Relevant log excerpts]

Recommendations:
[If any]
```

### 6.2 Defect Severity

| Severity | Definition | Example |
|----------|------------|---------|
| Critical | Safety violation possible | Unsafe trajectory approved |
| High | Timing requirement missed | Validation > 10ms |
| Medium | Functionality impaired | Incorrect error message |
| Low | Cosmetic/minor | Log formatting issue |

---

## 7. Test Automation

### 7.1 Automated Test Framework

```python
# test_safety_framework.py
class SafetyTestFramework:
    def setup(self):
        self.validator = ValidatorClient()
        self.estop = EmergencyStopClient()
        self.watchdog = WatchdogMonitor()
        self.timing = TimingAnalyzer()
    
    def test_velocity_limits(self):
        for velocity in [0.3, 0.5, 0.6]:
            traj = create_trajectory(linear_vel=velocity)
            start = time.now()
            result = self.validator.validate(traj)
            elapsed = time.now() - start
            
            if velocity <= 0.5:
                assert result.approved == True
            else:
                assert result.approved == False
            assert elapsed < 10ms
```

### 7.2 Continuous Integration

Tests should be run:
- On every commit to safety-related code
- Nightly regression suite
- Before each release

---

## 8. References

- [1] `docs/SAFETY_ARCHITECTURE_V1.md` - Safety architecture
- [2] `config/safety_limits.yaml` - Safety configuration
- [3] ISO 10218-1:2011 - Robot safety requirements
- [4] `msgs/` - Message definitions

---

## 9. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-03-06 | ENG-2 | Initial test plan |

---

## 10. Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Safety Officer | TBD | ___________ | _____ |
| Test Engineer | ENG-2 | ___________ | _____ |
