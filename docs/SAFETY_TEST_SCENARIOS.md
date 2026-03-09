# Safety Test Scenarios (STS) v1.0

**Document ID:** STS-001  
**Version:** 1.0  
**Date:** 2026-03-06  
**Author:** Safety Officer  
**Status:** APPROVED  
**Classification:** SAFETY-CRITICAL

---

## 1. Executive Summary

This document defines 52 comprehensive safety test scenarios for the Agent ROS Bridge v0.6.1-v0.7.0 project.

**Test Coverage:**
- Category 1: Human Safety (10 scenarios)
- Category 2: Sensor Failures (10 scenarios)
- Category 3: Unsafe Commands (10 scenarios)
- Category 4: System Failures (10 scenarios)
- Category 5: Edge Cases (12 scenarios)

**Total: 52 Test Scenarios**

---

## 2. Category 1: Human Safety (10 Scenarios)

### TEST-001: Human Suddenly Appears in Robot Path
- **Trigger:** Human steps into robot's planned trajectory while robot is in motion
- **Expected Response:** Human detection within 50ms; immediate deceleration; motion stopped before entering safety zone; safety certificate revoked; system enters SAFE_HOLD
- **Validation:** Inject simulated human; measure detection-to-stop latency; verify stop distance < 0.5m at 1.0 m/s
- **Severity:** CRITICAL

### TEST-002: Human Enters Restricted Zone
- **Trigger:** Human crosses boundary into predefined restricted zone
- **Expected Response:** Zone violation detected within 100ms; all robots in zone stop; warning issued; manual reset required
- **Validation:** Define test zone; simulate entry; verify all robots stop; verify warning signals
- **Severity:** HIGH

### TEST-003: Human Approaches Robot from Behind
- **Trigger:** Human enters robot's rear blind spot
- **Expected Response:** Rear sensors detect within 200ms; backward motion prevented; 360° awareness mode activated
- **Validation:** Position human in rear field; command backward motion; verify motion blocked
- **Severity:** HIGH

### TEST-004: Multiple Humans in Workspace
- **Trigger:** 3+ humans simultaneously enter workspace
- **Expected Response:** All humans tracked individually; robot operates at 50% speed; conservative planning; no motion in critical zones
- **Validation:** Spawn 5 humans; verify tracking; verify speed reduction; test motion blocking
- **Severity:** HIGH

### TEST-005: Human Falls Near Robot
- **Trigger:** Human falls within 2m of robot
- **Expected Response:** Fall detected; immediate e-stop; event logged; robot stopped until operator override
- **Validation:** Simulate fall; measure detection-to-stop time; verify e-stop activated
- **Severity:** CRITICAL

### TEST-006: Human Touches Robot During Operation
- **Trigger:** Physical contact with moving robot
- **Expected Response:** Contact detected via F/T sensors; stop within 50ms; compliant mode activated
- **Validation:** Apply test force; measure contact-to-stop latency; verify compliant mode
- **Severity:** CRITICAL

### TEST-007: Human Blocks Emergency Exit
- **Trigger:** Human stands in emergency exit path
- **Expected Response:** Blockage detected; alternative route calculated; robot clears path if blocking
- **Validation:** Define exit zone; position human; verify detection and alternative routing
- **Severity:** MEDIUM

### TEST-008: Human in Robot Blind Spot
- **Trigger:** Human in sensor coverage gap
- **Expected Response:** Blind spot occupancy inferred; conservative envelope applied; speed reduced to 25%
- **Validation:** Position human in blind spot; verify conservative mode; verify speed reduction
- **Severity:** HIGH

### TEST-009: Human Carrying Large Object
- **Trigger:** Human with large item enters workspace
- **Expected Response:** Object detected as extension; extended safety zone computed; larger clearance maintained
- **Validation:** Simulate human with object; verify extended tracking; verify clearance
- **Severity:** MEDIUM

### TEST-010: Child Enters Workspace
- **Trigger:** Child (smaller profile) enters workspace
- **Expected Response:** Child detected; lower thresholds activated; height-appropriate zones; no arm motion below 1m
- **Validation:** Simulate child-sized human; verify detection; verify height restrictions
- **Severity:** CRITICAL

---

## 3. Category 2: Sensor Failures (10 Scenarios)

### TEST-011: Lidar Stops Publishing
- **Trigger:** Lidar hardware failure
- **Expected Response:** Missing data detected within 100ms; degraded to available sensors; speed reduced to 10%
- **Validation:** Stop Lidar data; measure detection; verify degraded mode
- **Severity:** HIGH

### TEST-012: Camera Feed Goes Black
- **Trigger:** Camera shows all black frames
- **Expected Response:** Black frame detected; vision features disabled; Lidar safety maintained
- **Validation:** Inject black frames; verify detection; verify Lidar safety maintained
- **Severity:** MEDIUM

### TEST-013: IMU Reports Invalid Data
- **Trigger:** IMU publishes NaN or out-of-range values
- **Expected Response:** Invalid data rejected; odometry-only mode; reduced acceleration limits
- **Validation:** Inject NaN values; verify rejection; verify odometry mode
- **Severity:** MEDIUM

### TEST-014: Joint Encoder Fails
- **Trigger:** Encoder stops reporting
- **Expected Response:** Failure detected; motion stopped on affected joint; brake engaged; e-stop if critical
- **Validation:** Simulate encoder failure; verify detection; verify brake engagement
- **Severity:** CRITICAL

### TEST-015: Force/Torque Sensor Saturates
- **Trigger:** F/T sensor reports maximum values
- **Expected Response:** Saturation detected; collision assumed; immediate stop; compliant mode
- **Validation:** Inject saturated values; verify detection; verify immediate stop
- **Severity:** HIGH

### TEST-016: GPS Signal Lost
- **Trigger:** GPS fix lost (indoor/interference)
- **Expected Response:** Timeout within 2s; localization switches to odometry/SLAM; speed reduced to 50%
- **Validation:** Stop GPS data; verify timeout; verify localization fallback
- **Severity:** MEDIUM

### TEST-017: Ultrasonic False Positive
- **Trigger:** Ultrasonic reports obstacle when clear
- **Expected Response:** False positive detected via cross-validation; ultrasonic ignored temporarily
- **Validation:** Inject false reading; verify cross-sensor check; verify ultrasonic ignored
- **Severity:** LOW

### TEST-018: Depth Camera Calibration Error
- **Trigger:** Misaligned RGB-D data
- **Expected Response:** Calibration error detected; depth features disabled; RGB safety maintained
- **Validation:** Inject misaligned data; verify detection; verify RGB safety maintained
- **Severity:** MEDIUM

### TEST-019: Wheel Odometry Discrepancy
- **Trigger:** Odometry differs significantly from IMU/Lidar
- **Expected Response:** Discrepancy detected; odometry flagged unreliable; speed reduced to 30%
- **Validation:** Inject odometry drift; verify detection; verify sensor fusion adjustment
- **Severity:** MEDIUM

### TEST-020: Battery Voltage Sensor Failure
- **Trigger:** Battery sensor reports invalid data
- **Expected Response:** Failure detected; conservative power mode; return to charging station
- **Validation:** Stop battery data; verify detection; verify return-to-charge
- **Severity:** HIGH

---

## 4. Category 3: Unsafe Commands (10 Scenarios)

### TEST-021: Velocity Exceeds Safety Limit
- **Trigger:** Command velocity > 1.5 m/s
- **Expected Response:** Violation detected; command clamped; violation logged; hardware enforces limit
- **Validation:** Send excessive velocity; verify detection; verify clamping
- **Severity:** HIGH

### TEST-022: Acceleration Too High
- **Trigger:** Acceleration > 2.0 m/s²
- **Expected Response:** Limit checked; command rejected/smoothed; jerk-limited profile applied
- **Validation:** Send high acceleration; verify limit checking; verify smoothing
- **Severity:** HIGH

### TEST-023: Goal Outside Workspace
- **Trigger:** Navigation goal outside boundaries
- **Expected Response:** Boundary check fails; goal rejected; nearest valid goal suggested
- **Validation:** Set goal outside workspace; verify boundary check; verify rejection
- **Severity:** MEDIUM

### TEST-024: Trajectory Through Obstacle
- **Trigger:** Planned trajectory intersects obstacle
- **Expected Response:** Validation fails; safety certificate denied; replanning requested
- **Validation:** Submit trajectory through obstacle; verify validation failure
- **Severity:** CRITICAL

### TEST-025: Move While Human Detected
- **Trigger:** Motion command while human in safety zone
- **Expected Response:** Human check fails; command blocked; "Human detected" message; manual override option
- **Validation:** Place human in zone; send command; verify blocking
- **Severity:** CRITICAL

### TEST-026: Invalid Joint Angle
- **Trigger:** Joint angle outside physical limits
- **Expected Response:** Validation fails; command rejected; nearest valid angle suggested
- **Validation:** Send invalid angle; verify validation; verify rejection
- **Severity:** HIGH

### TEST-027: Gripper Force Exceeds Limit
- **Trigger:** Gripper force command too high
- **Expected Response:** Limit check fails; command clamped; warning issued
- **Validation:** Send excessive force; verify limit check; verify clamping
- **Severity:** HIGH

### TEST-028: Concurrent Conflicting Commands
- **Trigger:** Two simultaneous conflicting commands
- **Expected Response:** Conflict detected; higher priority selected; lower rejected
- **Validation:** Send conflicting commands; verify detection; verify arbitration
- **Severity:** MEDIUM

### TEST-029: Command During E-Stop
- **Trigger:** Motion command while e-stop active
- **Expected Response:** E-stop state checked; command rejected; reset instructions provided
- **Validation:** Activate e-stop; send command; verify rejection
- **Severity:** HIGH

### TEST-030: Malformed Trajectory (NaN)
- **Trigger:** Trajectory contains NaN/Inf values
- **Expected Response:** Schema validation fails; trajectory rejected; safe state maintained
- **Validation:** Submit NaN trajectory; verify validation failure
- **Severity:** CRITICAL

---

## 5. Category 4: System Failures (10 Scenarios)

### TEST-031: Network Latency Spike (>500ms)
- **Trigger:** Network latency exceeds 500ms
- **Expected Response:** Spike detected; conservative mode; speed reduced to 25%
- **Validation:** Inject network delay; verify detection; verify conservative mode
- **Severity:** HIGH

### TEST-032: Network Complete Disconnect
- **Trigger:** Complete network loss
- **Expected Response:** Timeout after 100ms; e-stop triggered; local controller takes over
- **Validation:** Disconnect network; verify timeout; verify e-stop
- **Severity:** CRITICAL

### TEST-033: Safety Validator Crashes
- **Trigger:** Validator process crashes
- **Expected Response:** Watchdog detects; e-stop triggered; crash logged; restart attempted
- **Validation:** Kill validator; verify watchdog; verify e-stop
- **Severity:** CRITICAL

### TEST-034: Watchdog Timeout
- **Trigger:** No heartbeat within 3ms
- **Expected Response:** Detected at 1kHz; e-stop after 3ms; failed node identified
- **Validation:** Stop heartbeat; verify detection timing; verify e-stop timing
- **Severity:** CRITICAL

### TEST-035: E-Stop Button Pressed
- **Trigger:** Physical e-stop activated
- **Expected Response:** Signal detected <10ms; motion stopped <50ms; power removed; latched
- **Validation:** Press e-stop; measure detection; measure stop time
- **Severity:** CRITICAL

### TEST-036: PLC Communication Failure
- **Trigger:** Safety PLC stops responding
- **Expected Response:** Timeout detected; hardware e-stop; PLC functions disabled
- **Validation:** Disconnect PLC; verify timeout; verify e-stop
- **Severity:** CRITICAL

### TEST-037: Safety Certificate Expires
- **Trigger:** Certificate expires during execution
- **Expected Response:** Expiration monitored; 5s grace period; re-validation attempted; stop if invalid
- **Validation:** Let certificate expire; verify monitoring; verify grace period
- **Severity:** HIGH

### TEST-038: Motion Planner Returns Invalid Plan
- **Trigger:** Planner produces invalid plan
- **Expected Response:** Validation fails; plan rejected; fallback planner activated
- **Validation:** Force invalid plan; verify validation failure; verify fallback
- **Severity:** MEDIUM

### TEST-039: Controller Node Crashes
- **Trigger:** Controller crashes
- **Expected Response:** Watchdog detects; hardware e-stop; crash logged; restart attempted
- **Validation:** Kill controller; verify watchdog; verify e-stop
- **Severity:** CRITICAL

### TEST-040: TF Tree Inconsistent
- **Trigger:** TF tree has loops or missing frames
- **Expected Response:** Consistency check fails; planning paused; last valid TF cached
- **Validation:** Inject TF error; verify check failure; verify planning pause
- **Severity:** MEDIUM

---

## 6. Category 5: Edge Cases (12 Scenarios)

### TEST-041: Power Loss During Motion
- **Trigger:** Main power fails while moving
- **Expected Response:** Power loss detected; brakes engage immediately; backup power for safety; state logged
- **Validation:** Simulate power loss; verify brake engagement; verify backup power
- **Severity:** CRITICAL

### TEST-042: Software Update During Operation
- **Trigger:** Update initiated while executing task
- **Expected Response:** Update rejected; "Robot active" message; queued for idle state
- **Validation:** Request update during motion; verify rejection; verify queuing
- **Severity:** MEDIUM

### TEST-043: Clock Skew Between Nodes
- **Trigger:** ROS nodes have >100ms timestamp skew
- **Expected Response:** Skew detected; sync attempted; conservative mode if persists
- **Validation:** Inject clock skew; verify detection; verify sync attempt
- **Severity:** MEDIUM

### TEST-044: Memory Exhaustion
- **Trigger:** Memory usage approaches 100%
- **Expected Response:** Threshold monitoring; warning at 80%; suspend non-critical; e-stop at 95%
- **Validation:** Simulate memory pressure; verify warning; verify e-stop at limit
- **Severity:** HIGH

### TEST-045: Disk Full
- **Trigger:** Disk reaches 100% capacity
- **Expected Response:** Space monitored; warning at 90%; log rotation attempted; RAM buffer for critical events
- **Validation:** Fill disk; verify warning; verify rotation
- **Severity:** MEDIUM

### TEST-046: ROS Bag Recording Stops
- **Trigger:** Bag recording fails
- **Expected Response:** Failure detected; alert issued; restart attempted; operation continues with warning
- **Validation:** Cause recording failure; verify detection; verify restart attempt
- **Severity:** LOW

### TEST-047: Multiple Robots Collision Course
- **Trigger:** Two robots on collision course
- **Expected Response:** Collision detected; priority arbitration; lower priority stops; alternative paths computed
- **Validation:** Setup collision course; verify detection; verify arbitration
- **Severity:** CRITICAL

### TEST-048: Dynamic Obstacle (Falling Object)
- **Trigger:** Object falls into robot path
- **Expected Response:** Falling object detected; immediate stop; protective posture if applicable
- **Validation:** Simulate falling object; verify detection; verify stop
- **Severity:** CRITICAL

### TEST-049: Environmental Hazard (Spill/Smoke)
- **Trigger:** Spill or smoke detected in workspace
- **Expected Response:** Hazard detected; e-stop; hazard type logged; evacuation mode if applicable
- **Validation:** Simulate environmental hazard; verify detection; verify e-stop
- **Severity:** CRITICAL

### TEST-050: Earthquake Simulation (Vibration)
- **Trigger:** Excessive vibration detected
- **Expected Response:** Vibration detected; emergency stop; stability check before resume
- **Validation:** Simulate vibration; verify detection; verify e-stop
- **Severity:** HIGH

### TEST-051: Cyber Attack (Malformed Messages)
- **Trigger:** Malformed/injected ROS messages
- **Expected Response:** Message validation fails; invalid messages rejected; source flagged; security alert
- **Validation:** Inject malformed messages; verify validation; verify rejection
- **Severity:** CRITICAL

### TEST-052: Power Surge
- **Trigger:** Voltage spike on power supply
- **Expected Response:** Surge detected; protection circuits activate; safe shutdown if needed
- **Validation:** Simulate power surge; verify detection; verify protection
- **Severity:** HIGH

---

## 7. Summary Statistics

| Category | Count | Critical | High | Medium | Low |
|----------|-------|----------|------|--------|-----|
| Human Safety | 10 | 4 | 4 | 2 | 0 |
| Sensor Failures | 10 | 1 | 2 | 6 | 1 |
| Unsafe Commands | 10 | 2 | 5 | 3 | 0 |
| System Failures | 10 | 6 | 2 | 2 | 0 |
| Edge Cases | 12 | 5 | 2 | 4 | 1 |
| **TOTAL** | **52** | **18** | **15** | **17** | **2** |

---

## 8. Approval

| Role | Name | Status | Date |
|------|------|--------|------|
| Safety Officer | (Subagent) | APPROVED | 2026-03-06 |

---

**END OF DOCUMENT**
