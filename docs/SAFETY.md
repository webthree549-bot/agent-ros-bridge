# Safety Guidelines for Agent ROS Bridge

**Version:** v0.6.5  
**Last Updated:** 2026-03-30  
**Status:** REQUIRED READING before production deployment

---

## ⚠️ Critical Warning

**Agent ROS Bridge requires human-in-the-loop operation until shadow mode validation is complete.**

The system is designed with a **safe-by-default** configuration:
- `autonomous_mode: false` (human approval required)
- `human_in_the_loop: true` (all AI proposals need human confirmation)
- `shadow_mode_enabled: true` (data collection active)

**DO NOT** enable autonomous mode without completing the full validation pipeline.

---

## Safety Architecture

### Layers of Protection

```
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 4: HUMAN OVERRIDE                                         │
│ - Emergency stop buttons                                        │
│ - Real-time monitoring dashboard                                │
│ - Human can interrupt any action                                │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 3: SAFETY VALIDATION                                      │
│ - Shadow mode data collection (200+ hours)                      │
│ - AI-human agreement measurement (>95% required)                │
│ - Gradual rollout with monitoring                             │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 2: SIMULATION VALIDATION                                  │
│ - 10,000+ scenarios in Gazebo (✅ PASSED - Gate 2)              │
│ - 95.93% success rate demonstrated                              │
│ - 0 safety violations in simulation                             │
└─────────────────────────────────────────────────────────────────┘
                              ↑
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 1: SAFE DEFAULTS                                          │
│ - autonomous_mode: false                                        │
│ - human_in_the_loop: true                                       │
│ - High confidence threshold (0.95)                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## Deployment Stages

### Stage 0: Simulation-Only (Current Status)

**Validation Status:** `simulation_only`

- ✅ 10K scenarios tested in Gazebo
- ✅ 95.93% success rate
- ✅ 0 safety violations
- ⚠️ **No real-world validation yet**

**Configuration:**
```yaml
safety:
  autonomous_mode: false
  human_in_the_loop: true
  safety_validation_status: "simulation_only"
```

**Requirements for next stage:**
- [ ] Deploy to supervised physical environment
- [ ] Enable shadow mode data collection
- [ ] Begin collecting human-AI decision pairs

---

### Stage 1: Supervised Operation

**Validation Status:** `supervised`

**Goal:** Collect 200+ hours of shadow mode data

**Configuration:**
```yaml
safety:
  autonomous_mode: false        # Still requires human approval
  human_in_the_loop: true       # All decisions supervised
  shadow_mode_enabled: true     # Collect data
  safety_validation_status: "supervised"
```

**Data Collection:**
- AI proposes actions
- Human approves/rejects/modifies
- System logs both AI proposal and human decision
- Calculates agreement rate in real-time

**Organic Collection Strategy:**
Instead of synthetic data collection, shadow mode runs continuously during normal supervised operation:

```python
# During every robot interaction
1. AI generates proposal
2. Shadow mode logs the proposal
3. Human approves/rejects
4. Shadow mode logs the human decision
5. System calculates agreement
```

**Exit Criteria:**
- 200+ hours of operation
- >95% AI-human agreement rate
- Zero safety incidents
- Human operator confidence

---

### Stage 2: Gradual Rollout

**Validation Status:** `validated`

**Goal:** Gradually increase autonomy while monitoring

**Configuration:**
```yaml
safety:
  autonomous_mode: true         # Enable autonomy
  human_in_the_loop: false      # For high-confidence actions
  min_confidence_for_auto: 0.95 # Very high threshold
  gradual_rollout_stage: 10     # Start at 10% autonomy
  safety_validation_status: "validated"
```

**Rollout Strategy:**

| Stage | Autonomy % | Conditions | Monitoring |
|-------|------------|------------|------------|
| 10% | Low-confidence only | confidence > 0.99 | Real-time alerts |
| 25% | Common tasks | confidence > 0.97 | Hourly review |
| 50% | Routine operations | confidence > 0.95 | Daily review |
| 75% | Most tasks | confidence > 0.93 | Weekly review |
| 100% | Full autonomy | All confidence levels | Monthly audit |

**At each stage:**
1. Run for minimum 1 week
2. Monitor incident rates
3. Collect operator feedback
4. Adjust confidence thresholds
5. Roll back if issues detected

---

### Stage 3: Full Autonomy

**Validation Status:** `autonomous`

**Goal:** Unsupervised operation in controlled environments

**Configuration:**
```yaml
safety:
  autonomous_mode: true
  human_in_the_loop: false
  gradual_rollout_stage: 100
  safety_validation_status: "autonomous"
```

**Requirements:**
- 6+ months of validated operation
- <0.1% incident rate
- Regulatory compliance (ISO 10218)
- Insurance approval
- Emergency response plan

---

## Configuration Reference

### Safety Configuration Options

```yaml
safety:
  # Primary safety flag
  # DEFAULT: false (safe)
  # WARNING: Only enable after validation complete
  autonomous_mode: false

  # Require human approval
  # DEFAULT: true
  # Should remain true until validation complete
  human_in_the_loop: true

  # Enable shadow mode logging
  # DEFAULT: true
  # Should be true during all supervised operation
  shadow_mode_enabled: true

  # Minimum confidence for autonomy
  # DEFAULT: 0.95 (95%)
  # Range: 0.0 - 1.0
  min_confidence_for_auto: 0.95

  # Gradual rollout percentage
  # DEFAULT: 0 (0% autonomous)
  # Range: 0 - 100
  gradual_rollout_stage: 0

  # Current validation stage
  # Options: simulation_only, supervised, validated, autonomous
  safety_validation_status: "simulation_only"

  # Metrics (auto-updated)
  shadow_mode_hours_collected: 0.0
  shadow_mode_agreement_rate: 0.0

  # Requirements (read-only targets)
  required_shadow_hours: 200.0
  min_agreement_rate: 0.95
```

### Environment Variables

```bash
# Override safety settings via environment
export BRIDGE_SAFETY_AUTONOMOUS_MODE=false
export BRIDGE_SAFETY_HUMAN_IN_THE_LOOP=true
export BRIDGE_SAFETY_SHADOW_MODE_ENABLED=true
export BRIDGE_SAFETY_MIN_CONFIDENCE=0.95
```

---

## Monitoring and Alerts

### Key Metrics to Monitor

| Metric | Target | Alert Threshold |
|--------|--------|-----------------|
| Agreement Rate | >95% | <90% |
| Incident Rate | <0.1% | >0.5% |
| Human Override Rate | <5% | >10% |
| Confidence Calibration | Within ±5% | Outside ±10% |
| Shadow Hours | 200+ | <100 |

### Dashboard Access

```
http://localhost:8000/static/index.html
```

Displays:
- Real-time agreement rate
- Gradual rollout progress
- Recent decisions and outcomes
- Safety alerts

---

## Emergency Procedures

### Immediate Stop

```python
# Emergency stop all robots
from agent_ros_bridge import RobotController

controller = RobotController()
controller.emergency_stop_all()
```

### Disable Autonomous Mode

```bash
# Immediate disable
export BRIDGE_SAFETY_AUTONOMOUS_MODE=false
```

### Rollback Procedure

1. **Immediate:** Disable autonomous mode
2. **Assess:** Review incident logs
3. **Adjust:** Modify confidence thresholds
4. **Retrain:** Collect additional shadow data
5. **Re-validate:** Re-run gradual rollout

---

## Regulatory Compliance

### ISO 10218 (Robot Safety)

Requirements:
- [ ] Risk assessment documented
- [ ] Safety-rated monitored stop
- [ ] Protective stop capability
- [ ] Emergency stop capability
- [ ] Speed and separation monitoring

### Documentation Required

- Safety validation report
- Shadow mode data analysis
- Incident response plan
- Operator training records
- Maintenance procedures

---

## Alternatives to Shadow Mode

If you cannot collect 200+ hours of shadow mode data:

### Option 1: Human-in-the-Loop Forever
- **Use case:** High-risk environments
- **Configuration:** `human_in_the_loop: true` (permanent)
- **Trade-off:** Slower operations, maximum safety

### Option 2: Formal Verification
- **Use case:** Safety-critical systems
- **Approach:** Mathematical safety proofs
- **Trade-off:** More restrictive AI capabilities

### Option 3: Hardware Safety Layer
- **Use case:** Physical robots
- **Approach:** Independent safety PLC
- **Trade-off:** Additional hardware cost

### Option 4: Simulation-Only Deployment
- **Use case:** Virtual robots, training
- **Approach:** Trust simulation validation
- **Trade-off:** No physical deployment

---

## Best Practices

### DO

✅ Keep `human_in_the_loop: true` until validation complete  
✅ Enable shadow mode during all supervised operation  
✅ Monitor agreement rate continuously  
✅ Start with very high confidence thresholds (0.99+)  
✅ Have emergency stop readily available  
✅ Document all incidents  
✅ Regular safety audits  

### DON'T

❌ Enable `autonomous_mode` without validation  
❌ Skip shadow mode data collection  
❌ Rush gradual rollout stages  
❌ Ignore operator feedback  
❌ Deploy without emergency procedures  
❌ Assume simulation covers all scenarios  

---

## Incident Response

### Classification

| Level | Description | Response |
|-------|-------------|----------|
| 1 | Near miss | Log and review |
| 2 | Minor incident | Immediate review, adjust thresholds |
| 3 | Major incident | Disable autonomy, full investigation |
| 4 | Safety violation | Emergency stop, regulatory notification |

### Reporting Template

```
Incident ID: [auto-generated]
Timestamp: [ISO 8601]
Classification: [1-4]
Description: [What happened]
AI Proposal: [What AI suggested]
Human Action: [What human did]
Outcome: [Result]
Root Cause: [Analysis]
Remediation: [Steps taken]
```

---

## Contact and Support

### Safety Team

- **Safety Lead:** safety@yourorg.com
- **On-call Engineer:** +1-xxx-xxx-xxxx
- **Emergency:** 911 (for physical incidents)

### Resources

- Shadow Mode Dashboard: http://localhost:8000
- Incident Log: `./shadow_data/incidents.log`
- Configuration: `./config/global_config.yaml`
- This Document: `./docs/SAFETY.md`

---

## Summary

**Remember:**

1. **Safe by default:** System ships with `autonomous_mode: false`
2. **Validation required:** 200+ hours shadow mode + >95% agreement
3. **Gradual rollout:** Start at 0%, increase slowly with monitoring
4. **Human override:** Always possible at any stage
5. **Documentation:** Keep detailed records of all safety decisions

**The goal is not to eliminate human oversight, but to prove the AI is worthy of trust through empirical evidence.**

---

*Last updated: 2026-03-30*  
*Document version: v0.6.5*  
*Next review: After first 100 shadow mode hours*
