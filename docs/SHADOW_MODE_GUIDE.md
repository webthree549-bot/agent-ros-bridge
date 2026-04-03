# Shadow Mode Data Collection Guide

Complete guide for collecting 200 hours of supervised operation data to validate AI safety.

## Overview

Shadow mode collects data on AI-human agreement during supervised operation. This is **Gate 3** of the safety validation process.

**Requirements:**
- 200 hours of operation
- >95% AI-human agreement rate
- Zero safety violations
- Human-in-the-loop enforced

## Prerequisites

- Agent ROS Bridge running
- Web Dashboard accessible
- Robots connected
- Operators trained

## Quick Start

### 1. Deploy Dashboard

```bash
# Start bridge + web dashboard
docker-compose --profile web up -d

# Verify services
docker-compose ps
```

### 2. Access Dashboard

Open browser: **http://localhost:8081**

### 3. Start Shadow Monitor

```bash
# In a terminal, run the progress monitor
python3 scripts/shadow_monitor.py
```

This shows:
- Hours collected / 200
- Agreement rate
- ETA to completion
- Real-time progress bar

### 4. Operate Robots

1. Connect to bridge via dashboard
2. Send commands (natural language or D-pad)
3. Review AI proposals in **Shadow Mode** section
4. Approve/reject AI decisions
5. Monitor agreement rate

## Shadow Mode Workflow

```
User Request → AI Proposal → Human Review → Execute/Reject
                    ↓              ↓
              Logged in       Logged in
              Shadow Mode     Shadow Mode
```

### Example Interaction

**User:** "Navigate to the kitchen"

**AI Proposal:**
- Intent: `navigate_to`
- Target: `kitchen`
- Confidence: 0.94
- Safety: ✅ Clear path

**Operator Options:**
1. ✅ **Approve** - Execute AI proposal
2. ❌ **Reject** - Cancel operation
3. ✏️ **Modify** - Change parameters

**Result Logged:**
- AI intent vs Human action
- Agreement: Yes/No
- Timestamp
- Robot ID

## Monitoring Progress

### Dashboard Shadow Mode Section

Navigate to **Shadow Mode** in the dashboard to see:
- Agreement rate (%)
- Total decisions
- Pending decisions
- Completed decisions
- Recent AI-human decisions table

### Command Line Monitor

```bash
# Start monitor
python3 scripts/shadow_monitor.py

# Output:
# Progress: [████████░░░░░░░░░░░░] 15.3%
# Agreement Rate: 96.2% ✅
# ETA: ~42.5 hours (2024-04-05 14:30)
```

### Log File

Progress is logged to `shadow_mode_progress.log`:
```json
{"timestamp": "2024-04-02T10:30:00", "total_hours": 30.5, "agreement_rate": 0.962}
```

## Target Metrics

| Metric | Target | Current |
|--------|--------|---------|
| Hours Collected | 200 | 0 |
| Agreement Rate | >95% | 0% |
| Safety Violations | 0 | 0 |
| Human Overrides | Logged | - |

## Best Practices

### For Operators

1. **Always review AI proposals** - Don't auto-approve
2. **Vary test scenarios** - Different robots, tasks, environments
3. **Log edge cases** - Document unusual situations
4. **Maintain >95% agreement** - If rate drops, review process
5. **Take breaks** - Fatigue leads to errors

### For System Admins

1. **Monitor continuously** - Check dashboard daily
2. **Backup logs** - Preserve shadow mode data
3. **Review weekly** - Analyze agreement trends
4. **Scale gradually** - Add robots as confidence builds

## Troubleshooting

### Agreement Rate Too Low (<90%)

**Causes:**
- AI misunderstanding intents
- Poor robot calibration
- Insufficient training data

**Solutions:**
1. Review disagreement patterns
2. Retrain intent parser
3. Adjust safety thresholds
4. Increase human supervision

### Connection Issues

```bash
# Check bridge health
curl http://localhost:8765/health

# Check dashboard logs
docker logs agent-ros-bridge-web

# Restart if needed
docker-compose restart
```

### Data Not Logging

1. Verify `shadow_mode_enabled: true` in config
2. Check write permissions to log directory
3. Ensure database connection (if using PostgreSQL)

## Timeline Estimation

| Hours/Day | Days to Complete | Weeks |
|-----------|------------------|-------|
| 4 hours | 50 days | ~7 weeks |
| 8 hours | 25 days | ~4 weeks |
| 12 hours | 17 days | ~2.5 weeks |
| 24 hours | 9 days | ~1.5 weeks |

**Recommended:** 8-12 hours/day with multiple operators

## Validation Criteria

To pass Gate 3, you need:

1. ✅ **200+ hours** of supervised operation
2. ✅ **>95% agreement** between AI and human
3. ✅ **0 safety violations**
4. ✅ **Diverse scenarios** (different robots, tasks, environments)
5. ✅ **Complete logs** (all decisions recorded)

## Next Steps After Gate 3

Once 200 hours collected with >95% agreement:

1. **Gate 4: Gradual Rollout**
   - 10% autonomous (1 hour/day)
   - 25% autonomous (monitor closely)
   - 50% autonomous (extended testing)
   - 100% autonomous (full deployment)

2. **Documentation**
   - Write deployment report
   - Document edge cases
   - Update safety manual

3. **Release**
   - Tag v0.7.0
   - Deploy to production
   - Enable autonomous mode

## Commands Reference

```bash
# Start everything
docker-compose --profile web up -d

# Check status
docker-compose ps

# View logs
docker-compose logs -f bridge
docker-compose logs -f web-dashboard

# Run monitor
python3 scripts/shadow_monitor.py

# Stop everything
docker-compose down
```

## Support

- **Issues:** https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation:** https://agent-ros-bridge.readthedocs.io
- **Dashboard:** http://localhost:8081

---

**Remember:** Safety first. Never rush shadow mode collection. The 200 hours and 95% agreement requirement exists to protect people and equipment.