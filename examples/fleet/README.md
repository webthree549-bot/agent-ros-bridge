# Fleet Example

**Multi-robot fleet orchestration demo.**

## What It Does

Simulates a fleet of 4 robots with task allocation, load balancing, and real-time metrics.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh
```

## Test

```bash
# In another terminal
wscat -c ws://localhost:8771

# Submit a navigation task
{"command": {"action": "fleet.submit_task", "parameters": {"type": "navigate", "target_location": "zone_a", "priority": 5}}}

# Check fleet status
{"command": {"action": "fleet.status"}}

# Get metrics
{"command": {"action": "fleet.metrics"}}
```

## What's Happening

This demonstrates:
- **Task Allocation**: Tasks assigned to available robots
- **Load Balancing**: Work distributed across fleet
- **Priority Queue**: Higher priority tasks served first
- **Real-time Metrics**: Fleet utilization tracking

All simulated — no real robots needed.

## Next Steps

- Read the [Fleet Management guide](../../docs/USER_MANUAL.md#fleet-management)
- Try [authentication](../auth/) for secure fleets
