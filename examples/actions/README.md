# ROS Actions Example

**Long-running tasks with progress feedback.**

## What It Does

Demonstrates ROS Actions for navigation and manipulation with real-time progress updates.

## Requirements

- Python 3.8+
- `agent-ros-bridge` installed

## Run

```bash
./run.sh --action navigate
```

Or:
```bash
./run.sh --action manipulate
```

## Test

```bash
# In another terminal
wscat -c ws://localhost:8773

# Navigate to pose
{"command": {"action": "actions.navigate", "parameters": {"x": 5.0, "y": 3.0, "theta": 1.57}}}

# Follow trajectory
{"command": {"action": "actions.follow_trajectory", "parameters": {"waypoints": [[0,0], [1,1], [2,2]]}}}
```

## What's Happening

This demonstrates:
- **Goal Submission**: Send long-running tasks
- **Progress Feedback**: Real-time status updates
- **Cancellation**: Preempt running actions
- **Result Handling**: Success/failure outcomes

## Action Types

| Action | Description |
|--------|-------------|
| `navigate` | Navigate to pose (Nav2) |
| `manipulate` | Follow joint trajectory (MoveIt) |
| `follow_trajectory` | Waypoint following |

## Next Steps

- Read [User Manual - ROS Actions](../../docs/USER_MANUAL.md#ros-actions)
- Try with real Nav2/MoveIt
