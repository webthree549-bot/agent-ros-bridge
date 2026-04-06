# Agent ROS Bridge Fleet

Multi-robot fleet management for production deployments.

[![Safety First](https://img.shields.io/badge/safety-validated-success)]()
[![Version](https://img.shields.io/badge/version-0.7.0.dev1-blue)]()

## Overview

Agent ROS Bridge Fleet provides enterprise-grade multi-robot orchestration with:

- **Task Allocation**: Intelligent task distribution across heterogeneous robots
- **Load Balancing**: Battery-aware, capability-matched task assignment
- **Fleet Coordination**: Mixed ROS1/ROS2 robot support
- **Safety Integration**: Built-in validation with Agent ROS Bridge safety layer

## Installation

```bash
pip install agent-ros-bridge-fleet
```

## Quick Start

```python
import asyncio
from agent_ros_bridge_fleet import (
    FleetOrchestrator,
    FleetRobot,
    RobotCapability,
    Task,
)

async def main():
    # Create orchestrator
    orchestrator = FleetOrchestrator()
    
    # Add robots to fleet
    robot1 = FleetRobot(
        robot_id="bot_01",
        name="Warehouse Bot 1",
        capabilities=RobotCapability(
            can_navigate=True,
            can_manipulate=True,
            max_payload_kg=50.0,
            ros_version="ros2",
        ),
    )
    await orchestrator.add_robot(robot1)
    
    # Submit task
    task = Task(
        type="navigate",
        target_location="zone_a",
        priority=1,
    )
    await orchestrator.submit_task(task)
    
    # Start orchestration
    await orchestrator.start()

asyncio.run(main())
```

## Features

| Feature | Description |
|---------|-------------|
| Task Queue | Priority-based task scheduling |
| Capability Matching | Assign tasks to capable robots |
| Battery Awareness | Route tasks based on charge levels |
| Dependency Management | Task chains and prerequisites |
| Fleet Metrics | Real-time performance monitoring |

## Architecture

```
┌─────────────────────────────────────┐
│   FleetOrchestrator                 │
│   - Task allocation                 │
│   - Load balancing                  │
│   - Coordination                    │
└─────────────┬───────────────────────┘
              │
    ┌─────────┼─────────┐
    ↓         ↓         ↓
┌───────┐ ┌───────┐ ┌───────┐
│Robot 1│ │Robot 2│ │Robot 3│
│ROS2   │ │ROS1   │ │ROS2   │
└───────┘ └───────┘ └───────┘
```

## Safety

Fleet operations respect Agent ROS Bridge safety configuration:

- Human-in-the-loop approval for critical tasks
- Shadow mode for validation
- Gradual rollout support

## License

MIT License - See [LICENSE](../LICENSE) for details.
