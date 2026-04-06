# Agent ROS Bridge Sim

Gazebo simulation integration for safe robot validation.

[![Safety First](https://img.shields.io/badge/safety-validated-success)]()
[![Version](https://img.shields.io/badge/version-0.7.0.dev1-blue)]()

## Overview

Agent ROS Bridge Sim provides physics-based simulation for:

- **Safety Validation**: Test safety policies before real-world deployment
- **Scenario Testing**: 10,000+ scenarios in parallel
- **Shadow Mode Training**: Collect AI-human agreement data in simulation
- **Nav2/MoveIt Integration**: Test navigation and manipulation safely

## Installation

```bash
# Requires ROS2 (Humble/Jazzy) and Gazebo
pip install agent-ros-bridge-sim
```

## Quick Start

```python
import asyncio
from agent_ros_bridge_sim import GazeboSimulator, Scenario

async def main():
    # Create simulator
    sim = GazeboSimulator()
    
    # Setup (starts Gazebo + ROS2)
    await sim.setup()
    
    # Load scenario
    scenario = Scenario.from_yaml("scenarios/navigate_basic.yaml")
    
    # Run simulation
    result = await sim.run_scenario(scenario)
    
    print(f"Success: {result.success}")
    print(f"Time: {result.execution_time}s")
    
    # Cleanup
    await sim.cleanup()

asyncio.run(main())
```

## Features

| Feature | Description |
|---------|-------------|
| Gazebo Harmonic | Latest Gazebo with ROS2 integration |
| Nav2 Support | Navigation2 stack for autonomous navigation |
| MoveIt Support | Motion planning for manipulation |
| Batch Runner | Run 10,000+ scenarios in parallel |
| Metrics | Collect detailed performance data |
| Safety Testing | Validate safety policies in simulation |

## Architecture

```
┌─────────────────────────────────────┐
│   GazeboBatchRunner                 │
│   - Parallel scenario execution     │
│   - Metrics collection              │
└─────────────┬───────────────────────┘
              │
    ┌─────────┼─────────┐
    ↓         ↓         ↓
┌───────┐ ┌───────┐ ┌───────┐
│Sim 001│ │Sim 002│ │Sim N  │
│Gazebo │ │Gazebo │ │Gazebo │
└───────┘ └───────┘ └───────┘
```

## Safety Validation

Run the 10K scenario validation suite:

```python
from agent_ros_bridge_sim import GazeboBatchRunner, BatchConfig

config = BatchConfig(
    scenarios_dir="scenarios/",
    parallel_workers=100,
    max_scenarios=10000,
)

runner = GazeboBatchRunner(config)
results = await runner.run_all()

print(f"Pass rate: {results.pass_rate:.2%}")
```

## Docker Support

Pre-built Docker image with ROS2 Jazzy + Nav2:

```bash
docker run -it agent-ros-bridge-sim:jazzy
```

## License

MIT License - See [LICENSE](../LICENSE) for details.
