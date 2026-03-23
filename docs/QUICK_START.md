# Quick Start Guide

Get started with Agent ROS Bridge in 5 minutes.

---

## Installation

```bash
pip install agent-ros-bridge==0.6.4
```

---

## 1. Basic Usage

### Send Command to Robot

```python
from agent_ros_bridge.gateway import AgentGateway

gateway = AgentGateway()
result = gateway.send_command(
    robot_id='bot1',
    command={
        'type': 'navigate_to',
        'parameters': {'x': 5.0, 'y': 5.0}
    }
)
print(result)  # {'success': True, 'message': 'Command executed'}
```

---

## 2. Shadow Mode Collection

### Start Collecting Data

```python
from agent_ros_bridge.shadow.collector import start_shadow_collection

# Start 200+ hour collection
collector = start_shadow_collection(
    output_dir='shadow_data',
    target_hours=200.0
)

# Data is collected automatically
# Check status
print(collector.get_status())
```

---

## 3. Human Confirmation UI

### Start Web Interface

```python
from agent_ros_bridge.ui.confirmation import ConfirmationUI

ui = ConfirmationUI(port=8080)
ui.start_server()
```

Access at: http://localhost:8080

---

## 4. Run 10K Validation

### Validate Gate 2

```python
from agent_ros_bridge.validation.scenario_10k import run_gate2_validation

result = run_gate2_validation(count=10000)

print(f"Success rate: {result['success_rate']*100:.2f}%")
print(f"Gate 2 passed: {result['gate2_passed']}")
```

---

## 5. Generate Scenarios

### Create Test Scenarios

```python
from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

gen = ScenarioGenerator()

# Generate 100 navigation scenarios
filepaths = gen.generate_and_save_batch(
    template='navigation',
    count=100,
    difficulty='medium'
)

print(f"Generated {len(filepaths)} scenarios")
```

---

## Next Steps

- [Deployment Guide](DEPLOYMENT_GUIDE.md) - Production deployment
- [API Reference](API_REFERENCE.md) - Complete API documentation
- [Examples](../examples/) - Code examples

---

## Support

- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation**: https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
