# Examples

Practical examples demonstrating Agent ROS Bridge capabilities.

## Quick Start Examples

### Basic Robot Control
```python
from agent_ros_bridge import RobotAgent

# Initialize robot
robot = RobotAgent(device_id="bot1", device_type="mobile_robot")

# Navigate to location
result = await robot.execute("Go to the kitchen")
```

### Using Tools Directly
```python
from agent_ros_bridge.tools import ROSTopicEchoTool

tool = ROSTopicEchoTool()
result = tool.execute(topic="/cmd_vel", count=5)
```

## AI Agent Integration Examples

See `ai_agent_integrations/` for:
- LangChain integration
- OpenClaw integration  
- Claude Desktop MCP integration
- Natural language examples

## Complete Demos

- `warehouse_automation.py` - Fleet coordination
- `healthcare_assistant.py` - Safety-critical operations
- `multiprotocol_iot_fleet.py` - Multi-protocol coordination

## Running Examples

```bash
# Quick demo
python3 examples/ai_agent_integrations/quick_demo.py

# Interactive demo
python3 examples/ai_agent_integrations/openclaw_integration_demo.py
```
