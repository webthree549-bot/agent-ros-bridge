# Agent ROS Bridge Tools

ROS tool implementations compatible with NASA ROSA (Robot Operating System Agent) tool ecosystem.

## Overview

This module provides standardized tools for AI agents to interact with ROS systems. All tools follow the NASA ROSA tool specification for maximum compatibility.

## Available Tools

### ROSTopicEchoTool

Read data from ROS topics in real-time.

**Usage:**
```python
from agent_ros_bridge.tools import ROSTopicEchoTool

tool = ROSTopicEchoTool()
result = tool.execute(topic="/cmd_vel", count=10)

if result.success:
    print(result.output)
else:
    print(f"Error: {result.error}")
```

**Parameters:**
- `topic` (str): ROS topic name (e.g., '/cmd_vel', '/odom')
- `count` (int): Number of messages to read (default: 1)
- `timeout_sec` (float): Timeout in seconds (default: 5.0)

### ROSServiceCallTool

Call ROS services synchronously.

**Usage:**
```python
from agent_ros_bridge.tools import ROSServiceCallTool

tool = ROSServiceCallTool()
result = tool.execute(
    service="/clear_costmap",
    request={},
    timeout_sec=5.0
)

if result.success:
    print(result.output)
```

**Parameters:**
- `service` (str): ROS service name (e.g., '/get_plan', '/clear_costmap')
- `request` (dict): Service request parameters (default: {})
- `timeout_sec` (float): Timeout in seconds (default: 5.0)

## NASA ROSA Compatibility

These tools are designed to be compatible with NASA's ROSA (Robot Operating System Agent) framework:

- ✅ Standardized tool interface
- ✅ JSON-serializable parameters
- ✅ Structured result format
- ✅ Error handling with details
- ✅ Safety validation integration

## Creating Custom Tools

Inherit from `ROSTool` base class:

```python
from agent_ros_bridge.tools.base import ROSTool, ToolResult

class MyCustomTool(ROSTool):
    name = "my_custom_tool"
    description = "Does something useful"
    version = "1.0.0"
    
    def execute(self, param1: str, **kwargs) -> ToolResult:
        # Implementation
        return ToolResult(success=True, output="Done")
```

## Safety Integration

All tools integrate with Agent ROS Bridge's safety system:
- Human-in-the-loop validation
- Shadow mode logging
- Execution monitoring
- Emergency stop capability

## Testing

Run tool tests:
```bash
pytest tests/unit/tools/ -v
```

## License

MIT License - Same as Agent ROS Bridge
