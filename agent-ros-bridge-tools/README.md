# Agent ROS Bridge Tools

ROS CLI tool ecosystem for Agent ROS Bridge.

[![Safety First](https://img.shields.io/badge/safety-validated-success)]()
[![Version](https://img.shields.io/badge/version-0.7.0.dev1-blue)]()

## Overview

Agent ROS Bridge Tools provides familiar ROS CLI tools as Agent ROS Bridge
integrations. Compatible with NASA ROSA tool format (MIT licensed).

### Available Tools

| Tool | Description | NASA ROSA Compatible |
|------|-------------|---------------------|
| `rostopic_echo` | Echo messages from ROS topics | ✅ |
| `rosservice_call` | Call ROS services | ✅ |
| `rosnode_list` | List running ROS nodes | ✅ |
| `rosparam_get` | Get ROS parameters | ✅ |
| `rosbag_play` | Play recorded bag files | ✅ |

## Installation

```bash
pip install agent-ros-bridge-tools
```

## Quick Start

```python
import asyncio
from agent_ros_bridge_tools import (
    ROSTopicEchoTool,
    ROSNodeListTool,
    ROSServiceCallTool,
)

async def main():
    # Echo a topic
    echo_tool = ROSTopicEchoTool()
    result = await echo_tool.execute("/cmd_vel", count=5)
    print(f"Received {result.data['count']} messages")

    # List nodes
    list_tool = ROSNodeListTool()
    result = await list_tool.execute()
    print(f"Found {result.data['count']} nodes")

    # Call a service
    call_tool = ROSServiceCallTool()
    result = await call_tool.execute("/spawn", x=1.0, y=2.0, name="robot1")
    print(f"Service response: {result.data['response']}")

asyncio.run(main())
```

## Plugin API

Create custom tools:

```python
from agent_ros_bridge_tools import Tool, ToolResult, register_tool

@register_tool
class MyCustomTool(Tool):
    name = "my_tool"
    description = "Does something useful"
    
    async def execute(self, param: str) -> ToolResult:
        # Your implementation
        return ToolResult.success_result(data={"result": param.upper()})
```

## NASA ROSA Compatibility

These tools are compatible with NASA ROSA (MIT License):

```python
# Works with both Agent ROS Bridge and NASA ROSA
from agent_ros_bridge_tools import ROSTopicEchoTool

tool = ROSTopicEchoTool()
result = await tool.execute("/cmd_vel")
```

## Architecture

```
┌─────────────────────────────────────┐
│   Agent ROS Bridge Tools            │
│   - ROSTopicEchoTool                │
│   - ROSServiceCallTool              │
│   - ROSNodeListTool                 │
│   - ROSParamGetTool                 │
│   - ROSBagPlayTool                  │
└─────────────┬───────────────────────┘
              │
    ┌─────────┼─────────┐
    ↓         ↓         ↓
┌───────┐ ┌───────┐ ┌───────┐
│ ROS1  │ │ ROS2  │ │ Custom│
└───────┘ └───────┘ └───────┘
```

## License

MIT License - See [LICENSE](../LICENSE) for details.
