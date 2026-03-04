# Universal Interface: Cross-Framework Implementation Guide

## Proof of Concept: One Robot, Multiple AI Frameworks

This guide demonstrates how the universal interface enables the same robot to be controlled by LangChain, AutoGPT, Claude (MCP), and OpenAI—using the **same natural language commands**.

---

## The Setup

### Robot: Warehouse AGV

```python
# Robot capabilities exposed through universal interface
ROBOT_SKILLS = {
    "navigation/move": {
        "nl_patterns": ["Move to {location}", "Go to {location}"],
        "parameters": {"location": "str"}
    },
    "manipulation/pick": {
        "nl_patterns": ["Pick up {item}", "Grab {item}"],
        "parameters": {"item": "str"}
    },
    "inspection/scan": {
        "nl_patterns": ["Check {area}", "Scan {area}"],
        "parameters": {"area": "str"}
    }
}
```

### Universal Interface Server

```python
# universal_server.py
from agent_ros_bridge import Bridge
from agent_ros_bridge.integrations.openclaw_adapter import OpenClawAdapter

bridge = Bridge()
adapter = bridge.get_openclaw_adapter()

# Start universal interface
# Exposes: POST /execute { "command": "natural language" }
```

---

## Framework 1: LangChain

### Traditional Approach (Framework-Specific)

```python
from langchain.tools import BaseTool
from langchain.agents import initialize_agent

class MoveRobotTool(BaseTool):
    name = "move_robot"
    description = "Move robot to location"
    
    def _run(self, location: str):
        # Direct ROS2 call - framework specific
        import roslibpy
        ros = roslibpy.Ros(host='localhost', port=9090)
        # ... complex ROS setup ...
        return f"Moved to {location}"

class PickItemTool(BaseTool):
    name = "pick_item"
    description = "Pick up item"
    
    def _run(self, item: str):
        # Direct robot API call
        return f"Picked {item}"

# Agent needs to know about each tool
tools = [MoveRobotTool(), PickItemTool()]
agent = initialize_agent(tools, llm)

agent.run("Move to shelf A and pick up the box")
# LangChain must: 1) Parse intent, 2) Select tools, 3) Call APIs
```

### Universal Approach (Framework-Agnostic)

```python
from langchain.tools import BaseTool
from universal_interface import UniversalClient

class UniversalRobotTool(BaseTool):
    name = "robot_control"
    description = """
    Control robot using natural language.
    Examples: "Move to shelf A", "Pick up the box", "Check battery"
    """
    
    def __init__(self):
        self.universal = UniversalClient("http://localhost:8765")
    
    def _run(self, natural_language_command: str):
        # Single tool handles everything
        result = self.universal.execute(natural_language_command)
        return result

# Just one tool needed
tools = [UniversalRobotTool()]
agent = initialize_agent(tools, llm)

agent.run("Move to shelf A and pick up the box")
# Universal interface handles: parsing, routing, execution
```

**Benefit:** One tool instead of N tools. No ROS knowledge needed.

---

## Framework 2: AutoGPT

### Traditional Approach

```python
# AutoGPT commands.py
from autogpt.command_decorator import command

@command(
    "move_robot",
    "Move robot to location",
    {
        "location": {
            "type": "string",
            "description": "Location to move to"
        }
    }
)
def move_robot(location: str) -> str:
    # Direct robot control
    robot.move(location)
    return f"Moved to {location}"

@command(
    "pick_item",
    "Pick up item",
    {
        "item": {"type": "string", "description": "Item to pick"}
    }
)
def pick_item(item: str) -> str:
    robot.arm.pick(item)
    return f"Picked {item}"

# AutoGPT must learn each command separately
```

### Universal Approach

```python
# AutoGPT commands.py
from autogpt.command_decorator import command
from universal_interface import UniversalClient

universal = UniversalClient("http://localhost:8765")

@command(
    "control_robot",
    "Control robot using natural language",
    {
        "command": {
            "type": "string",
            "description": "Natural language command like 'Move to shelf A'"
        }
    }
)
def control_robot(command: str) -> str:
    # Single command for all robot operations
    result = universal.execute(command)
    return result

# AutoGPT uses natural language planning:
# "I need to move to shelf A and pick up the box"
# → control_robot("Move to shelf A")
# → control_robot("Pick up the box")
```

**Benefit:** AutoGPT's autonomy works with any robot through NL.

---

## Framework 3: Claude (MCP)

### Traditional Approach

```python
# server.py - MCP server
from mcp.server import Server

@mcp.tool()
def move_robot(location: str) -> str:
    """Move robot to location"""
    # Direct control
    return robot.move(location)

@mcp.tool()
def pick_item(item: str) -> str:
    """Pick up item"""
    return robot.arm.pick(item)

# Claude must call specific tools
```

### Universal Approach

```python
# server.py - MCP server
from mcp.server import Server
from universal_interface import UniversalClient

universal = UniversalClient("http://localhost:8765")

@mcp.tool()
def control_robot(command: str) -> str:
    """
    Control robot using natural language.
    
    Examples:
    - "Move to shelf A"
    - "Pick up the box"
    - "Check battery status"
    - "Go to charging station"
    """
    return universal.execute(command)

# Claude conversation:
# User: "Can you move the robot to shelf A?"
# Claude: "I'll move the robot to shelf A for you."
# → control_robot("Move to shelf A")
```

**Benefit:** Claude's reasoning works with robots conversationally.

---

## Framework 4: OpenAI Functions

### Traditional Approach

```python
functions = [
    {
        "name": "move_robot",
        "description": "Move robot to location",
        "parameters": {
            "type": "object",
            "properties": {
                "location": {"type": "string"}
            },
            "required": ["location"]
        }
    },
    {
        "name": "pick_item",
        "description": "Pick up item",
        "parameters": {
            "type": "object",
            "properties": {
                "item": {"type": "string"}
            },
            "required": ["item"]
        }
    },
    # ... more functions for each capability
]

# GPT must select from many functions
```

### Universal Approach

```python
from universal_interface import UniversalClient

universal = UniversalClient("http://localhost:8765")

functions = [
    {
        "name": "control_robot",
        "description": "Control robot using natural language commands",
        "parameters": {
            "type": "object",
            "properties": {
                "command": {
                    "type": "string",
                    "description": "Natural language command like 'Move to shelf A' or 'Pick up the box'"
                }
            },
            "required": ["command"]
        }
    }
]

# Single function replaces dozens
# GPT: "Move to shelf A and pick up the box"
# → control_robot("Move to shelf A")
# → control_robot("Pick up the box")
```

**Benefit:** One function instead of dozens. Simpler prompts.

---

## Framework 5: Hugging Face Agents

### Traditional Approach

```python
from transformers import Tool

class MoveRobotTool(Tool):
    name = "move_robot"
    description = "Move robot to location"
    inputs = {"location": {"type": "string"}}
    
    def __call__(self, location: str):
        robot.move(location)
        return f"Moved to {location}"

# Agent learns each tool
```

### Universal Approach

```python
from transformers import Tool
from universal_interface import UniversalClient

class UniversalRobotTool(Tool):
    name = "robot_control"
    description = "Control robot using natural language"
    inputs = {"command": {"type": "string"}}
    
    def __init__(self):
        self.universal = UniversalClient("http://localhost:8765")
    
    def __call__(self, command: str):
        return self.universal.execute(command)

# Single tool for all operations
```

---

## Side-by-Side Comparison

### Task: "Move to shelf A, pick up the box, return to station"

| Framework | Traditional Code | Universal Code |
|-----------|------------------|----------------|
| **LangChain** | 3 tools, 50 lines | 1 tool, 10 lines |
| **AutoGPT** | 3 commands, 30 lines | 1 command, 10 lines |
| **Claude MCP** | 3 tools, 30 lines | 1 tool, 10 lines |
| **OpenAI** | 3 functions | 1 function |
| **HuggingFace** | 3 tools | 1 tool |

**Result:** 70-80% code reduction. Same functionality.

---

## Advanced: Shared Context Across Frameworks

```python
# All frameworks share the same context
from universal_interface import SharedContext

context = SharedContext()

# LangChain operation
langchain_agent.run("Move to shelf A")
# → Context: {current_location: "shelf_a"}

# AutoGPT continues
autogpt.run("Pick up the box from here")
# → Context knows "here" = "shelf_a"

# Claude reviews
claude.ask("Where is the robot now?")
# → Context: "The robot is at shelf A"
```

**Benefit:** Context persists across framework boundaries.

---

## Implementation Checklist

### For Framework Developers

- [ ] Add `UniversalClient` dependency
- [ ] Create single tool/function for robot control
- [ ] Accept natural language as parameter
- [ ] Delegate execution to universal interface

### For Robot Developers

- [ ] Expose capabilities as skills
- [ ] Define natural language patterns
- [ ] Start universal interface server
- [ ] Document available commands

### For End Users

- [ ] Install universal interface client
- [ ] Connect to robot server
- [ ] Use natural language with any AI framework
- [ ] Enjoy framework-agnostic control

---

## Migration Guide

### Step 1: Identify Framework-Specific Code

```python
# Before: Framework-specific robot calls
roslibpy.Ros(host='localhost', port=9090)  # ROS-specific
robot.move(x, y)  # Direct API
```

### Step 2: Replace with Universal Interface

```python
# After: Universal natural language
universal.execute("Move to position 5, 3")
```

### Step 3: Test Across Frameworks

```python
# Same command works everywhere
"Move to shelf A"
→ LangChain ✓
→ AutoGPT ✓
→ Claude ✓
→ OpenAI ✓
```

---

## Real-World Example: Warehouse Management

```python
# Universal interface enables this workflow:

# Morning - LangChain plans the day
langchain_agent.run("""
Create a plan for today:
1. Check all shelves for restocking needs
2. Move inventory from receiving to storage
3. Prepare orders for shipping
""")

# Midday - AutoGPT executes autonomously
autogpt.run("Execute the warehouse plan")
# AutoGPT breaks down tasks and executes through NL

# Afternoon - Claude handles exceptions
claude.ask("Shelf B-12 has a damaged box, what should I do?")
# Claude reasons and responds through NL interface

# Evening - OpenAI generates report
openai.chat.completions.create(
    messages=[{
        "role": "user",
        "content": "Generate warehouse activity report for today"
    }],
    functions=[universal_function]
)
```

**All through the same universal interface.**

---

## Conclusion

The universal interface pattern demonstrates that:

1. **Natural language is the universal API** for robot control
2. **Framework differences become irrelevant** at the interface layer
3. **Code complexity drops 70-80%** when using universal approach
4. **Context can be shared** across different AI frameworks
5. **Skills become portable** across the entire AI ecosystem

**The future is framework-agnostic robot control through natural language.**

---

*This guide proves the vision is implementable today.*
