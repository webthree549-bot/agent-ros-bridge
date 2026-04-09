# AI Agent Integrations for Agent ROS Bridge

This directory demonstrates how Agent ROS Bridge integrates with popular AI agent frameworks to enable **natural language control of ROS robots** with **adaptive safety**.

---

## 🎯 Overview

Agent ROS Bridge serves as the **safety-critical middleware** between AI agents and ROS robots. It enables:

- ✅ **Natural language** robot control
- ✅ **Adaptive safety** based on context
- ✅ **Human-in-the-loop** enforcement
- ✅ **Multi-agent framework** support
- ✅ **Shadow mode** learning

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     AI AGENT FRAMEWORK                          │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐                │
│  │ LangChain  │  │  OpenClaw  │  │Claude Desk │                │
│  │   Agent    │  │   Agent    │  │    MCP     │                │
│  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘                │
└────────┼───────────────┼───────────────┼────────────────────────┘
         │               │               │
         └───────────────┼───────────────┘
                         │
┌────────────────────────▼────────────────────────────────────────┐
│              AGENT ROS BRIDGE (Safety Layer)                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  • Natural Language Interpreter                          │   │
│  │  • Safety Validator (shadow mode + human-in-the-loop)    │   │
│  │  • Action Router                                         │   │
│  │  • Emergency Stop Handler                                │   │
│  └─────────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         │               │               │
┌────────▼──────┐ ┌──────▼─────┐ ┌──────▼──────┐
│   ROS Tools   │ │  Actions   │ │  Telemetry  │
│  (rostopic)   │ │  (Nav2)    │ │  (Sensors)  │
└───────────────┘ └────────────┘ └─────────────┘
```

---

## 🤖 Supported AI Frameworks

### 1. LangChain Integration

**File:** `langchain_integration.py`

Enable LangChain agents to control ROS robots:

```python
from langchain_integration import create_ros_agent

# Create LangChain agent with ROS capabilities
agent = create_ros_agent()

# Natural language robot control
result = agent.invoke({
    "input": "Navigate to the kitchen"
})

# Agent interprets, validates safety, executes
```

**Features:**
- Natural language to ROS command translation
- Integrated safety validation
- Tool-based architecture
- Human confirmation workflow

**Example Commands:**
```
"Navigate to position A"
"Check /cmd_vel topic"
"Call /clear_costmap service"
"What's the robot status?"
```

---

### 2. OpenClaw Integration

**File:** `openclaw_integration.py`

Enable OpenClaw agents to interact with ROS:

```python
from openclaw_integration import OpenClawROSBridge

bridge = OpenClawROSBridge()

# Process natural language
result = await bridge.process_natural_language(
    "Go to the kitchen carefully"
)

# Get natural response
response = bridge.generate_natural_response(result)
print(response)  # "✅ I've navigated to kitchen..."
```

**Features:**
- Pattern-based NL interpretation
- Context-aware command generation
- Safety level adaptation
- Command history tracking

**Adaptive Safety Example:**
```python
# Normal navigation
"Go to the kitchen" → Safety: medium, Confirmation: required

# Urgent navigation
"Go to the kitchen NOW" → Safety: high, Confirmation: required

# Emergency navigation
"Emergency: Go to exit" → Safety: critical, Confirmation: bypassed
```

---

### 3. Claude Desktop MCP Integration

**File:** `claude_desktop_mcp_integration.py`

Enable Claude Desktop to control robots via MCP:

```claude
User: "Navigate to the kitchen"

Claude: I'll help you navigate to the kitchen.
⚠️  Safety: Human confirmation required...

[Uses ros_navigate tool]
✅ Successfully navigated to kitchen.
Execution time: 12.4s
Human Approvals: 1
```

**MCP Tools Exposed:**
- `ros_navigate` - Navigate to locations
- `ros_inspect_topic` - Read ROS topics
- `ros_call_service` - Call ROS services
- `ros_get_status` - Get robot status
- `ros_emergency_stop` - Emergency stop
- `ros_manipulate` - Arm/gripper control

**Safety Features:**
- Tool descriptions include safety warnings
- Confirmation required for dangerous actions
- Emergency stop bypasses normal flow
- Status shows shadow mode progress

---

## 🗣️ Natural Language Examples

**File:** `natural_language_examples.py`

Demonstrates NL instruction generation and interpretation:

```python
# Generate context-aware instructions
nlg = NaturalLanguageGenerator(robot)

# Context-aware navigation
cmd = nlg.generate_navigation_command(
    destination="kitchen",
    urgency="normal",
    context={"obstacles": ["chair", "table"]}
)
# → "Please navigate to kitchen, carefully avoiding chair, table."

# Interpret user input
nli = NaturalLanguageInterpreter(robot)
result = nli.interpret("Go to the kitchen carefully")
# → {action: "navigate", destination: "kitchen", speed: "slow"}
```

---

## 🛡️ Safety Mechanisms

### Adaptive Safety Levels

| Situation | Safety Level | Confirmation | Example |
|-----------|--------------|--------------|---------|
| Normal operation | Medium | Required | "Go to kitchen" |
| Obstacles present | High | Required | "Go to kitchen avoiding chair" |
| Urgent request | High | Required | "Go to kitchen NOW" |
| Emergency | Critical | Bypassed | "Emergency: Go to exit" |
| Inspection | Low | Not required | "Check battery status" |
| Manipulation | High | Required | "Pick up the cup" |

### Safety Workflow

```
User Request
    ↓
[AI Agent Interpretation]
    ↓
[Safety Level Assessment]
    ↓
┌─────────────────────────────────────┐
│  Critical? → Execute immediately    │
│  High? → Require confirmation       │
│  Medium? → Log, execute with care   │
│  Low? → Execute directly            │
└─────────────────────────────────────┘
    ↓
[Agent ROS Bridge Execution]
    ↓
[Shadow Mode Logging]
    ↓
[Result + Safety Report]
```

---

## 🎮 Example Scenarios

### Scenario 1: Warehouse Robot

```python
# LangChain Agent
agent = create_ros_agent()

# Operator requests
commands = [
    "Navigate to aisle B",
    "Check if path is clear",
    "Pick up box at position A3",
    "Deliver to dock 5",
]

for cmd in commands:
    result = agent.invoke({"input": cmd})
    # Each command validated for safety
    # Human confirmation where required
    # Shadow mode logs decisions
```

### Scenario 2: Healthcare Assistant

```python
# Claude Desktop via MCP
# Nurse: "Bring medications to room 302"

# Claude interprets and executes:
# 1. ros_navigate to "room 302"
# 2. ros_get_status (verify arrival)
# 3. ros_manipulate (dispense medications)

# Safety: All manipulation requires confirmation
# Validation: Shadow mode tracks all interactions
```

### Scenario 3: Emergency Response

```python
# OpenClaw Bridge
user_input = "Emergency: Evacuate area immediately!"

result = await bridge.process_natural_language(user_input)
# → Safety Level: CRITICAL
# → Confirmation: Bypassed
# → Action: Emergency stop + evacuation route
```

---

## 📋 Running the Examples

### Prerequisites

```bash
# Install Agent ROS Bridge
pip install agent-ros-bridge

# For LangChain
pip install langchain langchain-openai

# For MCP (optional)
pip install mcp
```

### Run LangChain Example

```bash
export OPENAI_API_KEY=your-key
python langchain_integration.py
```

### Run OpenClaw Example

```bash
python openclaw_integration.py
```

### Run Natural Language Demo

```bash
python natural_language_examples.py
```

### Run Claude MCP Server

```bash
# Configure Claude Desktop to use this server
python claude_desktop_mcp_integration.py
```

---

## 🔧 Customization

### Adding New Natural Language Patterns

```python
# In your integration
patterns = {
    "my_custom_action": [
        "custom phrase {param}",
        "alternative phrasing",
    ]
}

def _handle_my_custom_action(self, text, original):
    # Extract parameters
    param = self._extract_param(text)
    
    # Execute with safety
    result = self.robot_agent.execute(f"custom({param})")
    
    return {
        "success": result.success,
        "action": "my_custom_action",
        "param": param,
    }
```

### Custom Safety Levels

```python
# Define custom safety logic
def assess_safety(natural_language: str, context: dict) -> str:
    if "emergency" in natural_language.lower():
        return "critical"
    elif context.get("humans_nearby"):
        return "high"
    elif "carefully" in natural_language.lower():
        return "medium"
    else:
        return "low"
```

---

## 📊 Metrics & Monitoring

### Shadow Mode Tracking

```python
# All AI agent interactions are logged
shadow_data = {
    "ai_proposal": "Navigate to kitchen",
    "human_decision": "Approved",
    "agreement": True,
    "context": {"urgency": "normal", "obstacles": []},
}

# Used for:
# - Safety validation
# - Gradual rollout
# - Agreement rate calculation
```

### Performance Metrics

```python
# Track AI agent performance
metrics = {
    "commands_executed": 156,
    "human_approvals": 142,
    "human_rejections": 14,
    "agreement_rate": 91.0,
    "avg_execution_time": 8.5,
}
```

---

## 🚀 Best Practices

### 1. Always Enforce Safety
```python
# ✅ Good: Safety enforced by default
agent = RobotAgent(require_confirmation=True)

# ❌ Bad: Disabling safety
agent = RobotAgent(require_confirmation=False)  # Don't do this
```

### 2. Context-Aware Instructions
```python
# ✅ Good: Include context
nlg.generate_navigation_command(
    destination="kitchen",
    context={"obstacles": ["chair"], "humans_nearby": True}
)

# ❌ Bad: Missing context
nlg.generate_navigation_command(destination="kitchen")
```

### 3. Clear Error Messages
```python
# ✅ Good: Actionable error
"Navigation failed: Path blocked by obstacle. Consider route: A→B→C"

# ❌ Bad: Vague error
"Error: Failed"
```

### 4. Gradual Autonomy
```python
# Start with 0% autonomy
safety.autonomous_mode = False

# Collect 200+ hours shadow data
safety.required_shadow_hours = 200.0

# Gradually increase
safety.gradual_rollout_stage = 10  # 10% autonomy
```

---

## 🔗 Integration Checklist

- [ ] Install Agent ROS Bridge
- [ ] Choose AI framework (LangChain/OpenClaw/Claude)
- [ ] Implement bridge/wrapper
- [ ] Define natural language patterns
- [ ] Configure safety levels
- [ ] Add emergency stop handling
- [ ] Test with simulated robot
- [ ] Validate shadow mode logging
- [ ] Deploy with human supervision
- [ ] Monitor agreement rates

---

## 📚 Further Reading

- [Agent ROS Bridge Documentation](../../docs/)
- [Safety Guidelines](../../docs/SAFETY.md)
- [API Reference](../../docs/API_REFERENCE.md)
- [Shadow Mode](../../docs/SHADOW_MODE.md)

---

## 🤝 Contributing

To add support for a new AI framework:

1. Create `{framework}_integration.py`
2. Implement bridge class
3. Add natural language patterns
4. Include safety validation
5. Write tests
6. Update this README

---

**Version:** v0.6.6  
**Last Updated:** April 8, 2026  
**Status:** Production Ready ✅
