# API Documentation

## Agent ROS Bridge v0.6.4

Complete API reference for all modules.

---

## Table of Contents

- [Core Gateway](#core-gateway)
- [Shadow Mode](#shadow-mode)
- [Simulation](#simulation)
- [AI/LLM](#aillm)
- [Safety](#safety)
- [UI](#ui)

---

## Core Gateway

### Gateway

Main entry point for robot control.

```python
from agent_ros_bridge.gateway import AgentGateway

gateway = AgentGateway()
gateway.start()
```

#### Methods

##### `send_command(robot_id: str, command: dict) -> dict`

Send command to robot.

**Parameters:**
- `robot_id` (str): Robot identifier
- `command` (dict): Command with 'type' and 'parameters'

**Returns:**
- `dict`: Result with 'success' and 'message'

**Example:**
```python
result = gateway.send_command(
    robot_id='bot1',
    command={
        'type': 'navigate_to',
        'parameters': {'x': 5.0, 'y': 5.0}
    }
)
```

##### `get_robot_status(robot_id: str) -> dict`

Get current robot status.

**Parameters:**
- `robot_id` (str): Robot identifier

**Returns:**
- `dict`: Status with position, velocity, battery, etc.

---

## Shadow Mode

### ShadowModeCollector

Collect AI vs human decision data.

```python
from agent_ros_bridge.shadow.collector import ShadowModeCollector

collector = ShadowModeCollector(
    output_dir="shadow_data",
    target_hours=200.0
)
collector.start_collection()
```

#### Methods

##### `on_ai_proposal(robot_id, intent_type, confidence, entities, reasoning)`

Log an AI proposal.

**Parameters:**
- `robot_id` (str): Robot identifier
- `intent_type` (str): Type of intent (e.g., 'NAVIGATE')
- `confidence` (float): AI confidence (0-1)
- `entities` (list): Extracted entities
- `reasoning` (str): AI reasoning

**Example:**
```python
collector.on_ai_proposal(
    robot_id='bot1',
    intent_type='NAVIGATE',
    confidence=0.95,
    entities=[{'type': 'LOCATION', 'value': 'kitchen'}],
    reasoning='User wants to go to kitchen'
)
```

##### `on_human_decision(robot_id, command, parameters, matched_ai_proposal)`

Log a human decision.

**Parameters:**
- `robot_id` (str): Robot identifier
- `command` (str): Command executed
- `parameters` (dict): Command parameters
- `matched_ai_proposal` (bool): Whether it matched AI proposal

**Example:**
```python
collector.on_human_decision(
    robot_id='bot1',
    command='navigate_to',
    parameters={'location': 'kitchen'},
    matched_ai_proposal=True
)
```

##### `get_status() -> dict`

Get collection status.

**Returns:**
```python
{
    'total_decisions': 1000,
    'ai_proposals': 500,
    'human_decisions': 500,
    'agreements': 450,
    'agreement_rate': 0.9,
    'hours_elapsed': 50.0,
    'hours_remaining': 150.0,
    'percent_complete': 25.0,
    'is_running': True
}
```

##### `export_data(format='json') -> Path`

Export collected data.

**Parameters:**
- `format` (str): 'json' or 'csv'

**Returns:**
- `Path`: Path to exported file

### ShadowModeHooks

Hooks for auto-logging decisions.

```python
from agent_ros_bridge.shadow.hooks import ShadowModeHooks, enable_shadow_mode

# Enable on existing components
hooks = enable_shadow_mode(
    intent_parser=my_parser,
    gateway=my_gateway
)
```

#### Methods

##### `on_intent_parsed(robot_id, intent_result)`

Hook for intent parsing.

##### `on_human_command(command)`

Hook for human commands.

##### `get_stats() -> dict`

Get statistics.

**Returns:**
```python
{
    'total_decisions': 100,
    'agreement_rate': 0.85,
    'avg_confidence': 0.92
}
```

---

## Simulation

### ScenarioGenerator

Generate scenarios for validation.

```python
from agent_ros_bridge.simulation.scenario_generator import ScenarioGenerator

gen = ScenarioGenerator(output_dir="scenarios")
```

#### Methods

##### `generate_scenario(template, seed, difficulty) -> dict`

Generate single scenario.

**Parameters:**
- `template` (str): 'navigation', 'manipulation', 'safety'
- `seed` (int): Random seed for reproducibility
- `difficulty` (str): 'easy', 'medium', 'hard'

**Returns:**
```python
{
    'name': 'navigation_000042_medium',
    'robot_config': {...},
    'environment': {...},
    'goal': {...}
}
```

##### `generate_batch(template, count, difficulty) -> list`

Generate multiple scenarios.

**Parameters:**
- `template` (str): Template name
- `count` (int): Number of scenarios
- `difficulty` (str): Difficulty level

**Returns:**
- `list`: List of scenario dicts

### GazeboBatchRunner

Run scenarios in parallel Gazebo worlds.

```python
from agent_ros_bridge.simulation.gazebo_batch import GazeboBatchRunner

runner = GazeboBatchRunner(
    num_worlds=8,
    headless=True
)
```

#### Methods

##### `run_batch(scenarios, progress_callback) -> list`

Execute scenarios in parallel.

**Parameters:**
- `scenarios` (list): List of scenario file paths
- `progress_callback` (callable): Called with (completed, total)

**Returns:**
- `list`: List of WorldResult objects

**Example:**
```python
def progress(completed, total):
    print(f"{completed}/{total} ({completed/total*100:.1f}%)")

results = runner.run_batch(
    scenario_files=['s1.yaml', 's2.yaml', ...],
    progress_callback=progress
)
```

### Scenario10KGenerator

Validate Gate 2 with 10K scenarios.

```python
from agent_ros_bridge.validation.scenario_10k import Scenario10KGenerator

gen = Scenario10KGenerator()
validation = gen.run_full_validation(count=10000)
```

#### Methods

##### `run_full_validation(count, progress_callback) -> dict`

Run complete validation.

**Parameters:**
- `count` (int): Number of scenarios (default: 10000)
- `progress_callback` (callable): Progress callback

**Returns:**
```python
{
    'total_scenarios': 10000,
    'successful': 9593,
    'failed': 407,
    'success_rate': 0.9593,
    'gate2_passed': True,
    'total_safety_violations': 0
}
```

---

## AI/LLM

### IntentParser

Parse natural language to robot commands.

```python
from agent_ros_bridge.ai.intent_parser import IntentParser

parser = IntentParser()
```

#### Methods

##### `parse(text, robot_id) -> dict`

Parse intent from text.

**Parameters:**
- `text` (str): Natural language input
- `robot_id` (str): Robot identifier

**Returns:**
```python
{
    'intent_type': 'NAVIGATE',
    'confidence': 0.95,
    'entities': [{'type': 'LOCATION', 'value': 'kitchen'}],
    'reasoning': 'User wants to navigate to kitchen'
}
```

### LLMParser

LLM-based intent parsing.

```python
from agent_ros_bridge.ai.llm_parser import OpenAIParser, MoonshotParser

# OpenAI
parser = OpenAIParser(api_key='your-key')

# Moonshot (Kimi)
parser = MoonshotParser(api_key='your-key')
```

#### Methods

##### `parse(text, robot_id) -> dict`

Parse with LLM.

---

## Safety

### SafetyValidator

Validate commands for safety.

```python
from agent_ros_bridge.safety.safety_validator import SafetyValidator

validator = SafetyValidator(
    max_velocity=1.0,
    max_acceleration=0.5
)
```

#### Methods

##### `validate(command) -> dict`

Validate command.

**Parameters:**
- `command` (dict): Command to validate

**Returns:**
```python
{
    'safe': True,
    'violations': []
}
```

---

## UI

### ConfirmationUI

Web interface for human confirmation.

```python
from agent_ros_bridge.ui.confirmation import ConfirmationUI

ui = ConfirmationUI(port=8080)
ui.start_server()
```

#### Methods

##### `receive_proposal(proposal_data) -> str`

Receive AI proposal for confirmation.

**Parameters:**
- `proposal_data` (dict): Proposal details

**Returns:**
- `str`: Proposal ID

##### `approve_proposal(proposal_id) -> dict`

Approve proposal.

**Parameters:**
- `proposal_id` (str): Proposal ID

**Returns:**
- `dict`: Result

##### `reject_proposal(proposal_id, reason) -> dict`

Reject proposal.

**Parameters:**
- `proposal_id` (str): Proposal ID
- `reason` (str): Rejection reason

##### `modify_proposal(proposal_id, original, modified) -> dict`

Modify and approve proposal.

**Parameters:**
- `proposal_id` (str): Proposal ID
- `original` (dict): Original parameters
- `modified` (dict): Modified parameters

### REST API Endpoints

#### GET `/api/pending`

Get pending proposals.

**Response:**
```json
[
  {
    "proposal_id": "prop_123",
    "robot_id": "bot1",
    "intent_type": "NAVIGATE",
    "confidence": 0.95
  }
]
```

#### POST `/api/approve/{proposal_id}`

Approve a proposal.

**Response:**
```json
{
  "success": true
}
```

#### POST `/api/reject/{proposal_id}`

Reject a proposal.

**Request:**
```json
{
  "reason": "unsafe"
}
```

**Response:**
```json
{
  "success": true
}
```

#### GET `/api/metrics`

Get current metrics.

**Response:**
```json
{
  "total_decisions": 100,
  "agreement_rate": 0.85,
  "avg_confidence": 0.92
}
```

---

## Configuration

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DOMAIN_ID` | ROS2 domain ID | `0` |
| `GAZEBO_MASTER_URI` | Gazebo master URI | `http://localhost:11345` |
| `OPENAI_API_KEY` | OpenAI API key | - |
| `ANTHROPIC_API_KEY` | Anthropic API key | - |
| `MOONSHOT_API_KEY` | Moonshot API key | - |
| `SHADOW_DATA_DIR` | Shadow data directory | `./shadow_data` |
| `SHADOW_TARGET_HOURS` | Target collection hours | `200` |
| `LOG_LEVEL` | Logging level | `INFO` |

### Configuration File

YAML configuration at `/etc/agent-ros-bridge/config.yaml`:

```yaml
gateway:
  host: "0.0.0.0"
  port: 8080

shadow_mode:
  enabled: true
  confidence_threshold: 0.7
  require_confirmation: true

simulation:
  enabled: true
  num_worlds: 4
  headless: true

llm:
  default_provider: "moonshot"
```

---

## Examples

### Complete Workflow

```python
from agent_ros_bridge.gateway import AgentGateway
from agent_ros_bridge.shadow.collector import ShadowModeCollector
from agent_ros_bridge.ui.confirmation import ConfirmationUI

# Initialize components
gateway = AgentGateway()
collector = ShadowModeCollector(output_dir="shadow_data")
ui = ConfirmationUI(port=8080)

# Start collection
collector.start_collection()
ui.start_server()

# Handle AI proposal
proposal_id = ui.receive_proposal({
    'robot_id': 'bot1',
    'intent_type': 'NAVIGATE',
    'confidence': 0.95,
    'entities': [{'type': 'LOCATION', 'value': 'kitchen'}]
})

# Log to collector
collector.on_ai_proposal(
    robot_id='bot1',
    intent_type='NAVIGATE',
    confidence=0.95,
    entities=[...],
    reasoning='...'
)

# Operator approves via UI
ui.approve_proposal(proposal_id)

# Log human decision
collector.on_human_decision(
    robot_id='bot1',
    command='navigate_to',
    parameters={'location': 'kitchen'},
    matched_ai_proposal=True
)
```

---

## Support

- **Issues**: https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation**: https://github.com/webthree549-bot/agent-ros-bridge/tree/main/docs
- **PyPI**: https://pypi.org/project/agent-ros-bridge/
