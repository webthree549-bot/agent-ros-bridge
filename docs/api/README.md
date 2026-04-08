# API Reference

Complete API documentation for Agent ROS Bridge.

## Core Classes

### RobotAgent

Main interface for robot control.

```python
from agent_ros_bridge import RobotAgent

robot = RobotAgent(
    device_id="bot1",
    device_type="mobile_robot",
    require_confirmation=True,
)
```

#### Methods

##### `execute(command: str) -> TaskResult`
Execute a natural language command.

```python
result = robot.execute("Navigate to the kitchen")
print(result.success)  # True
print(result.message)  # "Successfully navigated to kitchen"
```

##### `health_check() -> dict`
Get robot health diagnostics.

```python
health = robot.health_check()
print(health["battery"]["level"])  # 85.0
print(health["status"])  # "healthy"
```

##### `recognize_objects() -> list`
Recognize objects in environment.

```python
objects = robot.recognize_objects()
# [{"name": "red_cube", "confidence": 0.95, ...}]
```

##### `pick_object(name: str) -> TaskResult`
Pick up an object by name.

```python
result = robot.pick_object("red_cube")
```

##### `plan_mission(description: str) -> dict`
Create mission plan from natural language.

```python
mission = robot.plan_mission("Clean kitchen and check living room")
print(mission["tasks"])  # List of tasks
print(mission["estimated_duration"])  # 180 seconds
```

---

## Fleet Management

### FleetOrchestrator

Manage multiple robots.

```python
from agent_ros_bridge.fleet.orchestrator import FleetOrchestrator

fleet = FleetOrchestrator()
```

#### Methods

##### `get_fleet_status() -> dict`
Get complete fleet status.

```python
status = fleet.get_fleet_status()
print(status["total"])  # 5 robots
print(status["online"])  # 4 robots online
print(status["battery_avg"])  # 78.5
```

##### `emergency_stop_all() -> dict`
Emergency stop all robots.

```python
result = await fleet.emergency_stop_all()
print(result["robots_stopped"])  # 5
```

---

## Learning & Memory

### RobotMemory

Store and retrieve learned information.

```python
from agent_ros_bridge.learning.memory import RobotMemory

memory = RobotMemory(robot_id="bot1")
```

#### Methods

##### `store_path(path: dict)`
Store a successful path.

```python
memory.store_path({
    "start": "kitchen",
    "end": "living_room",
    "waypoints": [(0, 0), (1, 0)],
    "duration": 15.5,
    "success": True,
})
```

##### `get_path(start: str, end: str) -> dict`
Get best path between locations.

```python
path = memory.get_path("kitchen", "living_room")
print(path["duration"])  # 15.5
```

---

## Advanced Mission Planning

### AdvancedMissionPlanner

Complex mission planning with conditionals.

```python
from agent_ros_bridge.learning.mission import AdvancedMissionPlanner

planner = AdvancedMissionPlanner()
```

#### Methods

##### `evaluate_condition(step: dict, context: dict) -> str`
Evaluate if/then conditions.

```python
step = {
    "if": {"battery": {"lt": 20}},
    "then": [{"action": "go_charge"}],
    "else": [{"action": "continue"}],
}
result = planner.evaluate_condition(step, {"battery": 15})
# Returns: "then"
```

##### `optimize_task_order(tasks: list, start_location: tuple) -> list`
Optimize task ordering.

```python
optimized = planner.optimize_task_order(tasks, (0, 0))
```

---

## Multi-Robot Coordination

### SwarmCoordinator

Coordinate multiple robots.

```python
from agent_ros_bridge.fleet.swarm import SwarmCoordinator

coordinator = SwarmCoordinator()
```

#### Methods

##### `calculate_formation(robots: list, formation_type: str, spacing: float)`
Calculate swarm formation.

```python
positions = coordinator.calculate_formation(
    robots=["bot1", "bot2", "bot3"],
    formation_type="circle",
    spacing=2.0,
)
```

---

## Dashboard

### RealTimeDashboard

Monitor fleet in real-time.

```python
from agent_ros_bridge.dashboard.realtime import RealTimeDashboard

dashboard = RealTimeDashboard()
```

#### Methods

##### `update_fleet_status(fleet_data: dict)`
Update dashboard with fleet status.

```python
dashboard.update_fleet_status({
    "robots": [{"id": "bot1", "status": "active", "battery": 85}],
})
```

---

## Natural Language

### ConversationManager

Handle multi-turn conversations.

```python
from agent_ros_bridge.nlp.conversation import ConversationManager

conv = ConversationManager()
```

#### Methods

##### `process_input(user_input: str) -> dict`
Process natural language with context.

```python
result = conv.process_input("Go to the kitchen")
# Later...
result = conv.process_input("Check if it's clean")
# Resolves "it" to "kitchen"
```

---

## Safety

### SafetyValidator

Validate commands for safety.

```python
from agent_ros_bridge.safety.validator import SafetyValidator

validator = SafetyValidator()
```

#### Methods

##### `validate_velocity(cmd: dict) -> ValidationResult`
Check if velocity is safe.

```python
result = validator.validate_velocity({
    "linear": {"x": 0.5},
    "angular": {"z": 0.1},
})
print(result.is_safe)  # True
```

---

## Tools

### ROSTopicEchoTool

Read ROS topics.

```python
from agent_ros_bridge.tools import ROSTopicEchoTool

tool = ROSTopicEchoTool()
result = tool.execute(topic="/cmd_vel", count=10)
```

### ROSServiceCallTool

Call ROS services.

```python
from agent_ros_bridge.tools import ROSServiceCallTool

tool = ROSServiceCallTool()
result = tool.execute(service="/clear_costmap")
```

---

## Exceptions

### AgentROSBridgeError

Base exception.

```python
from agent_ros_bridge.exceptions import AgentROSBridgeError

try:
    robot.execute("command")
except AgentROSBridgeError as e:
    print(e.message)
```

### SafetyValidationError

Safety violation.

```python
from agent_ros_bridge.exceptions import SafetyValidationError

try:
    risky_operation()
except SafetyValidationError as e:
    print(e.violation_type)
```

---

## Configuration

### SafetyConfig

Safety configuration.

```python
from agent_ros_bridge.gateway_v2.config import SafetyConfig

config = SafetyConfig(
    autonomous_mode=False,  # Human approval required
    human_in_the_loop=True,
    shadow_mode_enabled=True,
)
```

---

## See Also

- [Quick Start Guide](../guides/quickstart.md)
- [Safety Guidelines](../SAFETY.md)
- [Examples](../../examples/README.md)
