# Agent ROS Bridge Tutorial

Complete tutorial from basics to advanced features.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Basic Robot Control](#basic-robot-control)
3. [Safety First](#safety-first)
4. [Fleet Management](#fleet-management)
5. [AI Agent Integration](#ai-agent-integration)
6. [Learning & Memory](#learning--memory)
7. [Advanced Missions](#advanced-missions)
8. [Multi-Robot Coordination](#multi-robot-coordination)
9. [Real-Time Monitoring](#real-time-monitoring)
10. [Best Practices](#best-practices)

---

## Getting Started

### Installation

```bash
pip install agent-ros-bridge
```

### First Robot

```python
from agent_ros_bridge import RobotAgent

# Create robot with safety
robot = RobotAgent(
    device_id="my_first_robot",
    device_type="mobile_robot",
    require_confirmation=True,  # Always confirm risky actions
)

print(f"Robot created: {robot.device_id}")
print(f"Safety: {robot.safety.human_in_the_loop}")
```

---

## Basic Robot Control

### Natural Language Commands

```python
# Simple navigation
result = robot.execute("Go to the kitchen")
print(result.message)  # "Successfully navigated to kitchen"

# Compound tasks
result = robot.execute("Go to kitchen and pick up the red cup")
print(f"Steps: {len(result.steps)}")

# With qualifiers
result = robot.execute("Navigate to the office carefully")
```

### Object Interaction

```python
# Recognize objects
objects = robot.recognize_objects()
for obj in objects:
    print(f"Found: {obj['name']} ({obj['confidence']:.0%})")

# Pick specific object
result = robot.pick_object("red_cube")
if result.success:
    print("Object picked successfully!")
```

### Health Monitoring

```python
# Check robot health
health = robot.health_check()

print(f"Status: {health['status']}")
print(f"Battery: {health['battery']['level']}%")
print(f"Warnings: {health['warnings']}")
```

---

## Safety First

### Understanding Safety Levels

```python
from agent_ros_bridge.gateway_v2.config import SafetyConfig

# Default: Maximum safety
config = SafetyConfig()
print(config.autonomous_mode)  # False - human approval required
print(config.human_in_the_loop)  # True

# Gradual rollout
config = SafetyConfig(
    autonomous_mode=True,
    min_confidence_for_auto=0.95,  # 95% confidence needed
    gradual_rollout_stage=10,  # 10% autonomous
)
```

### Safety Validation

```python
from agent_ros_bridge.safety.validator import SafetyValidator

validator = SafetyValidator()

# Check if velocity is safe
result = validator.validate_velocity({
    "linear": {"x": 0.5},
    "angular": {"z": 0.1},
})

if not result.is_safe:
    print(f"Unsafe: {result.reason}")
```

---

## Fleet Management

### Creating a Fleet

```python
from agent_ros_bridge.fleet.orchestrator import FleetOrchestrator
from agent_ros_bridge.fleet.robot import FleetRobot

fleet = FleetOrchestrator()

# Add robots
fleet.robots["bot1"] = FleetRobot(
    robot_id="bot1",
    name="Explorer",
    battery_percent=95.0,
)

fleet.robots["bot2"] = FleetRobot(
    robot_id="bot2",
    name="Worker",
    battery_percent=80.0,
)
```

### Fleet Status

```python
# Get complete status
status = fleet.get_fleet_status()

print(f"Total robots: {status['total']}")
print(f"Online: {status['online']}")
print(f"Avg battery: {status['battery_avg']}%")

# Individual robot status
for robot_data in status['robots']:
    print(f"{robot_data['id']}: {robot_data['status']}")
```

### Emergency Procedures

```python
# Emergency stop all
result = await fleet.emergency_stop_all()
print(f"Stopped: {result['robots_stopped']} robots")

# Return to base
result = await fleet.emergency_return_to_base()
print(f"Returning: {result['robots_returning']} robots")
```

---

## AI Agent Integration

### OpenClaw Integration

```python
from examples.ai_agent_integrations.openclaw_integration import OpenClawROSBridge

bridge = OpenClawROSBridge(robot)

# Process natural language
result = await bridge.process_natural_language("Go to the kitchen")
print(f"Action: {result['action']}")
print(f"Safety: {result['safety_level']}")
```

### LangChain Integration

```python
from examples.ai_agent_integrations.langchain_integration import create_ros_agent

agent = create_ros_agent()

# Use with LangChain
result = agent.invoke({"input": "Navigate to the charging station"})
```

### Natural Language Interpreter

```python
from examples.ai_agent_integrations.natural_language_examples import (
    NaturalLanguageInterpreter,
    NaturalLanguageGenerator,
)

# Interpret commands
nli = NaturalLanguageInterpreter(robot)
result = nli.interpret("Pick up the cup on the table")

# Generate instructions
nlg = NaturalLanguageGenerator(robot)
cmd = nlg.generate_navigation_command("kitchen", context={"obstacles": ["chair"]})
print(cmd.natural_language)
# "Please navigate to kitchen, carefully avoiding chair."
```

---

## Learning & Memory

### Storing Paths

```python
from agent_ros_bridge.learning.memory import RobotMemory

memory = RobotMemory(robot_id="bot1")

# Store successful path
memory.store_path({
    "start": "kitchen",
    "end": "living_room",
    "waypoints": [(0, 0), (1, 0), (2, 0)],
    "duration": 12.5,
    "success": True,
})

# Retrieve best path
path = memory.get_path("kitchen", "living_room")
print(f"Best time: {path['duration']}s")
```

### Learning from Failures

```python
# Record failure
memory.record_failure(
    task="navigate",
    location="narrow_corridor",
    reason="collision",
)

# Check if risky
if memory.is_location_risky("narrow_corridor"):
    print("Avoiding narrow corridor - previous collision")
```

### Adapting to Changes

```python
# Corridor now blocked
memory.update_path_status("main_corridor", blocked=True)

# Will no longer suggest paths through main_corridor
paths = memory.get_all_paths("A", "B")
```

---

## Advanced Missions

### Conditional Missions

```python
from agent_ros_bridge.learning.mission import AdvancedMissionPlanner

planner = AdvancedMissionPlanner()

# Mission with conditions
mission = {
    "steps": [
        {
            "action": "check_battery",
            "if": {"battery": {"lt": 20}},
            "then": [{"action": "go_charge"}],
            "else": [{"action": "continue_mission"}],
        }
    ]
}

# Evaluate with low battery
result = planner.evaluate_condition(
    mission["steps"][0],
    context={"battery": 15}
)
# Returns: "then" (go charge)
```

### Parallel Tasks

```python
# Tasks that can run in parallel
mission = {
    "parallel": [
        {"action": "check_sensors"},
        {"action": "update_status"},
        {"action": "log_position"},
    ]
}

parallel_tasks = planner.get_parallel_tasks(mission)
```

### Mission Optimization

```python
# Optimize task order
tasks = [
    {"action": "go_to_A", "location": (5, 5)},
    {"action": "go_to_B", "location": (1, 1)},
    {"action": "go_to_C", "location": (0, 0)},
]

# Starting from (0,0)
optimized = planner.optimize_task_order(tasks, (0, 0))
# Result: C (already there), B (close), A (far)
```

---

## Multi-Robot Coordination

### Swarm Formations

```python
from agent_ros_bridge.fleet.swarm import SwarmCoordinator

coordinator = SwarmCoordinator()

# Line formation
robots = ["bot1", "bot2", "bot3", "bot4"]
positions = coordinator.calculate_formation(
    robots=robots,
    formation_type="line",
    spacing=2.0,
)

# Circle formation
positions = coordinator.calculate_formation(
    robots=robots,
    formation_type="circle",
    spacing=2.0,
)
```

### Task Sharing

```python
# Heavy task - should be shared
heavy_task = {"type": "lift", "weight": 50}
should_share = coordinator.should_share_task(
    task=heavy_task,
    current_robot="bot1",
    available_robots=["bot2", "bot3"],
)
# Returns: True (weight > 30)
```

### Collision Avoidance

```python
# Check collision risk
robot1 = {"x": 0, "y": 0, "vx": 1, "vy": 0}
robot2 = {"x": 3, "y": 0, "vx": -1, "vy": 0}

risk = coordinator.check_collision_risk(robot1, robot2)
print(f"Risk: {risk}")  # 0.0 to 1.0

# Get avoidance suggestion
if risk > 0.5:
    avoidance = coordinator.suggest_avoidance(robot1, robot2)
    print(f"Action: {avoidance['action']}")
```

---

## Real-Time Monitoring

### Dashboard Setup

```python
from agent_ros_bridge.dashboard.realtime import RealTimeDashboard

dashboard = RealTimeDashboard()

# Update with fleet data
dashboard.update_fleet_status({
    "robots": [
        {"id": "bot1", "status": "active", "battery": 85},
        {"id": "bot2", "status": "charging", "battery": 20},
    ]
})

# Get current status
status = dashboard.get_current_status()
```

### Task Visualization

```python
tasks = [
    {"id": "task1", "robot": "bot1", "progress": 75, "status": "executing"},
    {"id": "task2", "robot": "bot2", "progress": 100, "status": "completed"},
]

viz = dashboard.visualize_tasks(tasks)
print(viz["task1"]["color"])  # "blue" (executing)
```

### Performance Metrics

```python
metrics = dashboard.get_performance_metrics()

print(f"Avg completion time: {metrics['avg_task_completion_time']}s")
print(f"Fleet utilization: {metrics['fleet_utilization']}%")
print(f"Success rate: {metrics['success_rate']}%")
```

### Anomaly Detection

```python
# Check for anomalies
robot_data = {"id": "bot1", "battery": 5, "status": "active"}
alerts = dashboard.check_anomalies(robot_data)

for alert in alerts:
    print(f"ALERT: {alert}")
# "CRITICAL: Robot bot1 battery at 5%"
```

---

## Best Practices

### 1. Always Use Safety

```python
# Good
robot = RobotAgent(
    device_id="bot1",
    require_confirmation=True,  # ✅ Safe
)

# Bad
robot = RobotAgent(
    device_id="bot1",
    require_confirmation=False,  # ❌ Risky without validation
)
```

### 2. Handle Failures Gracefully

```python
result = robot.execute("command")

if not result.success:
    # Log failure
    robot.memory.record_failure(
        task="command",
        location=result.data.get("location"),
        reason=result.message,
    )
    # Try alternative
    alternative = robot.memory.suggest_alternative()
```

### 3. Use Context in Conversations

```python
conv = ConversationManager()

# First command
conv.process_input("Go to the kitchen")

# Second command uses context
result = conv.process_input("Check if it's clean")
# Automatically resolves "it" to "kitchen"
```

### 4. Monitor Fleet Health

```python
# Regular health checks
for robot_id, robot in fleet.robots.items():
    health = robot.health_check()
    if health["status"] != "healthy":
        alerts = dashboard.check_anomalies({
            "id": robot_id,
            **health,
        })
        send_alerts(alerts)
```

### 5. Optimize Missions

```python
# Always optimize task order
optimized = planner.optimize_task_order(tasks, start_location)

# Use parallel execution when possible
mission = {"parallel": independent_tasks}
```

---

## Next Steps

- [API Reference](../api/README.md)
- [Safety Guidelines](../SAFETY.md)
- [Examples](../../examples/README.md)
- [Performance Tuning](./performance.md)

---

**Happy robot programming! 🤖**
