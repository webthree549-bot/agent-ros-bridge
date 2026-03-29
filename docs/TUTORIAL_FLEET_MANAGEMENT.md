# Fleet Management API Tutorial

This tutorial shows how to manage multiple robots using Agent ROS Bridge's fleet orchestration capabilities.

## Prerequisites

```bash
pip install agent-ros-bridge
```

## Quick Start

### 1. Initialize Fleet

```python
from agent_ros_bridge import Bridge
from agent_ros_bridge.fleet import FleetOrchestrator, RobotCapability

# Create bridge
bridge = Bridge(ros_version=2)

# Initialize fleet orchestrator
fleet = FleetOrchestrator(
    redis_url="redis://localhost:6379/0",  # For distributed state
    max_robots=100
)

# Connect to bridge
await fleet.initialize(bridge)
```

### 2. Register Robots

```python
from agent_ros_bridge.fleet import FleetRobot, RobotCapability

# Register a mobile robot
robot_1 = FleetRobot(
    robot_id="turtlebot_001",
    name="Kitchen Bot",
    capabilities=[
        RobotCapability.NAVIGATION,
        RobotCapability.SENSING,
    ],
    ros_namespace="/robot_1",
    max_speed=0.5,  # m/s
)

# Register a manipulator
robot_2 = FleetRobot(
    robot_id="ur5_arm_001",
    name="Pick & Place Arm",
    capabilities=[
        RobotCapability.MANIPULATION,
        RobotCapability.SENSING,
    ],
    ros_namespace="/robot_2",
    payload_kg=5.0,
)

await fleet.register_robot(robot_1)
await fleet.register_robot(robot_2)
```

### 3. Submit Tasks

```python
from agent_ros_bridge.fleet import Task, TaskPriority

# Navigation task
task_1 = Task(
    task_id="nav_001",
    task_type="navigate",
    parameters={
        "target_location": "kitchen",
        "goal_pose": {"x": 5.0, "y": 3.0, "theta": 1.57},
    },
    priority=TaskPriority.HIGH,
    required_capabilities=[RobotCapability.NAVIGATION],
)

# Manipulation task
task_2 = Task(
    task_id="pick_001",
    task_type="pick_and_place",
    parameters={
        "object": "cup",
        "source_location": "table",
        "target_location": "sink",
    },
    priority=TaskPriority.NORMAL,
    required_capabilities=[RobotCapability.MANIPULATION],
)

# Submit tasks to fleet
await fleet.submit_task(task_1)
await fleet.submit_task(task_2)
```

### 4. Monitor Fleet Status

```python
# Get fleet overview
status = await fleet.get_fleet_status()
print(f"Active robots: {status.active_robots}")
print(f"Pending tasks: {status.pending_tasks}")
print(f"Completed tasks: {status.completed_tasks}")

# Get individual robot status
robot_status = await fleet.get_robot_status("turtlebot_001")
print(f"Location: {robot_status.location}")
print(f"Battery: {robot_status.battery_percent}%")
print(f"Current task: {robot_status.current_task}")
print(f"Is available: {robot_status.is_available}")
```

### 5. Task Assignment Strategies

```python
# Strategy 1: Capability-based assignment
await fleet.set_assignment_strategy("capability_match")

# Strategy 2: Closest robot
await fleet.set_assignment_strategy("closest")

# Strategy 3: Load balancing
await fleet.set_assignment_strategy("least_busy")

# Strategy 4: Energy efficient (battery-aware)
await fleet.set_assignment_strategy("energy_efficient")
```

### 6. Handle Task Results

```python
# Wait for task completion
result = await fleet.wait_for_task("nav_001", timeout=60.0)

if result.success:
    print(f"Task completed in {result.duration_sec}s")
    print(f"Robot used: {result.robot_id}")
else:
    print(f"Task failed: {result.error_message}")
    # Retry with different robot
    await fleet.retry_task("nav_001", exclude_robots=[result.robot_id])
```

### 7. Fleet Coordination Patterns

#### Pattern 1: Sequential Tasks

```python
# Robot must complete navigation before manipulation
async def sequential_mission():
    # Step 1: Navigate to pickup location
    nav_task = Task(
        task_id="nav_to_pickup",
        task_type="navigate",
        parameters={"target_location": "storage_area"},
        required_capabilities=[RobotCapability.NAVIGATION],
    )
    await fleet.submit_task(nav_task)
    nav_result = await fleet.wait_for_task("nav_to_pickup")
    
    if nav_result.success:
        # Step 2: Pick object
        pick_task = Task(
            task_id="pick_object",
            task_type="pick",
            parameters={"object": "box", "robot_id": nav_result.robot_id},
            required_capabilities=[RobotCapability.MANIPULATION],
        )
        await fleet.submit_task(pick_task)
```

#### Pattern 2: Parallel Tasks

```python
# Multiple robots work simultaneously
async def parallel_mission():
    tasks = []
    
    # Robot 1: Clean room A
    tasks.append(fleet.submit_task(Task(
        task_id="clean_a",
        task_type="clean_room",
        parameters={"room": "A"},
    )))
    
    # Robot 2: Clean room B
    tasks.append(fleet.submit_task(Task(
        task_id="clean_b",
        task_type="clean_room",
        parameters={"room": "B"},
    )))
    
    # Wait for all
    results = await asyncio.gather(*tasks)
    return all(r.success for r in results)
```

#### Pattern 3: Cooperative Tasks

```python
# Multiple robots work together on same task
async def cooperative_lift():
    # Task requires 2 robots
    task = Task(
        task_id="heavy_lift",
        task_type="cooperative_lift",
        parameters={"object_weight_kg": 50},
        required_capabilities=[RobotCapability.MANIPULATION],
        min_robots=2,
        max_robots=2,
    )
    await fleet.submit_task(task)
    result = await fleet.wait_for_task("heavy_lift")
```

### 8. Error Handling & Recovery

```python
from agent_ros_bridge.fleet import FleetError

async def robust_task_execution():
    try:
        result = await fleet.execute_task(task)
    except FleetError.RobotOffline as e:
        print(f"Robot {e.robot_id} went offline, reassigning...")
        await fleet.reassign_task(task.task_id)
    except FleetError.NoCapableRobot:
        print("No robot can perform this task")
        await fleet.queue_task(task)  # Wait for suitable robot
    except FleetError.TaskTimeout:
        print("Task timed out, retrying...")
        await fleet.retry_task(task.task_id)
```

### 9. Fleet Metrics

```python
from agent_ros_bridge.fleet import FleetMetrics

# Get performance metrics
metrics = await fleet.get_metrics(time_window_hours=24)

print(f"Tasks completed: {metrics.tasks_completed}")
print(f"Success rate: {metrics.success_rate:.1%}")
print(f"Average task duration: {metrics.avg_task_duration_sec:.1f}s")
print(f"Fleet utilization: {metrics.fleet_utilization:.1%}")
print(f"Energy consumption: {metrics.total_energy_kwh:.2f} kWh")

# Robot-specific metrics
robot_metrics = await fleet.get_robot_metrics("turtlebot_001")
print(f"Distance traveled: {robot_metrics.distance_km:.2f} km")
print(f"Charging cycles: {robot_metrics.charging_cycles}")
```

### 10. Natural Language Fleet Control

```python
# Use AI to control fleet
result = await bridge.execute_intent(
    "Send the closest robot to the kitchen and have it pick up the cup",
    fleet_context=fleet.get_context()
)

# AI automatically:
# 1. Finds closest capable robot
# 2. Assigns navigation task
# 3. Assigns manipulation task
# 4. Returns combined result
```

## Advanced Topics

### Custom Task Types

```python
from agent_ros_bridge.fleet import Task, TaskExecutor

class CustomTaskExecutor(TaskExecutor):
    async def execute(self, task: Task, robot: FleetRobot) -> TaskResult:
        # Custom execution logic
        if task.task_type == "my_custom_task":
            # Implement custom behavior
            pass
        return TaskResult(success=True)

# Register executor
fleet.register_task_executor("my_custom_task", CustomTaskExecutor())
```

### Dynamic Reconfiguration

```python
# Add robot to fleet at runtime
new_robot = FleetRobot(robot_id="emergency_bot_01", ...)
await fleet.register_robot(new_robot)

# Remove robot for maintenance
await fleet.unregister_robot("robot_001", reason="maintenance")

# Update robot capabilities
await fleet.update_robot_capabilities(
    "robot_001",
    add_capabilities=[RobotCapability.CHARGING],
)
```

## Summary

The fleet management API provides:
- ✅ Multi-robot task distribution
- ✅ Capability-based assignment
- ✅ Real-time monitoring
- ✅ Error recovery
- ✅ Natural language integration

See `examples/fleet/` for complete working examples.
