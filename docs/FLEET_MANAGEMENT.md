# Fleet Management Guide

**Version:** v0.6.7  
**Last Updated:** 2026-04-10

---

## Overview

Agent ROS Bridge provides fleet management capabilities for coordinating multiple robots. This guide covers the current implementation and best practices.

---

## Current Capabilities

### Robot Discovery

```python
from agent_ros_bridge import RobotAgent

# Discover all robots on the network
robots = RobotAgent.discover_all()
for robot in robots:
    print(f"Found: {robot.device_id} ({robot.device_type})")
```

### Fleet Status Monitoring

```python
# Get status of all robots
from agent_ros_bridge.fleet import FleetOrchestrator

fleet = FleetOrchestrator()
status = await fleet.get_fleet_status()

for robot_id, info in status.items():
    print(f"{robot_id}: {info['status']}, Battery: {info['battery']}%")
```

### Robot Selection

```python
# Select robot by criteria
best_robot = fleet.select_robot(
    criteria="battery",  # or "distance", "capability"
    target_location=(10, 20),
    required_capabilities=["navigation", "manipulation"]
)
```

### Task Distribution

```python
# Send command to specific robot
result = await fleet.send_command(
    robot_id="bot1",
    command="navigate_to",
    parameters={"x": 10, "y": 20}
)

# Broadcast to all robots
results = await fleet.broadcast_command(
    command="emergency_stop"
)
```

---

## Architecture

### FleetOrchestrator

The `FleetOrchestrator` manages multiple robots:

```
┌─────────────────────────────────────────────────────────────┐
│                    FleetOrchestrator                        │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Robot 1    │  │   Robot 2    │  │   Robot N    │      │
│  │  Controller  │  │  Controller  │  │  Controller  │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                 │                 │               │
│         └─────────────────┼─────────────────┘               │
│                           │                                 │
│                    ┌──────▼──────┐                         │
│                    │   Fleet     │                         │
│                    │   State     │                         │
│                    └─────────────┘                         │
└─────────────────────────────────────────────────────────────┘
```

### FleetRobot

Each robot in the fleet is represented by a `FleetRobot`:

```python
@dataclass
class FleetRobot:
    robot_id: str
    robot_type: str
    status: RobotStatus
    capabilities: list[RobotCapability]
    current_task: Task | None
    battery_level: float
    position: tuple[float, float, float]
```

---

## Task Management

### Task Types

| Task Type | Description | Example |
|-----------|-------------|---------|
| `navigation` | Move to location | "Go to kitchen" |
| `manipulation` | Pick/place objects | "Pick up box" |
| `inspection` | Sensor-based task | "Check temperature" |
| `patrol` | Repeated route | "Patrol hourly" |
| `custom` | User-defined | Any custom action |

### Task Priority

```python
from agent_ros_bridge.fleet import Task, TaskPriority

task = Task(
    task_type="navigation",
    parameters={"x": 10, "y": 20},
    priority=TaskPriority.HIGH,  # LOW, MEDIUM, HIGH, CRITICAL
    deadline=datetime.now() + timedelta(minutes=5)
)

await fleet.assign_task(robot_id="bot1", task=task)
```

---

## Multi-Robot Coordination

### Search Pattern

```python
# Divide area among robots
area = {"x_min": 0, "x_max": 100, "y_min": 0, "y_max": 100}
robots = ["bot1", "bot2", "bot3"]

await fleet.coordinated_search(
    robots=robots,
    area=area,
    pattern="parallel"  # or "spiral", "random"
)
```

### Convoy Formation

```python
# Follow-the-leader pattern
await fleet.form_convoy(
    leader="bot1",
    followers=["bot2", "bot3"],
    spacing=2.0  # meters between robots
)
```

### Area Coverage

```python
# Distributed coverage
await fleet.cover_area(
    robots=["bot1", "bot2", "bot3"],
    area=area,
    algorithm="boustrophedon"  # or "spiral", "random"
)
```

---

## Fleet Metrics

### Real-time Metrics

```python
from agent_ros_bridge.fleet import FleetMetrics

metrics = FleetMetrics(fleet)

# Get utilization
utilization = metrics.get_utilization()
print(f"Fleet utilization: {utilization}%")

# Get efficiency
efficiency = metrics.get_task_efficiency()
print(f"Task efficiency: {efficiency}%")

# Get battery status
battery_stats = metrics.get_battery_summary()
print(f"Average battery: {battery_stats['average']}%")
```

### Dashboard Integration

Fleet metrics are available on the dashboard:

```
http://localhost:8000/fleet
```

Displays:
- Robot positions on map
- Task assignments
- Battery levels
- Status indicators
- Real-time telemetry

---

## Best Practices

### DO

✅ Monitor battery levels and return to charge automatically  
✅ Use appropriate robot for task (match capabilities)  
✅ Set task priorities for critical operations  
✅ Implement retry logic for failed commands  
✅ Log all fleet operations for analysis  
✅ Use emergency stop for safety issues  

### DON'T

❌ Assign navigation tasks to stationary sensors  
❌ Overload single robot with all tasks  
❌ Ignore low battery warnings  
❌ Send conflicting commands to same robot  
❌ Deploy without testing coordination algorithms  

---

## Limitations (Current Version)

### What Works

- ✅ Multi-robot discovery and monitoring
- ✅ Task assignment to specific robots
- ✅ Broadcast commands to fleet
- ✅ Robot selection by battery/distance
- ✅ Basic coordination (search, convoy)
- ✅ Real-time fleet dashboard

### Roadmap

- 🔄 **Auction-based task allocation** - Coming in v0.7.0
- 🔄 **Distributed consensus** - Coming in v0.7.0
- 🔄 **Path conflict detection** - Coming in v0.7.0
- 🔄 **Dynamic task rebalancing** - Coming in v0.7.0

---

## Examples

### Warehouse Inventory

```python
# Search warehouse with multiple robots
robots = ["forklift_1", "forklift_2", "drone_1"]
warehouse_zones = ["A", "B", "C", "D"]

for zone in warehouse_zones:
    # Assign zone to available robot
    robot = fleet.select_robot(criteria="battery")
    await fleet.send_command(
        robot_id=robot,
        command="scan_inventory",
        parameters={"zone": zone}
    )
```

### Security Patrol

```python
# Patrol with handoff
patrol_points = [(0, 0), (10, 0), (10, 10), (0, 10)]
robots = ["security_1", "security_2"]

# Alternate robots at each point
for i, point in enumerate(patrol_points):
    robot = robots[i % len(robots)]
    await fleet.send_command(
        robot_id=robot,
        command="patrol_point",
        parameters={"x": point[0], "y": point[1]}
    )
```

### Delivery Coordination

```python
# Coordinate multiple deliveries
deliveries = [
    {"item": "package_A", "destination": "room_101"},
    {"item": "package_B", "destination": "room_205"},
    {"item": "package_C", "destination": "room_302"},
]

for delivery in deliveries:
    # Select best robot based on distance
    robot = fleet.select_robot(
        criteria="distance",
        target_location=delivery["destination"]
    )
    
    await fleet.send_command(
        robot_id=robot,
        command="deliver",
        parameters=delivery
    )
```

---

## API Reference

### FleetOrchestrator

```python
class FleetOrchestrator:
    async def get_fleet_status() -> dict[str, FleetRobot]
    async def select_robot(criteria: str, **kwargs) -> str
    async def send_command(robot_id: str, command: str, parameters: dict) -> CommandResult
    async def broadcast_command(command: str, parameters: dict = None) -> dict[str, CommandResult]
    async def assign_task(robot_id: str, task: Task) -> TaskResult
    async def coordinated_search(robots: list[str], area: dict, pattern: str)
    async def form_convoy(leader: str, followers: list[str], spacing: float)
```

### FleetRobot

```python
@dataclass
class FleetRobot:
    robot_id: str
    status: RobotStatus  # IDLE, BUSY, ERROR, OFFLINE
    battery_level: float
    position: tuple[float, float, float]
    current_task: Task | None
    capabilities: list[RobotCapability]
```

---

## Troubleshooting

### Robot Not Appearing in Fleet

1. Check network connectivity
2. Verify robot is running ROS bridge
3. Check discovery service is enabled
4. Review logs for errors

### Task Assignment Failing

1. Check robot is IDLE (not BUSY or ERROR)
2. Verify robot has required capabilities
3. Check battery level (may be too low)
4. Review command syntax

### Coordination Issues

1. Ensure all robots have synchronized clocks
2. Check for conflicting commands
3. Verify robots can communicate (no network partitions)
4. Review coordination algorithm parameters

---

## Related Documentation

- [API Reference](API_REFERENCE.md) - Complete API documentation
- [Safety Guidelines](SAFETY.md) - Fleet safety considerations
- [Architecture](ARCHITECTURE_V2.md) - System design
- [Examples](../examples/) - Code examples

---

*Last updated: 2026-04-10*  
*Version: v0.6.7*
