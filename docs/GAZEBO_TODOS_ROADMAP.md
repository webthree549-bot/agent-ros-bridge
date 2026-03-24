# Gazebo Integration TODOs - Implementation Roadmap

This document tracks the TODO items in `gazebo_batch.py` and provides implementation guidance.

## Current Status: Structure Complete, Implementation Pending

The Gazebo simulator infrastructure is in place with:
- ✅ Parallel world management (4-8 worlds)
- ✅ Docker environment detection
- ✅ Foxglove WebSocket structure
- ✅ Scenario loading framework
- ✅ Batch execution orchestration
- 🚧 **Pending:** Actual Gazebo transport integration

---

## TODO Items

### 1. Gazebo Transport Integration (Lines 307, 313)
**TODO:** Implement via Gazebo transport

**What:** Connect to Gazebo's internal transport system for:
- Scene queries (object positions, states)
- Physics control (pause, step, reset)
- Entity spawning and removal

**Implementation:**
```python
# Using pygz (Gazebo Python bindings) or direct protobuf
from gz.transport import Node

node = Node()
# Subscribe to scene info topic
node.subscribe('/gazebo/default/scene/info', scene_callback)
# Publish entity spawn request
node.publish('/gazebo/default/entity/system', spawn_msg)
```

**Priority:** HIGH - Required for simulation state queries

---

### 2. Physics Simulation State (Line 318)
**TODO:** Check physics simulation state

**What:** Verify physics is running, check for:
- Simulation time advancement
- Physics engine status (ODE, Bullet, DART)
- Real-time factor

**Implementation:**
```python
def check_physics_state(self) -> dict:
    """Query physics simulation status."""
    return {
        "running": self._get_sim_time() > self._last_sim_time,
        "real_time_factor": self._get_rtf(),
        "paused": self._is_paused(),
        "physics_engine": self._get_physics_engine(),
    }
```

**Priority:** MEDIUM - Needed for reliable scenario validation

---

### 3. ROS2/Gazebo Spawn (Line 324)
**TODO:** Implement via ROS2/Gazebo spawn

**What:** Spawn robots and obstacles using:
- ROS2 `create_entity` service
- Gazebo spawn_entity service
- Direct SDF insertion

**Implementation:**
```python
from gazebo_msgs.srv import SpawnEntity

spawn_client = self._node.create_client(SpawnEntity, '/spawn_entity')
request = SpawnEntity.Request()
request.xml = robot_sdf
request.robot_namespace = f'robot_{world_id}'
request.initial_pose = pose
spawn_client.call_async(request)
```

**Priority:** HIGH - Required for multi-robot scenarios

---

### 4. Navigation Execution (Line 336)
**TODO:** Implement actual navigation

**What:** Execute Nav2 navigation to target poses

**Implementation:**
```python
from nav2_simple_commander import RobotNavigator

navigator = RobotNavigator(namespace=f'robot_{world_id}')
navigator.goToPose(goal_pose)
# Wait for completion with timeout
while not navigator.isTaskComplete():
    time.sleep(0.1)
```

**Priority:** HIGH - Core functionality for navigation scenarios

---

### 5. Pose Sampling (Line 347)
**TODO:** Sample poses from simulation

**What:** Get valid spawn/navigation poses from:
- Occupancy grid (Nav2 costmap)
- Gazebo world bounds
- Predefined locations

**Implementation:**
```python
def sample_valid_pose(self, world_id: int) -> Pose:
    """Sample collision-free pose from simulation."""
    # Query costmap for free space
    costmap = self._get_costmap(world_id)
    free_cells = np.where(costmap == 0)
    # Random selection from free space
    idx = random.randint(0, len(free_cells[0]) - 1)
    return self._cell_to_pose(free_cells[0][idx], free_cells[1][idx])
```

**Priority:** MEDIUM - Needed for randomized scenarios

---

### 6. State Queries (Line 353)
**TODO:** Query from Gazebo/ROS

**What:** Get current simulation state:
- Robot pose (AMCL, ground truth)
- Object positions
- Sensor readings

**Implementation:**
```python
# Ground truth from Gazebo
self._node.create_subscription(
    ModelStates, '/gazebo/model_states', self._model_states_cb
)

# AMCL estimate from robot
self._node.create_subscription(
    PoseWithCovarianceStamped,
    f'/robot_{world_id}/amcl_pose',
    self._amcl_pose_cb
)
```

**Priority:** HIGH - Required for metrics collection

---

### 7. Collision Detection (Line 371)
**TODO:** Query collision state

**What:** Detect collisions between:
- Robot and obstacles
- Robot and environment
- Objects with each other

**Implementation:**
```python
def get_collision_state(self, world_id: int) -> dict:
    """Query current collisions."""
    # Option 1: Gazebo contacts sensor
    contacts = self._query_contacts(world_id)
    # Option 2: ROS2 bumper/scan topics
    return {
        "collision_count": len(contacts),
        "contacts": contacts,
        "is_collision": len(contacts) > 0,
    }
```

**Priority:** HIGH - Required for safety metrics

---

### 8. Nav2 Status (Line 376)
**TODO:** Get from Nav2

**What:** Query Nav2 navigation status:
- Current path
- Goal reached
- Navigation failure reason

**Implementation:**
```python
from nav2_msgs.action import NavigateToPose

# Monitor action result
result = navigator.getResult()
if result == NavigateToPose.Result().SUCCEEDED:
    status = "success"
elif result == NavigateToPose.Result().FAILED:
    status = "failed"
```

**Priority:** HIGH - Required for navigation validation

---

### 9. Foxglove MCAP Protocol (Line 516)
**TODO:** Implement MCAP/WebSocket protocol

**What:** Stream simulation data to Foxglove Studio:
- Robot trajectory
- Sensor data
- Decision points

**Implementation:**
```python
from foxglove_websocket import FoxgloveServer

server = FoxgloveServer(
    name="Gazebo Batch Runner",
    port=8765
)
# Publish trajectory as PoseArray
server.add_channel({
    "topic": "/robot/trajectory",
    "schema": "geometry_msgs/PoseArray"
})
```

**Priority:** LOW - Visualization enhancement, not core functionality

---

### 10. WebSocket Server Startup (Line 521)
**TODO:** Implement actual WebSocket server startup

**What:** Start Foxglove WebSocket server for each world

**Implementation:**
```python
async def start_foxglove_server(self, world_id: int, port: int):
    """Start Foxglove WebSocket server for world."""
    server = FoxgloveServer(
        name=f"World {world_id}",
        port=port
    )
    self._foxglove_servers[world_id] = server
    await server.start()
```

**Priority:** LOW - Visualization enhancement

---

## Implementation Priority Matrix

| Priority | TODOs | Effort | Impact |
|----------|-------|--------|--------|
| **P0 - Critical** | 3, 4, 7, 8 | 2-3 days | Cannot run scenarios without these |
| **P1 - High** | 1, 6 | 1-2 days | Required for full metrics |
| **P2 - Medium** | 2, 5 | 1 day | Nice to have for robustness |
| **P3 - Low** | 9, 10 | 2-3 days | Visualization only |

---

## Blocked By

These TODOs require Docker environment with:
- [ ] Gazebo Garden/Harmonic running
- [ ] Nav2 stack operational
- [ ] ROS2 Humble/Jazzy available
- [ ] PyGZ or ROS2 bindings installed

Current workaround: Structure is in place, implementations stubbed until Docker environment is fully operational.

---

## Next Steps

1. **Verify Docker environment** — Ensure Gazebo + Nav2 container is running
2. **Implement P0 TODOs** — Core navigation and collision detection
3. **Run integration tests** — Validate with 100 scenarios
4. **Iterate on P1/P2** — Full metrics and robustness
5. **Add visualization** — Foxglove integration for debugging

---

*Document Version: 1.0*  
*Last Updated: 2026-03-24*  
*Status: Planning Phase*
