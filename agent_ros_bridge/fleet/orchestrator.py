#!/usr/bin/env python3
"""Fleet Orchestrator for Agent ROS Bridge

Manages multi-robot fleets with task allocation, load balancing,
and coordination across heterogeneous robots (ROS1/ROS2 mixed).

Usage:
    from agent_ros_bridge.fleet import FleetOrchestrator, Task, RobotCapability

    orchestrator = FleetOrchestrator()
    await orchestrator.add_robot(robot)
    await orchestrator.submit_task(Task(type="navigate", target="zone_a"))
"""

import asyncio
import logging
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional, Set

logger = logging.getLogger("fleet.orchestrator")


class TaskStatus(Enum):
    """Task lifecycle states"""

    PENDING = auto()
    ASSIGNED = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()


class RobotStatus(Enum):
    """Robot availability states"""

    IDLE = auto()
    BUSY = auto()
    CHARGING = auto()
    OFFLINE = auto()
    ERROR = auto()


@dataclass
class RobotCapability:
    """Robot capabilities for task matching"""

    can_navigate: bool = True
    can_manipulate: bool = False
    can_lift: bool = False
    max_payload_kg: float = 0.0
    max_speed_ms: float = 1.0
    battery_hours: float = 4.0
    ros_version: str = "ros2"  # ros1, ros2
    special_skills: Set[str] = field(default_factory=set)


@dataclass
class Task:
    """Fleet task definition"""

    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    type: str = "navigate"  # navigate, manipulate, transport, charge
    priority: int = 5  # 1-10, lower = higher priority
    target_location: Optional[str] = None
    payload_kg: float = 0.0
    required_capabilities: RobotCapability = field(default_factory=RobotCapability)
    deadline: Optional[datetime] = None
    dependencies: List[str] = field(default_factory=list)  # Task IDs that must complete first
    metadata: Dict[str, Any] = field(default_factory=dict)

    # Runtime fields
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[str] = None
    created_at: datetime = field(default_factory=datetime.utcnow)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    error_message: Optional[str] = None


@dataclass
class FleetRobot:
    """Robot in the fleet"""

    robot_id: str
    name: str
    capabilities: RobotCapability
    status: RobotStatus = RobotStatus.IDLE
    current_task: Optional[str] = None
    current_location: Optional[str] = None
    battery_percent: float = 100.0
    total_tasks_completed: int = 0
    ros_endpoint: str = "localhost"  # For multi-ROS

    # Runtime
    last_seen: datetime = field(default_factory=datetime.utcnow)
    task_history: deque = field(default_factory=lambda: deque(maxlen=100))


@dataclass
class FleetMetrics:
    """Fleet performance metrics"""

    total_robots: int = 0
    active_robots: int = 0
    idle_robots: int = 0
    tasks_pending: int = 0
    tasks_executing: int = 0
    tasks_completed: int = 0
    tasks_failed: int = 0
    avg_task_duration_sec: float = 0.0
    fleet_utilization_percent: float = 0.0


class FleetOrchestrator:
    """Multi-robot fleet orchestration and task allocation"""

    def __init__(self):
        self.robots: Dict[str, FleetRobot] = {}
        self.tasks: Dict[str, Task] = {}
        self.task_queue: List[Task] = []  # Priority queue
        self.running = False
        self._allocation_loop_task: Optional[asyncio.Task] = None

        # Callbacks
        self.on_task_assigned: Optional[Callable[[Task, FleetRobot], None]] = None
        self.on_task_completed: Optional[Callable[[Task, FleetRobot], None]] = None
        self.on_task_failed: Optional[Callable[[Task, str], None]] = None
        self.on_robot_status_changed: Optional[Callable[[FleetRobot], None]] = None

    async def start(self):
        """Start the orchestrator"""
        self.running = True
        self._allocation_loop_task = asyncio.create_task(self._allocation_loop())
        logger.info("🚀 Fleet orchestrator started")

    async def stop(self):
        """Stop the orchestrator"""
        self.running = False
        if self._allocation_loop_task:
            self._allocation_loop_task.cancel()
            try:
                await self._allocation_loop_task
            except asyncio.CancelledError:
                pass
        logger.info("⏹️  Fleet orchestrator stopped")

    async def add_robot(self, robot: FleetRobot) -> bool:
        """Add a robot to the fleet"""
        if robot.robot_id in self.robots:
            logger.warning(f"Robot {robot.robot_id} already in fleet")
            return False

        self.robots[robot.robot_id] = robot
        logger.info(f"🤖 Robot added: {robot.name} ({robot.robot_id})")
        return True

    async def remove_robot(self, robot_id: str) -> bool:
        """Remove a robot from the fleet"""
        if robot_id not in self.robots:
            return False

        robot = self.robots[robot_id]

        # Cancel current task if any
        if robot.current_task:
            await self.cancel_task(robot.current_task)

        del self.robots[robot_id]
        logger.info(f"🗑️  Robot removed: {robot_id}")
        return True

    async def submit_task(self, task: Task) -> str:
        """Submit a new task to the fleet"""
        self.tasks[task.id] = task

        # Insert into priority queue (sorted by priority)
        insert_idx = 0
        for i, t in enumerate(self.task_queue):
            if t.priority > task.priority:
                insert_idx = i
                break
            insert_idx = i + 1

        self.task_queue.insert(insert_idx, task)
        logger.info(f"📋 Task submitted: {task.type} (priority: {task.priority}, id: {task.id})")

        # Trigger immediate allocation attempt
        asyncio.create_task(self._allocate_tasks())

        return task.id

    async def cancel_task(self, task_id: str) -> bool:
        """Cancel a pending or executing task"""
        if task_id not in self.tasks:
            return False

        task = self.tasks[task_id]

        if task.status in [TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED]:
            return False

        # Remove from queue if pending
        if task.status == TaskStatus.PENDING:
            if task in self.task_queue:
                self.task_queue.remove(task)

        # Release robot if assigned
        if task.assigned_robot and task.assigned_robot in self.robots:
            robot = self.robots[task.assigned_robot]
            robot.status = RobotStatus.IDLE
            robot.current_task = None

        task.status = TaskStatus.CANCELLED
        logger.info(f"🚫 Task cancelled: {task_id}")
        return True

    async def update_robot_status(
        self,
        robot_id: str,
        status: RobotStatus,
        location: Optional[str] = None,
        battery: Optional[float] = None,
    ):
        """Update robot status from external monitor"""
        if robot_id not in self.robots:
            return

        robot = self.robots[robot_id]
        old_status = robot.status
        robot.status = status
        robot.last_seen = datetime.utcnow()

        if location:
            robot.current_location = location
        if battery is not None:
            robot.battery_percent = battery

        if old_status != status and self.on_robot_status_changed:
            self.on_robot_status_changed(robot)

        # Handle robot coming back online
        if old_status == RobotStatus.OFFLINE and status != RobotStatus.OFFLINE:
            asyncio.create_task(self._allocate_tasks())

    async def report_task_progress(
        self, task_id: str, progress_percent: float, message: Optional[str] = None
    ):
        """Report task progress from robot"""
        if task_id not in self.tasks:
            return

        task = self.tasks[task_id]
        task.metadata["progress"] = progress_percent
        if message:
            task.metadata["status_message"] = message

        logger.debug(f"Task {task_id} progress: {progress_percent}%")

    async def complete_task(
        self, task_id: str, success: bool = True, result: Optional[Dict] = None
    ):
        """Mark task as completed or failed"""
        if task_id not in self.tasks:
            return

        task = self.tasks[task_id]
        robot = None

        if task.assigned_robot and task.assigned_robot in self.robots:
            robot = self.robots[task.assigned_robot]
            robot.status = RobotStatus.IDLE
            robot.current_task = None
            robot.total_tasks_completed += 1
            robot.task_history.append(
                {
                    "task_id": task_id,
                    "type": task.type,
                    "success": success,
                    "timestamp": datetime.utcnow().isoformat(),
                }
            )

        if success:
            task.status = TaskStatus.COMPLETED
            task.completed_at = datetime.utcnow()
            logger.info(f"✅ Task completed: {task_id}")
            if self.on_task_completed and robot:
                self.on_task_completed(task, robot)
        else:
            task.status = TaskStatus.FAILED
            task.error_message = result.get("error") if result else "Unknown error"
            logger.error(f"❌ Task failed: {task_id} - {task.error_message}")
            if self.on_task_failed:
                self.on_task_failed(task, task.error_message)

        # Trigger reallocation
        asyncio.create_task(self._allocate_tasks())

    def get_metrics(self) -> FleetMetrics:
        """Get current fleet metrics"""
        metrics = FleetMetrics()
        metrics.total_robots = len(self.robots)
        metrics.active_robots = sum(1 for r in self.robots.values() if r.status == RobotStatus.BUSY)
        metrics.idle_robots = sum(1 for r in self.robots.values() if r.status == RobotStatus.IDLE)
        metrics.tasks_pending = len(
            [t for t in self.tasks.values() if t.status == TaskStatus.PENDING]
        )
        metrics.tasks_executing = len(
            [t for t in self.tasks.values() if t.status == TaskStatus.EXECUTING]
        )
        metrics.tasks_completed = len(
            [t for t in self.tasks.values() if t.status == TaskStatus.COMPLETED]
        )
        metrics.tasks_failed = len(
            [t for t in self.tasks.values() if t.status == TaskStatus.FAILED]
        )

        if metrics.total_robots > 0:
            metrics.fleet_utilization_percent = (metrics.active_robots / metrics.total_robots) * 100

        return metrics

    def get_fleet_status(self) -> Dict[str, Any]:
        """Get complete fleet status for dashboard"""
        return {
            "robots": [
                {
                    "id": r.robot_id,
                    "name": r.name,
                    "status": r.status.name,
                    "location": r.current_location,
                    "battery": r.battery_percent,
                    "current_task": r.current_task,
                    "capabilities": {
                        "navigate": r.capabilities.can_navigate,
                        "manipulate": r.capabilities.can_manipulate,
                        "lift": r.capabilities.can_lift,
                        "max_payload": r.capabilities.max_payload_kg,
                    },
                }
                for r in self.robots.values()
            ],
            "tasks": [
                {
                    "id": t.id,
                    "type": t.type,
                    "status": t.status.name,
                    "priority": t.priority,
                    "assigned_robot": t.assigned_robot,
                    "progress": t.metadata.get("progress", 0),
                }
                for t in self.tasks.values()
                if t.status not in [TaskStatus.COMPLETED, TaskStatus.CANCELLED]
            ],
            "metrics": self.get_metrics(),
        }

    async def _allocation_loop(self):
        """Background task allocation loop"""
        while self.running:
            try:
                await self._allocate_tasks()
                await asyncio.sleep(1.0)  # Check every second
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in allocation loop: {e}")
                await asyncio.sleep(5.0)

    async def _allocate_tasks(self):
        """Allocate pending tasks to available robots"""
        if not self.task_queue:
            return

        for task in list(self.task_queue):
            if task.status != TaskStatus.PENDING:
                continue

            # Check dependencies
            deps_satisfied = True
            for dep_id in task.dependencies:
                if dep_id in self.tasks:
                    dep_task = self.tasks[dep_id]
                    if dep_task.status != TaskStatus.COMPLETED:
                        deps_satisfied = False
                        break

            if not deps_satisfied:
                continue

            # Find best robot for task
            robot = self._select_robot_for_task(task)

            if robot:
                await self._assign_task_to_robot(task, robot)
                self.task_queue.remove(task)

    def _select_robot_for_task(self, task: Task) -> Optional[FleetRobot]:
        """Select best robot for a task based on capabilities and availability"""
        candidates = []

        for robot in self.robots.values():
            # Must be idle
            if robot.status != RobotStatus.IDLE:
                continue

            # Must have sufficient battery
            if robot.battery_percent < 20:
                continue

            # Check capability match
            if not self._can_robot_handle_task(robot, task):
                continue

            # Calculate score (higher = better)
            score = self._calculate_robot_score(robot, task)
            candidates.append((robot, score))

        if not candidates:
            return None

        # Return highest scoring robot
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates[0][0]

    def _can_robot_handle_task(self, robot: FleetRobot, task: Task) -> bool:
        """Check if robot has required capabilities"""
        caps = robot.capabilities
        required = task.required_capabilities

        if required.can_navigate and not caps.can_navigate:
            return False
        if required.can_manipulate and not caps.can_manipulate:
            return False
        if required.can_lift and not caps.can_lift:
            return False
        if task.payload_kg > caps.max_payload_kg:
            return False

        return True

    def _calculate_robot_score(self, robot: FleetRobot, task: Task) -> float:
        """Calculate suitability score for robot-task pairing"""
        score = 100.0

        # Prefer robots with higher battery
        score += robot.battery_percent * 0.5

        # Prefer robots with higher success rate (if history available)
        if robot.task_history:
            recent = list(robot.task_history)[-10:]
            success_rate = sum(1 for t in recent if t["success"]) / len(recent)
            score += success_rate * 50

        # Prefer closer robots (if location data available)
        if robot.current_location and task.target_location:
            # Simple string match, could be replaced with distance calculation
            if robot.current_location == task.target_location:
                score += 25

        # Penalize robots with mismatched ROS version (for efficiency)
        if robot.capabilities.ros_version != task.required_capabilities.ros_version:
            score -= 10

        return score

    async def _assign_task_to_robot(self, task: Task, robot: FleetRobot):
        """Assign a task to a robot"""
        task.status = TaskStatus.ASSIGNED
        task.assigned_robot = robot.robot_id
        task.started_at = datetime.utcnow()

        robot.status = RobotStatus.BUSY
        robot.current_task = task.id

        logger.info(f"🎯 Task {task.id} assigned to {robot.name}")

        if self.on_task_assigned:
            self.on_task_assigned(task, robot)

        # Execute task (in real implementation, this would send command to robot)
        asyncio.create_task(self._execute_task(task, robot))

    async def _execute_task(self, task: Task, robot: FleetRobot):
        """Execute task on robot (simulated)"""
        task.status = TaskStatus.EXECUTING

        logger.info(f"▶️  Executing task {task.id} on {robot.name}")

        # In real implementation, send command to robot via bridge
        # For now, simulate task execution
        try:
            # Simulate task duration based on type
            duration = {"navigate": 5.0, "manipulate": 8.0, "transport": 10.0, "charge": 30.0}.get(
                task.type, 5.0
            )

            # Simulate progress updates
            for i in range(10):
                await asyncio.sleep(duration / 10)
                await self.report_task_progress(task.id, (i + 1) * 10)

            # Complete successfully
            await self.complete_task(
                task.id, success=True, result={"location": task.target_location}
            )

        except Exception as e:
            await self.complete_task(task.id, success=False, result={"error": str(e)})
