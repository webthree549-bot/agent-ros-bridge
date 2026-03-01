#!/usr/bin/env python3
"""Fleet Orchestrator Demo.

Demonstrates multi-robot task allocation with a simulated warehouse fleet.

Usage:
    python demo/fleet_demo.py
"""

import asyncio
import logging

from agent_ros_bridge import Bridge
from agent_ros_bridge.fleet.orchestrator import (
    FleetOrchestrator,
    FleetRobot,
    RobotCapability,
    Task,
)
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("fleet_demo")


async def handle_fleet_command(orchestrator, message, identity):
    """Handle fleet-related commands from WebSocket."""
    if not message.command:
        return None

    cmd = message.command
    action = cmd.action
    params = cmd.parameters or {}

    if action == "fleet.status":
        status = orchestrator.get_fleet_status()
        return {"type": "fleet_status", "data": status}

    elif action == "fleet.submit_task":
        task = Task(
            type=params.get("type", "navigate"),
            priority=params.get("priority", 5),
            target_location=params.get("target"),
            payload_kg=params.get("payload", 0.0),
        )
        task_id = await orchestrator.submit_task(task)
        return {"type": "task_submitted", "task_id": task_id}

    elif action == "fleet.cancel_task":
        success = await orchestrator.cancel_task(params.get("task_id"))
        return {"type": "task_cancelled", "success": success}

    elif action == "fleet.metrics":
        metrics = orchestrator.get_metrics()
        return {
            "type": "fleet_metrics",
            "data": {
                "total_robots": metrics.total_robots,
                "active": metrics.active_robots,
                "idle": metrics.idle_robots,
                "pending_tasks": metrics.tasks_pending,
                "executing": metrics.tasks_executing,
                "completed": metrics.tasks_completed,
                "utilization": round(metrics.fleet_utilization_percent, 1),
            },
        }

    return None


async def main():
    # Create bridge
    bridge = Bridge()

    # Add WebSocket transport
    ws_transport = WebSocketTransport({"port": 8771})
    bridge.transport_manager.register(ws_transport)

    # Create fleet orchestrator
    orchestrator = FleetOrchestrator()

    # Set up callbacks
    def on_task_assigned(task, robot):
        logger.info(f"✅ Task {task.id[:8]} assigned to {robot.name}")

    def on_task_completed(task, robot):
        logger.info(f"🎉 Task {task.id[:8]} completed by {robot.name}")

    def on_task_failed(task, error):
        logger.error(f"❌ Task {task.id[:8]} failed: {error}")

    orchestrator.on_task_assigned = on_task_assigned
    orchestrator.on_task_completed = on_task_completed
    orchestrator.on_task_failed = on_task_failed

    # Start orchestrator
    await orchestrator.start()

    # Add robots to fleet (mixed ROS1/ROS2)
    robots = [
        FleetRobot(
            robot_id="tb4_001",
            name="TurtleBot4-Alpha",
            capabilities=RobotCapability(
                can_navigate=True,
                can_manipulate=False,
                max_payload_kg=5.0,
                max_speed_ms=0.5,
                ros_version="ros2",
            ),
            current_location="charging_station",
        ),
        FleetRobot(
            robot_id="tb4_002",
            name="TurtleBot4-Beta",
            capabilities=RobotCapability(
                can_navigate=True,
                can_manipulate=False,
                max_payload_kg=5.0,
                max_speed_ms=0.5,
                ros_version="ros2",
            ),
            current_location="warehouse_a",
        ),
        FleetRobot(
            robot_id="ur5_001",
            name="UR5-Arm-01",
            capabilities=RobotCapability(
                can_navigate=False,
                can_manipulate=True,
                can_lift=True,
                max_payload_kg=10.0,
                ros_version="ros1",
                special_skills={"pick_and_place", "assembly"},
            ),
            current_location="assembly_station",
        ),
        FleetRobot(
            robot_id="amr_001",
            name="Heavy-AMR-01",
            capabilities=RobotCapability(
                can_navigate=True,
                can_lift=True,
                max_payload_kg=50.0,
                max_speed_ms=0.3,
                ros_version="ros2",
            ),
            current_location="loading_dock",
        ),
    ]

    for robot in robots:
        await orchestrator.add_robot(robot)

    # Start bridge
    await bridge.start()

    # Auto-submit some demo tasks
    demo_tasks = [
        Task(type="navigate", priority=3, target_location="zone_a"),
        Task(type="transport", priority=2, target_location="zone_b", payload_kg=3.0),
        Task(type="navigate", priority=5, target_location="zone_c"),
        Task(
            type="transport", priority=1, target_location="zone_d", payload_kg=45.0
        ),  # Needs heavy lifter
    ]

    print("=" * 70)
    print("🚛 FLEET ORCHESTRATOR DEMO")
    print("=" * 70)
    print("")
    print("Fleet Composition:")
    for robot in robots:
        print(f"  • {robot.name} ({robot.robot_id})")
        print(
            f"    ROS: {robot.capabilities.ros_version} | Navigate: {robot.capabilities.can_navigate} | Lift: {robot.capabilities.can_lift}"
        )
    print("")
    print("WebSocket: ws://localhost:8771")
    print("")
    print("Submitting demo tasks...")

    for task in demo_tasks:
        await orchestrator.submit_task(task)
        await asyncio.sleep(1)

    print("")
    print("Available commands:")
    print('  {"command": {"action": "fleet.status"}}')
    print('  {"command": {"action": "fleet.metrics"}}')
    print(
        '  {"command": {"action": "fleet.submit_task", "parameters": {"type": "navigate", "target": "zone_x"}}}'
    )
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 70)

    # Status printer
    async def print_status():
        while True:
            await asyncio.sleep(10)
            metrics = orchestrator.get_metrics()
            print(
                f"\n📊 Fleet: {metrics.active_robots}/{metrics.total_robots} robots active, "
                f"{metrics.tasks_pending} pending, {metrics.tasks_executing} executing"
            )

    status_task = asyncio.create_task(print_status())

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️  Stopping...")
        status_task.cancel()
        await orchestrator.stop()
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
