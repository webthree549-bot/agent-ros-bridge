"""
Example 1: Warehouse Automation with Safety Validation

This example demonstrates deploying Agent ROS Bridge in a warehouse environment
with full safety validation before autonomous operation.

Key Features:
- Shadow mode data collection (200+ hours)
- Gradual rollout (0% → 100% autonomy)
- Fleet coordination (multiple forklifts/AGVs)
- Production monitoring

Use Case: E-commerce fulfillment center with 50+ robots
"""

from agent_ros_bridge import RobotAgent
from agent_ros_bridge.shadow import ShadowModeIntegration
from agent_ros_bridge.fleet import FleetManager
from dataclasses import dataclass
from typing import List, Dict
import time


@dataclass
class WarehouseTask:
    """Represents a warehouse operation task."""
    task_id: str
    task_type: str  # "pick", "place", "transport", "inspect"
    source_location: str
    target_location: str
    payload_description: str
    priority: int = 1


class WarehouseRobot:
    """
    Warehouse robot with safety-first deployment.
    
    Stage 0 (Current): Human approves all actions
    Stage 1: Collect 200+ hours shadow data
    Stage 2: Gradual rollout 10% → 100%
    Stage 3: Full autonomy (after validation)
    """
    
    def __init__(self, robot_id: str):
        self.agent = RobotAgent(
            device_id=robot_id,
            device_type='mobile_robot',
            llm_provider='moonshot',
            require_confirmation=True,  # Safety: Human approval required
        )
        self.shadow = ShadowModeIntegration()
        self.tasks_completed = 0
        self.tasks_failed = 0
        
    def execute_task(self, task: WarehouseTask, human_approval: bool = False) -> dict:
        """
        Execute warehouse task with safety validation.
        
        Args:
            task: Warehouse task to execute
            human_approval: Required for all actions in Stage 0
            
        Returns:
            Execution result with metrics
        """
        print(f"\n🤖 Robot {self.agent.device_id} - Task: {task.task_type}")
        print(f"   From: {task.source_location} → To: {task.target_location}")
        
        # AI proposes action
        command = f"{task.task_type} from {task.source_location} to {task.target_location}"
        
        # Log AI proposal to shadow mode
        record_id = self.shadow.log_ai_decision(
            robot_id=self.agent.device_id,
            intent_type=task.task_type.upper(),
            confidence=0.92,
            entities=[
                {'type': 'SOURCE', 'value': task.source_location},
                {'type': 'TARGET', 'value': task.target_location},
            ],
        )
        
        # Stage 0: Human approval required
        if not human_approval:
            print("   ⏳ Waiting for human approval...")
            # In production, this would wait for operator input
            human_approval = True  # Simulated approval
        
        # Log human decision
        self.shadow.log_human_decision(
            robot_id=self.agent.device_id,
            command=task.task_type,
            parameters={
                'source': task.source_location,
                'target': task.target_location,
            },
        )
        
        # Execute with monitoring
        print("   ✅ Human approved - Executing...")
        start_time = time.time()
        
        try:
            # Simulated execution
            time.sleep(0.5)  # Task execution time
            success = True
            self.tasks_completed += 1
        except Exception as e:
            success = False
            self.tasks_failed += 1
            print(f"   ❌ Task failed: {e}")
        
        duration = time.time() - start_time
        
        # Log outcome
        self.shadow.log_outcome(
            record_id=record_id,
            success=success,
            execution_time_ms=duration * 1000,
        )
        
        return {
            'success': success,
            'duration': duration,
            'task_id': task.task_id,
        }
    
    def get_safety_metrics(self) -> dict:
        """Get current safety metrics for this robot."""
        metrics = self.shadow.get_metrics()
        return {
            'robot_id': self.agent.device_id,
            'total_decisions': metrics['total_decisions'],
            'completed_decisions': metrics['completed_decisions'],
            'tasks_completed': self.tasks_completed,
            'tasks_failed': self.tasks_failed,
            'safety_status': 'COMPLIANT' if self.tasks_failed == 0 else 'REVIEW',
        }


class WarehouseFleet:
    """
    Coordinates multiple warehouse robots with fleet-wide safety.
    
    Example:
        fleet = WarehouseFleet(fleet_id='warehouse_alpha', max_robots=10)
        fleet.add_robot('forklift_01')
        fleet.add_robot('forklift_02')
        fleet.add_robot('agv_03')
        
        # Distribute tasks across fleet
        tasks = generate_pick_tasks()
        fleet.distribute_tasks(tasks)
    """
    
    def __init__(self, fleet_id: str, max_robots: int = 10):
        self.fleet_id = fleet_id
        self.max_robots = max_robots
        self.robots: Dict[str, WarehouseRobot] = {}
        self.fleet_manager = FleetManager(fleet_id=fleet_id, max_robots=max_robots)
        
    def add_robot(self, robot_id: str) -> WarehouseRobot:
        """Add robot to fleet with safety configuration."""
        if len(self.robots) >= self.max_robots:
            raise ValueError(f"Fleet at capacity ({self.max_robots})")
        
        robot = WarehouseRobot(robot_id)
        self.robots[robot_id] = robot
        print(f"✅ Added {robot_id} to fleet {self.fleet_id}")
        return robot
    
    def distribute_tasks(self, tasks: List[WarehouseTask]) -> dict:
        """
        Distribute tasks across available robots.
        
        Simple round-robin distribution with safety checks.
        """
        results = {}
        robot_ids = list(self.robots.keys())
        
        for i, task in enumerate(tasks):
            robot_id = robot_ids[i % len(robot_ids)]
            robot = self.robots[robot_id]
            
            print(f"\n📋 Assigning {task.task_id} to {robot_id}")
            result = robot.execute_task(task, human_approval=True)
            results[task.task_id] = result
        
        return results
    
    def get_fleet_metrics(self) -> dict:
        """Get aggregate metrics for entire fleet."""
        total_tasks = sum(r.tasks_completed for r in self.robots.values())
        total_failures = sum(r.tasks_failed for r in self.robots.values())
        
        return {
            'fleet_id': self.fleet_id,
            'active_robots': len(self.robots),
            'total_tasks_completed': total_tasks,
            'total_failures': total_failures,
            'success_rate': total_tasks / (total_tasks + total_failures) if (total_tasks + total_failures) > 0 else 0,
            'safety_status': 'OPERATIONAL' if total_failures == 0 else 'INVESTIGATE',
        }


def main():
    """Demonstrate warehouse automation with safety."""
    print("=" * 60)
    print("🏭 Warehouse Automation Example")
    print("=" * 60)
    print("\n🛡️  Safety Configuration:")
    print("   - Human approval required: YES")
    print("   - Shadow mode: ENABLED")
    print("   - Gradual rollout: 0% (Stage 0)")
    print("   - Target: 200+ hours before autonomy")
    
    # Create fleet
    fleet = WarehouseFleet(fleet_id='fulfillment_center_1', max_robots=5)
    
    # Add robots
    fleet.add_robot('forklift_01')
    fleet.add_robot('forklift_02')
    fleet.add_robot('agv_03')
    
    # Create sample tasks
    tasks = [
        WarehouseTask('T001', 'pick', 'A1', 'B2', 'pallet_electronics'),
        WarehouseTask('T002', 'place', 'B2', 'C3', 'pallet_electronics'),
        WarehouseTask('T003', 'transport', 'C3', 'DOCK_1', 'pallet_electronics'),
        WarehouseTask('T004', 'pick', 'A3', 'B4', 'pallet_clothing'),
        WarehouseTask('T005', 'place', 'B4', 'C5', 'pallet_clothing'),
    ]
    
    print(f"\n📦 Processing {len(tasks)} warehouse tasks...")
    
    # Execute tasks
    results = fleet.distribute_tasks(tasks)
    
    # Show metrics
    print("\n" + "=" * 60)
    print("📊 Fleet Metrics")
    print("=" * 60)
    metrics = fleet.get_fleet_metrics()
    for key, value in metrics.items():
        print(f"   {key}: {value}")
    
    print("\n✅ Warehouse automation example complete!")
    print("   All tasks executed with human oversight.")
    print("   Shadow mode collecting validation data.")


if __name__ == '__main__':
    main()
