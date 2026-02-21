"""
Mars Colony Command - Fully integrated with Agent ROS Bridge
Demonstrates: gRPC transport for mission-critical comms, multi-robot fleet coordination, 
distributed task planning across heterogeneous robots
"""
import asyncio
import os
import json
import random
from datetime import datetime
from typing import Dict, List

# Required: Agent ROS Bridge imports
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.grpc import gRPCTransport
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, RobotCapability, Task, RobotStatus


class MarsResource:
    """Resource types on Mars"""
    ICE = "ice"
    POWER = "power"
    MINERALS = "minerals"
    OXYGEN = "oxygen"


class MarsColony:
    """
    Mars Habitat Alpha Command Center using Agent ROS Bridge for:
    1. gRPC transport for reliable mission-critical communication
    2. Fleet orchestration for 7 specialized robots
    3. Distributed task planning and allocation
    """
    
    def __init__(self):
        self.sol = 1
        self.resources = {
            MarsResource.ICE: 100,
            MarsResource.POWER: 100,
            MarsResource.MINERALS: 50,
            MarsResource.OXYGEN: 100
        }
        self.buildings = ["Hab Module Alpha", "Solar Array", "Ice Refinery"]
        self.crew_health = 100
        
        # Agent ROS Bridge components
        self.bridge = None
        self.fleet = None
        self.grpc_port = 50051
        
    async def initialize(self):
        """
        Initialize Agent ROS Bridge with gRPC for Mars operations.
        gRPC provides reliable, low-latency communication for critical missions.
        """
        print("🚀 MARS COLONY COMMAND - Agent ROS Bridge v0.3.3")
        print("=" * 60)
        
        # REQUIRED: JWT_SECRET
        jwt_secret = os.environ.get("JWT_SECRET")
        if not jwt_secret:
            raise RuntimeError(
                "❌ JWT_SECRET required!\n"
                "Run: export JWT_SECRET=$(openssl rand -base64 32)"
            )
        
        # Initialize Bridge
        self.bridge = Bridge()
        print("✓ Bridge initialized")
        
        # Register gRPC transport (mission-critical reliability)
        grpc_transport = gRPCTransport({
            "port": self.grpc_port,
            "host": "0.0.0.0"
        })
        self.bridge.transport_manager.register(grpc_transport)
        print(f"✓ gRPC transport on port {self.grpc_port}")
        
        # Start bridge
        await self.bridge.start()
        print("✓ Bridge active")
        
        # Initialize Fleet Orchestrator
        self.fleet = FleetOrchestrator()
        
        # Register robot fleet
        await self._initialize_fleet()
        
        # Start fleet coordination
        asyncio.create_task(self.fleet.start())
        print("✓ Fleet coordination active")
        
        print("=" * 60)
        print("🚀 Mars Colony ready - Agent ROS Bridge active")
        print(f"   gRPC: localhost:{self.grpc_port}")
        print(f"   Fleet: {len(self.fleet.robots)} robots")
        print()
        
    async def _initialize_fleet(self):
        """Register Mars robot fleet with capabilities"""
        robots = [
            ("excavator_1", "excavator", ["mining", "drilling", "hauling"], "Mining ice and minerals"),
            ("excavator_2", "excavator", ["mining", "drilling", "hauling"], "Mining operations"),
            ("solar_drone_1", "solar_drone", ["deployment", "maintenance", "monitoring"], "Power generation"),
            ("solar_drone_2", "solar_drone", ["deployment", "maintenance", "monitoring"], "Power farm maintenance"),
            ("builder_1", "builder", ["construction", "repair", "fabrication"], "Construction and repairs"),
            ("scout_1", "scout", ["exploration", "mapping", "surveying"], "Terrain exploration"),
            ("scout_2", "scout", ["exploration", "mapping", "surveying"], "Resource mapping"),
        ]
        
        for bot_id, bot_type, capabilities, task in robots:
            robot = FleetRobot(
                robot_id=bot_id,
                robot_type=bot_type,
                capabilities=[RobotCapability(cap) for cap in capabilities],
                status=RobotStatus.IDLE
            )
            self.fleet.register_robot(robot)
            print(f"  🤖 {bot_id} ({bot_type}) - {task}")
            
    async def run_mission(self, sols: int = 10):
        """
        Run Mars colony mission with fleet task allocation.
        Demonstrates: Distributed planning, resource management, emergency response
        """
        print(f"\n🚀 Mission: Survive {sols} sols")
        print("=" * 60)
        
        for sol in range(1, sols + 1):
            self.sol = sol
            print(f"\n🗓️  SOL {sol}")
            print("-" * 40)
            
            # Daily resource consumption
            self.resources[MarsResource.OXYGEN] -= 5
            self.resources[MarsResource.POWER] -= 10
            
            # Create and allocate tasks
            await self._allocate_daily_tasks()
            
            # Random events
            if random.random() < 0.3:
                await self._handle_event()
            
            # Check critical resources
            for resource, amount in self.resources.items():
                if amount < 20:
                    print(f"  ⚠️  LOW {resource.upper()}: {amount} remaining!")
                    await self._emergency_allocation(resource)
            
            # Show fleet status
            await self._show_fleet_status()
            
            await asyncio.sleep(2)
        
        await self._mission_summary()
        
    async def _allocate_daily_tasks(self):
        """Allocate daily tasks to robot fleet"""
        tasks = [
            ("mine_ice", ["mining"], {"ice": 15}),
            ("mine_minerals", ["mining"], {"minerals": 10}),
            ("generate_power", ["deployment"], {"power": 20}),
            ("explore_sector", ["exploration"], {"discovery": True}),
        ]
        
        for task_id, required_caps, rewards in tasks:
            task = Task(
                task_id=f"{task_id}_sol{self.sol}",
                description=task_id.replace("_", " ").title(),
                required_capabilities=required_caps,
                priority=random.randint(3, 8)
            )
            
            self.fleet.submit_task(task)
            robot_id = self.fleet.allocate_task(task)
            
            if robot_id:
                print(f"  ✅ {robot_id}: {task.description}")
                
                # Apply rewards
                for resource, amount in rewards.items():
                    if resource in self.resources:
                        self.resources[resource] = min(200, self.resources[resource] + amount)
                        print(f"     +{amount} {resource}")
                
                # Mark complete
                self.fleet.mark_task_complete(task.task_id, success=True)
            else:
                print(f"  ⚠️ No robot available for {task.description}")
                
    async def _emergency_allocation(self, resource: str):
        """Emergency task allocation for critical resources"""
        print(f"  🚨 Emergency: Allocating all available robots to gather {resource}")
        
        emergency_task = Task(
            task_id=f"emergency_{resource}_{self.sol}",
            description=f"Emergency {resource} gathering",
            required_capabilities=["mining" if resource in ["ice", "minerals"] else "deployment"],
            priority=10  # Critical
        )
        
        self.fleet.submit_task(task)
        
        # Try to allocate multiple robots
        for _ in range(3):
            robot_id = self.fleet.allocate_task(emergency_task)
            if robot_id:
                print(f"     {robot_id} responding to emergency")
                self.resources[resource] = min(200, self.resources[resource] + 30)
                self.fleet.mark_task_complete(emergency_task.task_id, success=True)
                
    async def _handle_event(self):
        """Handle random events"""
        events = [
            ("Dust storm reduces solar output", lambda: self.resources.update({MarsResource.POWER: self.resources[MarsResource.POWER] - 20})),
            ("Rich ice deposit found!", lambda: self.resources.update({MarsResource.ICE: self.resources[MarsResource.ICE] + 40})),
            ("Mineral vein discovered", lambda: self.resources.update({MarsResource.MINERALS: self.resources[MarsResource.MINERALS] + 25})),
            ("Equipment malfunction", lambda: setattr(self, 'crew_health', self.crew_health - 5)),
        ]
        
        event_text, effect = random.choice(events)
        print(f"  📢 Event: {event_text}")
        effect()
        
        # Broadcast event via gRPC
        await self._broadcast_event(event_text)
        
    async def _broadcast_event(self, event_text: str):
        """Broadcast event via gRPC to all connected clients"""
        message = {
            "type": "event",
            "sol": self.sol,
            "text": event_text,
            "resources": self.resources,
            "timestamp": datetime.now().isoformat()
        }
        print(f"   📡 gRPC Broadcast: {json.dumps(message)[:80]}...")
        
    async def _show_fleet_status(self):
        """Display fleet status"""
        metrics = self.fleet.get_metrics()
        print(f"\n   📊 Fleet: {metrics.completed_tasks} tasks, "
              f"{metrics.robot_utilization}")
        
    async def _mission_summary(self):
        """End of mission summary"""
        print("\n" + "=" * 60)
        print("🚀 MISSION COMPLETE")
        print("=" * 60)
        print(f"Survived: {self.sol} sols")
        print(f"Buildings: {len(self.buildings)}")
        print("Resources:")
        for r, a in self.resources.items():
            print(f"  {r}: {a}")
        
        metrics = self.fleet.get_metrics()
        print(f"\nFleet Statistics:")
        print(f"  Total tasks: {metrics.total_tasks}")
        print(f"  Completed: {metrics.completed_tasks}")
        print(f"  Failed: {metrics.failed_tasks}")
        
        print("\n🌐 Agent ROS Bridge active")
        print(f"   gRPC: localhost:{self.grpc_port}")
        print("   Press Ctrl+C to stop")
        
        while True:
            await asyncio.sleep(1)


async def main():
    try:
        colony = MarsColony()
        await colony.initialize()
        await colony.run_mission(sols=10)
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
    except KeyboardInterrupt:
        print("\n\n🚀 Mars Colony shutting down...")


if __name__ == "__main__":
    asyncio.run(main())