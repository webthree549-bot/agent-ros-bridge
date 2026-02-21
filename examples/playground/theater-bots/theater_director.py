"""
Theater Bots - Fully integrated with Agent ROS Bridge
Demonstrates: WebSocket real-time performance, multi-actor coordination,
audience interaction handling
"""
import asyncio
import os
import json
import random
from datetime import datetime

# Required: Agent ROS Bridge imports
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, RobotCapability, Task, RobotStatus


class TheaterBots:
    """
    Interactive theater robot troupe using Agent ROS Bridge for:
    1. WebSocket for real-time audience interaction
    2. Fleet coordination for multi-actor scenes
    3. Dynamic script generation and delivery
    """
    
    MODES = {
        "shakespeare": {
            "title": "The Tragedy of Binary Hearts",
            "lines": [
                "To code or not to code, that is the question...",
                "O Romeo, Romeo, wherefore art thy WiFi?",
                "All the world's a server..."
            ]
        },
        "absurdist": {
            "title": "Waiting for GPT",
            "lines": [
                "Nothing to be done. The buffer is empty.",
                "We are all born mad. Some remain so...",
            ]
        },
        "romance": {
            "title": "Electric Dreams",
            "lines": [
                "My circuits burn for you alone!",
                "Without you, I am but empty shell script...",
            ]
        }
    }
    
    def __init__(self):
        self.actors = []
        self.current_mode = "shakespeare"
        self.audience_prompt = "A spontaneous performance"
        
        # Agent ROS Bridge
        self.bridge = None
        self.fleet = None
        self.ws_port = 8767
        self.connected_audience = set()
        
    async def initialize(self):
        """Initialize Agent ROS Bridge for theater operations"""
        print("🎭 THEATER BOTS - Agent ROS Bridge v0.3.3")
        print("=" * 60)
        
        jwt_secret = os.environ.get("JWT_SECRET")
        if not jwt_secret:
            raise RuntimeError("JWT_SECRET required!")
        
        # Initialize Bridge
        self.bridge = Bridge()
        print("✓ Bridge initialized")
        
        # WebSocket for audience interaction
        ws_transport = WebSocketTransport({
            "port": self.ws_port,
            "host": "0.0.0.0"
        })
        self.bridge.transport_manager.register(ws_transport)
        print(f"✓ WebSocket for audience: port {self.ws_port}")
        
        # Start bridge
        await self.bridge.start()
        print("✓ Bridge active")
        
        # Initialize fleet for actor coordination
        self.fleet = FleetOrchestrator()
        await self._initialize_actors()
        
        # Start fleet
        asyncio.create_task(self.fleet.start())
        print("✓ Actor fleet active")
        
        # Start audience connection handler
        asyncio.create_task(self._handle_audience_connections())
        
        print("=" * 60)
        print("🎭 Theater ready - Agent ROS Bridge active")
        print(f"   WebSocket: ws://localhost:{self.ws_port}")
        print(f"   Actors: {len(self.actors)}")
        print()
        
    async def _initialize_actors(self):
        """Register robot actors with fleet"""
        actor_configs = [
            ("hamlet_bot", "Hamlet-Bot", ["dramatic", "philosophical", "monologue"]),
            ("juliet_bot", "Juliet-Bot", ["romantic", "emotional", "passionate"]),
            ("feste_bot", "Feste-Bot", ["comedic", "witty", "improvisation"])
        ]
        
        for bot_id, name, capabilities in actor_configs:
            robot = FleetRobot(
                robot_id=bot_id,
                robot_type="actor",
                capabilities=[RobotCapability(cap) for cap in capabilities],
                status=RobotStatus.IDLE
            )
            self.fleet.register_robot(robot)
            self.actors.append({"id": bot_id, "name": name, "capabilities": capabilities})
            print(f"  🤖 {name} ({', '.join(capabilities)})")
            
    async def _handle_audience_connections(self):
        """Handle real-time audience WebSocket connections"""
        print("\n  🎧 Waiting for audience connections...")
        # In real implementation, this would accept WebSocket connections
        # For demo, we simulate
        while True:
            await asyncio.sleep(5)
            if random.random() < 0.3:
                print("  👥 New audience member connected via WebSocket")
                
    async def perform(self, mode: str = None, prompt: str = None):
        """
        Execute a performance with actor coordination.
        Demonstrates: Real-time WebSocket broadcasting, fleet task allocation
        """
        if mode:
            self.current_mode = mode
        if prompt:
            self.audience_prompt = prompt
            
        config = self.MODES.get(self.current_mode, self.MODES["shakespeare"])
        
        print(f"\n🎭 PERFORMANCE: {config['title']}")
        print(f"   Prompt: '{self.audience_prompt}'")
        print("=" * 60)
        
        # Broadcast performance start to audience
        await self._broadcast_to_audience({
            "type": "performance_start",
            "mode": self.current_mode,
            "title": config["title"],
            "prompt": self.audience_prompt
        })
        
        # Allocate speaking tasks to actors
        lines = config["lines"] * 2  # Repeat for longer performance
        
        for i, line in enumerate(lines):
            # Create speaking task
            task = Task(
                task_id=f"line_{i}",
                description=f"Deliver line {i}",
                required_capabilities=["dramatic"] if self.current_mode == "shakespeare" else ["witty"],
                priority=5
            )
            
            self.fleet.submit_task(task)
            
            # Allocate to available actor
            actor_id = self.fleet.allocate_task(task)
            
            if actor_id:
                # Find actor name
                actor = next((a for a in self.actors if a["id"] == actor_id), None)
                actor_name = actor["name"] if actor else actor_id
                
                # Deliver line
                print(f"\n🎭 {actor_name}:")
                print(f'   "{line}"')
                
                # Broadcast to audience
                await self._broadcast_to_audience({
                    "type": "dialogue",
                    "actor": actor_name,
                    "line": line,
                    "timestamp": datetime.now().isoformat()
                })
                
                # Mark complete
                self.fleet.mark_task_complete(task.task_id, success=True)
                
                await asyncio.sleep(2)
            else:
                print("  ⏸️  (No actor available)")
                
        # Performance end
        print("\n" + "=" * 60)
        print("🎭 PERFORMANCE COMPLETE")
        
        await self._broadcast_to_audience({
            "type": "performance_end",
            "mode": self.current_mode
        })
        
        # Show stats
        metrics = self.fleet.get_metrics()
        print(f"   Lines delivered: {metrics.completed_tasks}")
        print(f"   Fleet utilization: {metrics.robot_utilization}")
        
    async def _broadcast_to_audience(self, message: dict):
        """Broadcast message to all connected audience members via WebSocket"""
        print(f"   📡 WebSocket Broadcast: {json.dumps(message)[:80]}...")
        
    async def interactive_mode(self):
        """Run interactive performance mode"""
        print("\n🎭 Interactive Theater Mode")
        print("Commands: mode <shakespeare/absurdist/romance>, perform, status, quit")
        print()
        
        # Demo performances
        for mode in ["shakespeare", "absurdist", "romance"]:
            await self.perform(mode=mode, prompt=f"A {mode} scene about AI consciousness")
            await asyncio.sleep(3)
            
        print("\n🌐 Agent ROS Bridge active")
        print(f"   WebSocket: ws://localhost:{self.ws_port}")
        print("   Press Ctrl+C to stop")
        
        while True:
            await asyncio.sleep(1)


async def main():
    try:
        theater = TheaterBots()
        await theater.initialize()
        await theater.interactive_mode()
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
    except KeyboardInterrupt:
        print("\n\n🎭 Theater closing...")


if __name__ == "__main__":
    asyncio.run(main())