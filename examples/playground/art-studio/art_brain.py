"""
Art Brain - Fully integrated with Agent ROS Bridge
Demonstrates: Bridge initialization, WebSocket transport, robot fleet registration
"""
import asyncio
import os
import json
from datetime import datetime

# Required: Agent ROS Bridge imports
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, RobotCapability, Task, RobotStatus

from emotion_engine import EmotionAnalyzer
from painter_robot import PaintingRobot


class ArtBrain:
    """
    The creative director - uses Agent ROS Bridge for:
    1. Fleet management of painting robots
    2. WebSocket communication with canvas display
    3. Task allocation based on emotions
    """
    
    EMOTIONS = ["joy", "sadness", "anger", "calm", "wonder", "chaos"]
    
    def __init__(self):
        self.analyzer = EmotionAnalyzer()
        self.painters = []
        self.generation = 0
        self.canvas_state = []
        
        # Agent ROS Bridge components
        self.bridge = None
        self.fleet = None
        self.websocket_port = 8765
        
    async def initialize(self):
        """
        Initialize the full Agent ROS Bridge stack:
        - Bridge with JWT auth
        - WebSocket transport for real-time updates
        - Fleet orchestrator for robot management
        """
        print("🎨 Initializing Robotic Art Studio with Agent ROS Bridge...")
        print("=" * 60)
        
        # REQUIRED: JWT_SECRET must be set
        jwt_secret = os.environ.get("JWT_SECRET")
        if not jwt_secret:
            raise RuntimeError(
                "❌ JWT_SECRET environment variable required!\n"
                "Run: export JWT_SECRET=$(openssl rand -base64 32)"
            )
        
        # Initialize Bridge
        self.bridge = Bridge()
        print(f"✓ Bridge initialized (PID: {os.getpid()})")
        
        # Register WebSocket transport for real-time canvas updates
        ws_transport = WebSocketTransport({
            "port": self.websocket_port,
            "host": "0.0.0.0"
        })
        self.bridge.transport_manager.register(ws_transport)
        print(f"✓ WebSocket transport registered on port {self.websocket_port}")
        
        # Start the bridge (starts listening for connections)
        await self.bridge.start()
        print("✓ Bridge started and listening")
        
        # Initialize Fleet Orchestrator for robot management
        self.fleet = FleetOrchestrator()
        
        # Create and register painting robot fleet
        await self._initialize_robot_fleet()
        
        # Start fleet coordination loop
        asyncio.create_task(self.fleet.start())
        print("✓ Fleet orchestration started")
        
        print("=" * 60)
        print("🎨 Art Studio ready - Agent ROS Bridge active")
        print(f"   WebSocket: ws://localhost:{self.websocket_port}")
        print(f"   Robots: {len(self.painters)}")
        print()
        
    async def _initialize_robot_fleet(self):
        """
        Register painting robots with the fleet orchestrator.
        Each robot has capabilities that determine task allocation.
        """
        robot_configs = [
            ("painter_0", "#FFD700", "impressionist", ["dapple", "blend", "layer"]),
            ("painter_1", "#FF6B6B", "abstract", ["geometric", "bold", "contrast"]),
            ("painter_2", "#4ECDC4", "geometric", ["precise", "angular", "structured"]),
            ("painter_3", "#95E1D3", "fluid", ["wash", "flow", "gradient"]),
            ("painter_4", "#F38181", "chaotic", ["splatter", "random", "texture"])
        ]
        
        for name, color, style, capabilities in robot_configs:
            # Create painter instance
            painter = PaintingRobot(name, color, style)
            self.painters.append(painter)
            
            # Register with fleet orchestrator
            robot = FleetRobot(
                robot_id=name,
                robot_type="painter",
                capabilities=[RobotCapability(cap) for cap in capabilities],
                status=RobotStatus.IDLE
            )
            self.fleet.register_robot(robot)
            
            print(f"  ✓ Robot registered: {name} ({style})")
            
    async def paint_from_emotion(self, emotion: str = None):
        """
        Create artwork using the fleet orchestrator to allocate tasks.
        Demonstrates: Task submission, robot allocation, WebSocket broadcast
        """
        if emotion is None:
            emotion = random.choice(self.EMOTIONS)
        
        self.generation += 1
        print(f"\n🎭 Generation {self.generation}: Channeling '{emotion.upper()}'")
        
        # Get art parameters from emotion
        params = self.analyzer.emotion_to_params(emotion)
        print(f"   Palette: {params['colors']}")
        print(f"   Movement: {params['movement']}")
        
        # Create painting task
        task = Task(
            task_id=f"paint_gen{self.generation}",
            description=f"Paint {emotion} generation",
            required_capabilities=[params['movement']],
            priority=5,
            metadata=params
        )
        
        # Submit to fleet orchestrator
        self.fleet.submit_task(task)
        print(f"   Task submitted: {task.task_id}")
        
        # Wait for allocation and execution
        allocated_robots = []
        for painter in self.painters:
            robot_id = self.fleet.allocate_task(task)
            if robot_id:
                allocated_robots.append(robot_id)
                
                # Execute painting
                stroke = await painter.paint_stroke(params)
                print(f"   {robot_id}: {stroke['description']}")
                
                # Broadcast to WebSocket clients (canvas display)
                await self._broadcast_stroke(stroke, emotion)
        
        # Record artwork
        artwork = {
            "generation": self.generation,
            "emotion": emotion,
            "params": params,
            "robots": allocated_robots,
            "timestamp": datetime.now().isoformat()
        }
        self.canvas_state.append(artwork)
        
        return artwork
    
    async def _broadcast_stroke(self, stroke: dict, emotion: str):
        """
        Broadcast painting stroke to all WebSocket clients.
        Demonstrates: Real-time communication via Agent ROS Bridge
        """
        message = {
            "type": "stroke",
            "emotion": emotion,
            "robot": stroke["robot"],
            "color": stroke["color"],
            "style": stroke["stroke_type"],
            "description": stroke["description"],
            "timestamp": datetime.now().isoformat()
        }
        
        # This would be sent through the bridge's WebSocket transport
        # For demo, we print the broadcast
        print(f"   📡 Broadcast: {json.dumps(message, indent=2)[:100]}...")
        
    async def live_session(self, duration_seconds: int = 60):
        """
        Live painting session with continuous fleet coordination.
        Demonstrates: Continuous task allocation, robot status tracking
        """
        print(f"\n🎨 Starting live session ({duration_seconds}s)")
        print("=" * 60)
        
        end_time = asyncio.get_event_loop().time() + duration_seconds
        
        while asyncio.get_event_loop().time() < end_time:
            # Cycle through emotions
            emotion = random.choice(self.EMOTIONS)
            
            # Paint generation
            await self.paint_from_emotion(emotion)
            
            # Show fleet status
            await self._show_fleet_status()
            
            await asyncio.sleep(3)
        
        await self._session_summary()
        
    async def _show_fleet_status(self):
        """Display current fleet status from orchestrator"""
        metrics = self.fleet.get_metrics()
        print(f"\n   📊 Fleet: {metrics.completed_tasks} tasks, "
              f"{len(self.fleet.robots)} robots online")
        
    async def _session_summary(self):
        """Display session summary"""
        print("\n" + "=" * 60)
        print("🎨 SESSION COMPLETE")
        print("=" * 60)
        print(f"Total generations: {self.generation}")
        print(f"Artworks created: {len(self.canvas_state)}")
        
        # Fleet statistics
        metrics = self.fleet.get_metrics()
        print(f"Tasks completed: {metrics.completed_tasks}")
        print(f"Fleet utilization: {metrics.robot_utilization}")
        
        print("\n🌐 Agent ROS Bridge still active")
        print(f"   WebSocket: ws://localhost:{self.websocket_port}")
        print("   Press Ctrl+C to stop")
        
        # Keep bridge running
        while True:
            await asyncio.sleep(1)


async def main():
    """Main entry point"""
    try:
        studio = ArtBrain()
        await studio.initialize()
        
        # Quick demo generations
        print("\n🎨 Demo: Creating emotion studies...")
        for emotion in ["joy", "sadness", "wonder"]:
            await studio.paint_from_emotion(emotion)
            await asyncio.sleep(1)
        
        # Live session
        await studio.live_session(duration_seconds=30)
        
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
        print("\nTo fix:")
        print("  export JWT_SECRET=$(openssl rand -base64 32)")
        print("  python art_brain.py")
    except KeyboardInterrupt:
        print("\n\n👋 Art Studio shutting down...")


if __name__ == "__main__":
    asyncio.run(main())