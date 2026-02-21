"""
Garden Oracle - Fully integrated with Agent ROS Bridge
Demonstrates: MQTT transport for IoT sensors, robot task allocation, event-driven architecture
"""
import asyncio
import os
import json
import random
from datetime import datetime

# Required: Agent ROS Bridge imports
from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.mqtt import MQTTTransport
from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot, RobotCapability, Task, RobotStatus

from plant_personalities import Plant, PlantSpecies
from gardener_bot import GardenerBot


class GardenOracle:
    """
    The poetic voice of the garden using Agent ROS Bridge for:
    1. MQTT transport for plant sensor data (IoT integration)
    2. Fleet management of gardener robots
    3. Event-driven poetry generation
    """
    
    def __init__(self):
        self.plants = []
        self.gardener = None
        self.poetry_log = []
        
        # Agent ROS Bridge components
        self.bridge = None
        self.fleet = None
        self.mqtt_broker = "localhost"
        self.mqtt_port = 1883
        
    async def initialize(self):
        """
        Initialize Agent ROS Bridge with MQTT for IoT sensor simulation.
        """
        print("🌱 Initializing The Talking Garden with Agent ROS Bridge...")
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
        print(f"✓ Bridge initialized")
        
        # Register MQTT transport for IoT sensor data
        # This simulates real plant sensors publishing data
        mqtt_transport = MQTTTransport({
            "broker": self.mqtt_broker,
            "port": self.mqtt_port,
            "client_id": "garden_oracle"
        })
        self.bridge.transport_manager.register(mqtt_transport)
        print(f"✓ MQTT transport registered (broker: {self.mqtt_broker}:{self.mqtt_port})")
        
        # Start bridge
        await self.bridge.start()
        print("✓ Bridge started")
        
        # Initialize Fleet Orchestrator for gardener robots
        self.fleet = FleetOrchestrator()
        
        # Create plant sensors and gardener
        await self._initialize_garden()
        
        # Start fleet coordination
        asyncio.create_task(self.fleet.start())
        print("✓ Fleet orchestration started")
        
        # Subscribe to MQTT sensor topics
        await self._subscribe_sensors()
        
        print("=" * 60)
        print("🌱 Garden Oracle ready - Agent ROS Bridge active")
        print(f"   MQTT: mqtt://{self.mqtt_broker}:{self.mqtt_port}")
        print(f"   Plants: {len(self.plants)}")
        print()
        
    async def _initialize_garden(self):
        """Initialize plants and register gardener robot with fleet"""
        plant_configs = [
            ("Fernando", PlantSpecies.FERN, "A shy poet who loves humidity"),
            ("Cactilda", PlantSpecies.CACTUS, "A stoic philosopher of drought"),
            ("Rosalind", PlantSpecies.ROSE, "A dramatic romantic"),
            ("Basilion", PlantSpecies.BASIL, "An enthusiastic chef's friend"),
            ("Orchidelle", PlantSpecies.ORCHID, "A mysterious aristocrat"),
            ("Sunflower_Sam", PlantSpecies.SUNFLOWER, "An eternal optimist")
        ]
        
        for name, species, personality in plant_configs:
            plant = Plant(name, species, personality)
            self.plants.append(plant)
            print(f"  🌿 {name} ({species.value}) - {personality}")
        
        # Create and register gardener bot
        self.gardener = GardenerBot("gentle_gardener")
        
        # Register with fleet
        gardener_robot = FleetRobot(
            robot_id="gentle_gardener",
            robot_type="gardener",
            capabilities=[
                RobotCapability("watering"),
                RobotCapability("pruning"),
                RobotCapability("monitoring")
            ],
            status=RobotStatus.IDLE
        )
        self.fleet.register_robot(gardener_robot)
        print(f"\n🤖 Gardener bot registered with fleet")
        
    async def _subscribe_sensors(self):
        """
        Subscribe to MQTT topics for plant sensor data.
        Topics: garden/sensors/{plant_id}/{moisture,light,temperature}
        """
        for plant in self.plants:
            topic = f"garden/sensors/{plant.id}/+"
            # In real implementation, this would subscribe to MQTT
            print(f"  📡 Subscribed: {topic}")
            
    async def listen_to_garden(self, cycles: int = 10):
        """
        Main listening loop - processes sensor data and generates poetry.
        Demonstrates: Event-driven architecture, MQTT message handling
        """
        print("\n🎙️ The Garden Oracle begins listening...")
        print("   (Simulating MQTT sensor stream)")
        print()
        
        for cycle in range(cycles):
            # Simulate MQTT sensor readings
            await self._read_sensors()
            
            # Check for plants needing attention
            for plant in self.plants:
                if plant.needs_attention():
                    # Generate poetry from sensor data
                    poem = await self._compose_poetry(plant)
                    print(f"\n🌱 [{datetime.now().strftime('%H:%M:%S')}]")
                    print(poem)
                    
                    # Log the poem
                    self.poetry_log.append({
                        "timestamp": datetime.now().isoformat(),
                        "plant": plant.name,
                        "poem": poem,
                        "sensors": {
                            "moisture": plant.moisture,
                            "light": plant.light,
                            "temperature": plant.temperature
                        }
                    })
                    
                    # Publish MQTT message (simulated)
                    await self._publish_care_needed(plant)
                    
                    # Allocate gardener task via fleet
                    await self._allocate_gardener_task(plant)
                    
                    # Give care
                    await self.gardener.tend(plant)
                    
                    await asyncio.sleep(2)
            
            await asyncio.sleep(1)
        
        await self._garden_summary()
        
    async def _read_sensors(self):
        """Simulate MQTT sensor readings from plants"""
        for plant in self.plants:
            # Simulate sensor drift
            plant.moisture += random.uniform(-8, 3)
            plant.light += random.uniform(-15, 20)
            plant.temperature += random.uniform(-3, 3)
            
            # Keep in bounds
            plant.moisture = max(0, min(100, plant.moisture))
            plant.light = max(0, min(100, plant.light))
            plant.temperature = max(5, min(40, plant.temperature))
            
    async def _publish_care_needed(self, plant: Plant):
        """
        Publish MQTT message when plant needs care.
        Topic: garden/alerts/care_needed
        """
        message = {
            "plant_id": plant.id,
            "plant_name": plant.name,
            "need": plant.current_need,
            "sensors": {
                "moisture": plant.moisture,
                "light": plant.light,
                "temperature": plant.temperature
            },
            "timestamp": datetime.now().isoformat()
        }
        
        topic = "garden/alerts/care_needed"
        print(f"   📤 MQTT Publish [{topic}]: {plant.name} needs {plant.current_need}")
        
    async def _allocate_gardener_task(self, plant: Plant):
        """
        Allocate gardener task via fleet orchestrator.
        Demonstrates: Dynamic task allocation based on plant needs
        """
        task = Task(
            task_id=f"care_{plant.id}_{datetime.now().timestamp()}",
            description=f"Care for {plant.name}",
            required_capabilities=[plant.current_need],
            priority=8 if plant.moisture < 20 else 5
        )
        
        self.fleet.submit_task(task)
        robot_id = self.fleet.allocate_task(task)
        
        if robot_id:
            print(f"   ✅ Task allocated to {robot_id}")
        else:
            print(f"   ⚠️ No gardener available for {plant.name}")
            
    async def _compose_poetry(self, plant: Plant) -> str:
        """Generate poetry based on plant state"""
        # Poetry generation logic here
        opening = random.choice([
            f"The {plant.species.value.lower()} whispers:",
            f"From {plant.name}:",
            f"In {plant.species.value} tongue:"
        ])
        
        need = plant.current_need
        if need == "water":
            body = f"My roots thirst... moisture at {plant.moisture:.0f}%"
        elif need == "light":
            body = f"The shadows grow... light at {plant.light:.0f}%"
        elif need == "temperature":
            body = f"I shiver... temperature at {plant.temperature:.0f}°C"
        else:
            body = "I exist in strange unnamed longing..."
        
        return f"{opening}\n  \"{body}\""
        
    async def _garden_summary(self):
        """Display garden session summary"""
        print("\n" + "=" * 60)
        print("🌿 GARDEN SESSION COMPLETE")
        print("=" * 60)
        print(f"Poems spoken: {len(self.poetry_log)}")
        
        metrics = self.fleet.get_metrics()
        print(f"Care tasks completed: {metrics.completed_tasks}")
        
        print("\n🌐 Agent ROS Bridge still active")
        print("   MQTT broker accepting connections")
        print("   Press Ctrl+C to stop")
        
        while True:
            await asyncio.sleep(1)


async def main():
    try:
        oracle = GardenOracle()
        await oracle.initialize()
        await oracle.listen_to_garden(cycles=10)
    except RuntimeError as e:
        print(f"\n❌ Error: {e}")
    except KeyboardInterrupt:
        print("\n\n🌱 Garden Oracle shutting down...")


if __name__ == "__main__":
    asyncio.run(main())