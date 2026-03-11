"""Example: Using LCM Transport and Blueprint patterns.

This example demonstrates the new architectural patterns inspired by dimos,
adapted for ROS compatibility.
"""

import asyncio
from dataclasses import dataclass
from typing import Optional

# New imports
from agent_ros_bridge.gateway_v2.module import Module, In, Out, rpc, skill
from agent_ros_bridge.gateway_v2.blueprint import Blueprint, autoconnect
from agent_ros_bridge.gateway_v2.transports.lcm_transport import LCMTransport


# Define message types
@dataclass
class Image:
    """Camera image message."""
    width: int
    height: int
    data: bytes
    timestamp: float


@dataclass
class Detection:
    """Object detection message."""
    class_name: str
    confidence: float
    bbox: tuple  # (x, y, w, h)


@dataclass
class Twist:
    """Velocity command."""
    linear_x: float
    angular_z: float


# Example 1: Simple Module with Streams
class CameraModule(Module):
    """Camera module with image output stream."""
    
    # Define streams using type annotations (like dimos)
    image: Out[Image]
    config: In[dict]
    
    def __init__(self, camera_id: str = "camera_0", fps: int = 30):
        super().__init__(name=f"camera_{camera_id}")
        self.camera_id = camera_id
        self.fps = fps
        self._capture_task: Optional[asyncio.Task] = None
    
    @rpc
    def set_exposure(self, exposure: float) -> bool:
        """RPC: Set camera exposure."""
        print(f"Setting exposure to {exposure}")
        return True
    
    @skill
    def capture_photo(self) -> Image:
        """AI-callable skill: Capture a single photo."""
        # Simulate capture
        return Image(
            width=1920,
            height=1080,
            data=b'fake_image_data',
            timestamp=asyncio.get_event_loop().time()
        )
    
    async def run(self) -> None:
        """Main loop: Continuously capture images."""
        while self._running:
            # Simulate image capture
            img = Image(
                width=1920,
                height=1080,
                data=b'fake_frame_data',
                timestamp=asyncio.get_event_loop().time()
            )
            
            # Publish to stream
            await self.image.publish(img)
            
            # Wait for next frame
            await asyncio.sleep(1.0 / self.fps)


# Example 2: Detector Module
class DetectorModule(Module):
    """Object detector module."""
    
    image: In[Image]
    detections: Out[Detection]
    
    def __init__(self, model: str = "yolo"):
        super().__init__(name="detector")
        self.model = model
    
    async def run(self) -> None:
        """Main loop: Process images and output detections."""
        while self._running:
            # Get image from stream
            img = await self.image.get()
            
            # Simulate detection
            detection = Detection(
                class_name="person",
                confidence=0.95,
                bbox=(100, 100, 50, 100)
            )
            
            # Publish detection
            await self.detections.publish(detection)


# Example 3: Navigation Module
class NavigationModule(Module):
    """Navigation module with velocity output."""
    
    target: In[dict]  # Target position
    cmd_vel: Out[Twist]
    
    def __init__(self, max_speed: float = 1.0):
        super().__init__(name="navigation")
        self.max_speed = max_speed
    
    @skill
    def navigate_to(self, x: float, y: float) -> bool:
        """AI-callable skill: Navigate to position."""
        print(f"Navigating to ({x}, {y})")
        return True
    
    async def run(self) -> None:
        """Main loop: Generate velocity commands."""
        while self._running:
            # Simulate navigation
            cmd = Twist(
                linear_x=0.5,
                angular_z=0.1
            )
            await self.cmd_vel.publish(cmd)
            await asyncio.sleep(0.1)


# Example 4: Using Blueprints
async def example_blueprint():
    """Example: Building a robot system with blueprints."""
    
    # Create module blueprints
    camera_bp = CameraModule.blueprint(camera_id="front", fps=30)
    detector_bp = DetectorModule.blueprint(model="yolo_v8")
    nav_bp = NavigationModule.blueprint(max_speed=1.5)
    
    # Method 1: Manual blueprint construction
    blueprint = Blueprint()
    blueprint.add_module("camera", camera_bp)
    blueprint.add_module("detector", detector_bp)
    blueprint.add_module("navigation", nav_bp)
    
    # Connect streams manually
    blueprint.connect("camera", "image", "detector", "image")
    
    # Use LCM transport for image streams
    blueprint.transports({
        ("image", Image): LCMTransport({"udp_url": "udpm://239.255.76.67:7667"})
    })
    
    # Build and start
    await blueprint.start()
    
    print("Robot system running with blueprint pattern!")
    print(f"Modules: {list(blueprint._instances.keys())}")
    
    # Run for 10 seconds
    await asyncio.sleep(10)
    
    await blueprint.stop()


# Example 5: Using autoconnect
async def example_autoconnect():
    """Example: Using autoconnect for automatic wiring."""
    
    # Create blueprints
    camera_bp = CameraModule.blueprint()
    detector_bp = DetectorModule.blueprint()
    
    # Autoconnect matches streams by name and type
    blueprint = autoconnect(camera_bp, detector_bp)
    
    # Start
    await blueprint.start()
    
    print("Autoconnected system running!")
    
    await asyncio.sleep(10)
    
    await blueprint.stop()


# Example 6: LCM Transport Usage
async def example_lcm_transport():
    """Example: Using LCM transport directly."""
    
    # Create LCM transport
    transport = LCMTransport({
        "udp_url": "udpm://239.255.76.67:7667",
        "shared_memory": True
    })
    
    # Start transport
    await transport.start()
    
    # Create publisher
    pub = transport.publisher("robot/commands")
    
    # Create subscriber
    def on_command(data):
        print(f"Received command: {data}")
    
    sub = transport.subscriber("robot/commands", on_command)
    sub.subscribe()
    
    # Publish messages
    for i in range(10):
        pub.publish({"type": "move", "speed": 0.5})
        await asyncio.sleep(0.5)
    
    await transport.stop()


# Example 7: Mixed ROS and LCM
async def example_mixed_transports():
    """Example: Using both ROS and LCM transports."""
    
    from agent_ros_bridge import Bridge
    from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
    
    # Create bridge with multiple transports
    bridge = Bridge()
    
    # WebSocket for external AI agents
    bridge.transport_manager.register(
        WebSocketTransport({"port": 8765})
    )
    
    # LCM for internal high-performance communication
    bridge.transport_manager.register(
        LCMTransport({"udp_url": "udpm://239.255.76.67:7667"})
    )
    
    await bridge.start()
    
    print("Bridge running with WebSocket + LCM transports")
    
    # Keep running
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Agent ROS Bridge - Blueprint & LCM Examples")
    print("=" * 60)
    
    # Run examples
    print("\n1. Blueprint Example:")
    asyncio.run(example_blueprint())
    
    print("\n2. Autoconnect Example:")
    asyncio.run(example_autoconnect())
    
    print("\n3. LCM Transport Example:")
    asyncio.run(example_lcm_transport())
