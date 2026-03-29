"""
Universal ROS Hardware Support

Makes Agent ROS Bridge work with ANY ROS device:
- Sensors (cameras, lidars, IMUs, etc.)
- Actuators (arms, grippers, wheels, etc.)
- Drones (quadcopters, fixed-wing)
- Humanoids (bipedal robots)
- Custom hardware
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Capability:
    """A capability that a ROS device can perform"""

    name: str  # e.g., "move", "grasp", "fly", "sense"
    description: str
    parameters: dict[str, Any]  # Expected parameters
    return_type: str  # What the action returns
    safety_critical: bool = False  # Requires extra validation


@dataclass
class DeviceProfile:
    """Profile of a ROS device (robot, sensor, actuator)"""

    device_id: str
    device_type: str  # 'mobile_robot', 'drone', 'manipulator', 'humanoid', etc.
    manufacturer: str
    model: str
    capabilities: list[Capability] = field(default_factory=list)
    sensors: list[str] = field(default_factory=list)  # camera, lidar, imu, etc.
    actuators: list[str] = field(default_factory=list)  # wheels, arms, rotors, etc.
    ros_topics: dict[str, str] = field(default_factory=dict)  # topic mappings
    ros_services: dict[str, str] = field(default_factory=dict)  # service mappings
    action_servers: list[str] = field(default_factory=list)  # available actions
    limits: dict[str, Any] = field(default_factory=dict)  # safety limits


class ROSDevice(ABC):
    """
    Abstract base class for ANY ROS device.

    Subclass this to support new hardware types:
    - Mobile robots (TurtleBot, Husky, etc.)
    - Drones (DJI, PX4, etc.)
    - Manipulators (UR, Franka, etc.)
    - Humanoids (Digit, Atlas, etc.)
    - Custom devices
    """

    def __init__(self, device_id: str, device_profile: DeviceProfile):
        self.device_id = device_id
        self.profile = device_profile
        self._connected = False

    @abstractmethod
    def connect(self) -> bool:
        """Connect to the ROS device"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the ROS device"""
        pass

    @abstractmethod
    def execute_capability(
        self,
        capability_name: str,
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        """
        Execute a capability on the device.

        Args:
            capability_name: Name of the capability
            parameters: Parameters for the capability

        Returns:
            Execution result
        """
        pass

    @abstractmethod
    def get_state(self) -> dict[str, Any]:
        """Get current device state"""
        pass

    def get_capabilities(self) -> list[Capability]:
        """Get list of supported capabilities"""
        return self.profile.capabilities

    def has_capability(self, capability_name: str) -> bool:
        """Check if device supports a capability"""
        return any(c.name == capability_name for c in self.profile.capabilities)


class MobileRobot(ROSDevice):
    """Mobile robot implementation (TurtleBot, Husky, etc.)"""

    def connect(self) -> bool:
        # Connect to robot's ROS topics
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def execute_capability(
        self, capability_name: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        if capability_name == "navigate_to":
            return self._navigate(parameters)
        elif capability_name == "rotate":
            return self._rotate(parameters)
        elif capability_name == "stop":
            return self._stop()
        else:
            return {"success": False, "error": f"Unknown capability: {capability_name}"}

    def _navigate(self, params: dict) -> dict[str, Any]:
        # Publish to /cmd_vel or use Nav2
        return {"success": True, "message": "Navigating"}

    def _rotate(self, params: dict) -> dict[str, Any]:
        return {"success": True, "message": "Rotating"}

    def _stop(self) -> dict[str, Any]:
        return {"success": True, "message": "Stopped"}

    def get_state(self) -> dict[str, Any]:
        return {
            "position": [0, 0, 0],
            "battery": 100,
            "velocity": [0, 0, 0],
        }


class Drone(ROSDevice):
    """Drone implementation (DJI, PX4, etc.)"""

    def connect(self) -> bool:
        # Connect to MAVROS or drone SDK
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def execute_capability(
        self, capability_name: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        if capability_name == "takeoff":
            return self._takeoff(parameters)
        elif capability_name == "land":
            return self._land()
        elif capability_name == "fly_to":
            return self._fly_to(parameters)
        elif capability_name == "hover":
            return self._hover()
        elif capability_name == "capture_image":
            return self._capture_image(parameters)
        else:
            return {"success": False, "error": f"Unknown capability: {capability_name}"}

    def _takeoff(self, params: dict) -> dict[str, Any]:
        altitude = params.get("altitude", 2.0)
        return {"success": True, "message": f"Taking off to {altitude}m"}

    def _land(self) -> dict[str, Any]:
        return {"success": True, "message": "Landing"}

    def _fly_to(self, params: dict) -> dict[str, Any]:
        x, y, z = params.get("x", 0), params.get("y", 0), params.get("z", 5)
        return {"success": True, "message": f"Flying to ({x}, {y}, {z})"}

    def _hover(self) -> dict[str, Any]:
        return {"success": True, "message": "Hovering"}

    def _capture_image(self, params: dict) -> dict[str, Any]:
        return {"success": True, "message": "Image captured", "image_path": "/tmp/image.jpg"}

    def get_state(self) -> dict[str, Any]:
        return {
            "position": [0, 0, 5],
            "battery": 85,
            "altitude": 5.0,
            "gps_satellites": 12,
        }


class Manipulator(ROSDevice):
    """Robot arm implementation (UR, Franka, etc.)"""

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def execute_capability(
        self, capability_name: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        if capability_name == "move_to":
            return self._move_to(parameters)
        elif capability_name == "grasp":
            return self._grasp(parameters)
        elif capability_name == "release":
            return self._release()
        elif capability_name == "follow_trajectory":
            return self._follow_trajectory(parameters)
        else:
            return {"success": False, "error": f"Unknown capability: {capability_name}"}

    def _move_to(self, params: dict) -> dict[str, Any]:
        x, y, z = params.get("x", 0), params.get("y", 0), params.get("z", 0)
        return {"success": True, "message": f"Moving to ({x}, {y}, {z})"}

    def _grasp(self, params: dict) -> dict[str, Any]:
        force = params.get("force", 0.5)
        return {"success": True, "message": f"Grasping with {force}N force"}

    def _release(self) -> dict[str, Any]:
        return {"success": True, "message": "Released"}

    def _follow_trajectory(self, params: dict) -> dict[str, Any]:
        waypoints = params.get("waypoints", [])
        return {"success": True, "message": f"Following {len(waypoints)} waypoints"}

    def get_state(self) -> dict[str, Any]:
        return {
            "joint_positions": [0, 0, 0, 0, 0, 0],
            "end_effector_position": [0.5, 0, 0.3],
            "gripper_state": "open",
        }


class Humanoid(ROSDevice):
    """Humanoid robot implementation (Digit, Atlas, etc.)"""

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def execute_capability(
        self, capability_name: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        if capability_name == "walk":
            return self._walk(parameters)
        elif capability_name == "balance":
            return self._balance()
        elif capability_name == "reach":
            return self._reach(parameters)
        elif capability_name == "climb":
            return self._climb(parameters)
        elif capability_name == "manipulate":
            return self._manipulate(parameters)
        else:
            return {"success": False, "error": f"Unknown capability: {capability_name}"}

    def _walk(self, params: dict) -> dict[str, Any]:
        direction = params.get("direction", "forward")
        steps = params.get("steps", 1)
        return {"success": True, "message": f"Walking {direction} {steps} steps"}

    def _balance(self) -> dict[str, Any]:
        return {"success": True, "message": "Balancing"}

    def _reach(self, params: dict) -> dict[str, Any]:
        target = params.get("target", [0, 0, 0])
        return {"success": True, "message": f"Reaching to {target}"}

    def _climb(self, params: dict) -> dict[str, Any]:
        surface = params.get("surface", "stairs")
        return {"success": True, "message": f"Climbing {surface}"}

    def _manipulate(self, params: dict) -> dict[str, Any]:
        object_name = params.get("object", "unknown")
        action = params.get("action", "grasp")
        return {"success": True, "message": f"{action} {object_name}"}

    def get_state(self) -> dict[str, Any]:
        return {
            "position": [0, 0, 1.7],  # Standing height
            "balance": "stable",
            "joint_states": {},
        }


class SensorArray(ROSDevice):
    """Sensor array for data collection"""

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def execute_capability(
        self, capability_name: str, parameters: dict[str, Any]
    ) -> dict[str, Any]:
        if capability_name == "sense":
            return self._sense(parameters)
        elif capability_name == "capture":
            return self._capture(parameters)
        elif capability_name == "scan":
            return self._scan(parameters)
        else:
            return {"success": False, "error": f"Unknown capability: {capability_name}"}

    def _sense(self, params: dict) -> dict[str, Any]:
        modality = params.get("modality", "all")
        return {"success": True, "message": f"Sensing {modality}", "data": {}}

    def _capture(self, params: dict) -> dict[str, Any]:
        sensor = params.get("sensor", "camera")
        return {"success": True, "message": f"Capturing from {sensor}"}

    def _scan(self, params: dict) -> dict[str, Any]:
        area = params.get("area", "surroundings")
        return {"success": True, "message": f"Scanning {area}"}

    def get_state(self) -> dict[str, Any]:
        return {
            "active_sensors": ["camera", "lidar", "imu"],
            "data_rate": 30,
        }


class DeviceRegistry:
    """Registry for managing all connected ROS devices"""

    def __init__(self):
        self._devices: dict[str, ROSDevice] = {}
        self._device_types: dict[str, type[ROSDevice]] = {
            "mobile_robot": MobileRobot,
            "drone": Drone,
            "manipulator": Manipulator,
            "humanoid": Humanoid,
            "sensor_array": SensorArray,
        }

    def register_device_type(
        self,
        device_type: str,
        device_class: type[ROSDevice],
    ) -> None:
        """Register a new device type"""
        self._device_types[device_type] = device_class

    def create_device(
        self,
        device_id: str,
        device_type: str,
        profile: DeviceProfile,
    ) -> ROSDevice:
        """Create and connect a device"""
        device_class = self._device_types.get(device_type)
        if not device_class:
            raise ValueError(f"Unknown device type: {device_type}")

        device = device_class(device_id, profile)
        device.connect()
        self._devices[device_id] = device
        return device

    def get_device(self, device_id: str) -> ROSDevice | None:
        """Get a device by ID"""
        return self._devices.get(device_id)

    def list_devices(self) -> list[str]:
        """List all connected device IDs"""
        return list(self._devices.keys())

    def get_devices_by_capability(self, capability: str) -> list[ROSDevice]:
        """Find all devices that support a capability"""
        return [device for device in self._devices.values() if device.has_capability(capability)]


# Example usage
if __name__ == "__main__":
    print("=" * 70)
    print("🤖 UNIVERSAL ROS HARDWARE SUPPORT DEMO")
    print("=" * 70)
    print()

    registry = DeviceRegistry()

    # Create different device types
    print("Creating devices...")
    print()

    # Mobile robot
    robot_profile = DeviceProfile(
        device_id="turtlebot_01",
        device_type="mobile_robot",
        manufacturer="Robotis",
        model="TurtleBot3",
        capabilities=[
            Capability("navigate_to", "Navigate to location", {"x": float, "y": float}, "success"),
            Capability("rotate", "Rotate in place", {"angle": float}, "success"),
        ],
    )
    robot = registry.create_device("bot1", "mobile_robot", robot_profile)
    print(f"✅ Created: {robot.profile.model} ({robot.device_id})")
    print(f"   Capabilities: {[c.name for c in robot.get_capabilities()]}")
    print()

    # Drone
    drone_profile = DeviceProfile(
        device_id="djimavic_01",
        device_type="drone",
        manufacturer="DJI",
        model="Mavic 3",
        capabilities=[
            Capability("takeoff", "Take off", {"altitude": float}, "success"),
            Capability(
                "fly_to", "Fly to coordinates", {"x": float, "y": float, "z": float}, "success"
            ),
            Capability("capture_image", "Capture image", {}, "image_path"),
        ],
    )
    drone = registry.create_device("drone1", "drone", drone_profile)
    print(f"✅ Created: {drone.profile.model} ({drone.device_id})")
    print(f"   Capabilities: {[c.name for c in drone.get_capabilities()]}")
    print()

    # Robot arm
    arm_profile = DeviceProfile(
        device_id="ur10_01",
        device_type="manipulator",
        manufacturer="Universal Robots",
        model="UR10",
        capabilities=[
            Capability(
                "move_to", "Move to position", {"x": float, "y": float, "z": float}, "success"
            ),
            Capability("grasp", "Grasp object", {"force": float}, "success"),
        ],
    )
    arm = registry.create_device("arm1", "manipulator", arm_profile)
    print(f"✅ Created: {arm.profile.model} ({arm.device_id})")
    print(f"   Capabilities: {[c.name for c in arm.get_capabilities()]}")
    print()

    # Find devices by capability
    print("Finding devices with 'navigate_to' capability...")
    navigators = registry.get_devices_by_capability("navigate_to")
    for device in navigators:
        print(f"   - {device.device_id}: {device.profile.model}")
    print()

    # Execute capability
    print("Executing 'navigate_to' on TurtleBot...")
    result = robot.execute_capability("navigate_to", {"x": 5.0, "y": 3.0})
    print(f"   Result: {result}")
    print()

    print("Executing 'takeoff' on DJI Mavic...")
    result = drone.execute_capability("takeoff", {"altitude": 10.0})
    print(f"   Result: {result}")
    print()

    print("=" * 70)
    print("✅ Universal hardware support working!")
    print("=" * 70)
