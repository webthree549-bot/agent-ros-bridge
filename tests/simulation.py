"""Simulation Environment for Agent ROS Bridge Testing

Provides a mock ROS2 environment that simulates real robot behavior
without requiring actual ROS installation. This enables fast, reliable
TDD cycles.

Usage:
    # In your test file
    from tests.simulation import SimulatedRobot, SimulatedROS2Node

    robot = SimulatedRobot("test_robot")
    robot.add_topic("/cmd_vel", "geometry_msgs/Twist")
    robot.add_topic("/odom", "nav_msgs/Odometry")
"""

import asyncio
import contextlib
import logging
from collections.abc import Callable
from dataclasses import dataclass, field
from math import cos, sin
from typing import Any
from unittest import mock

logger = logging.getLogger(__name__)


@dataclass
class SimulatedMessage:
    """A simulated ROS message."""

    topic: str
    data: Any
    timestamp: float = field(default_factory=lambda: asyncio.get_event_loop().time())
    msg_type: str = "unknown"


@dataclass
class SimulatedTopic:
    """A simulated ROS topic."""

    name: str
    msg_type: str
    publishers: list[str] = field(default_factory=list)
    subscribers: list[str] = field(default_factory=list)
    last_message: SimulatedMessage | None = None
    message_history: list[SimulatedMessage] = field(default_factory=list)


@dataclass
class SimulatedService:
    """A simulated ROS service."""

    name: str
    service_type: str
    handler: Callable | None = None


class SimulatedROS2Node:
    """Simulates a ROS2 node for testing.

    This provides the same API as a real ROS2 node but with
    simulated behavior for fast, deterministic testing.
    """

    def __init__(self, node_name: str = "simulated_node"):
        self.node_name = node_name
        self.topics: dict[str, SimulatedTopic] = {}
        self.services: dict[str, SimulatedService] = {}
        self.subscribers: dict[str, Callable] = {}
        self.publishers: dict[str, Any] = {}
        self._message_queue: asyncio.Queue = asyncio.Queue()
        self._running = True

    def create_publisher(self, msg_class, topic: str, qos_profile=None):
        """Create a simulated publisher."""
        publisher = mock.MagicMock()
        publisher.topic = topic
        publisher.msg_class = msg_class

        def publish(msg):
            """Publish a message to the topic."""
            sim_msg = SimulatedMessage(
                topic=topic,
                data=self._msg_to_dict(msg),
                msg_type=getattr(msg_class, "__name__", str(msg_class)),
            )
            if topic in self.topics:
                self.topics[topic].last_message = sim_msg
                self.topics[topic].message_history.append(sim_msg)
            asyncio.create_task(self._message_queue.put(sim_msg))
            logger.debug(f"Published to {topic}: {sim_msg.data}")

        publisher.publish = publish
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_class, topic: str, callback, qos_profile=None):
        """Create a simulated subscription."""
        self.subscribers[topic] = callback
        if topic not in self.topics:
            self.topics[topic] = SimulatedTopic(
                name=topic, msg_type=getattr(msg_class, "__name__", str(msg_class))
            )
        self.topics[topic].subscribers.append(self.node_name)

        # Return a mock subscription that can be destroyed
        subscription = mock.MagicMock()
        subscription.topic = topic
        subscription.callback = callback
        return subscription

    def create_service(self, srv_type, name: str, callback):
        """Create a simulated service."""
        self.services[name] = SimulatedService(
            name=name, service_type=getattr(srv_type, "__name__", str(srv_type)), handler=callback
        )

    def get_topic_names_and_types(self) -> list[tuple]:
        """Get all topics."""
        return [(name, [topic.msg_type]) for name, topic in self.topics.items()]

    def get_service_names_and_types(self) -> list[tuple]:
        """Get all services."""
        return [(name, [service.service_type]) for name, service in self.services.items()]

    def get_node_names(self) -> list[str]:
        """Get all node names."""
        return [self.node_name]

    def destroy_subscription(self, subscription):
        """Destroy a subscription."""
        if subscription.topic in self.subscribers:
            del self.subscribers[subscription.topic]

    def destroy_publisher(self, publisher):
        """Destroy a publisher."""
        if publisher.topic in self.publishers:
            del self.publishers[publisher.topic]

    def simulate_message(self, topic: str, data: dict[str, Any], msg_type: str = "unknown"):
        """Simulate receiving a message on a topic."""
        SimulatedMessage(topic=topic, data=data, msg_type=msg_type)
        if topic in self.subscribers:
            # Create a mock message object
            mock_msg = self._dict_to_msg(data, msg_type)
            asyncio.create_task(self._call_callback(self.subscribers[topic], mock_msg))

    async def _call_callback(self, callback, msg):
        """Call a subscriber callback."""
        try:
            callback(msg)
        except Exception as e:
            logger.error(f"Error in subscriber callback: {e}")

    def add_topic(self, name: str, msg_type: str):
        """Add a topic to the simulation."""
        self.topics[name] = SimulatedTopic(name=name, msg_type=msg_type)

    def add_service(self, name: str, service_type: str, handler: Callable | None = None):
        """Add a service to the simulation."""
        self.services[name] = SimulatedService(
            name=name, service_type=service_type, handler=handler
        )

    def _msg_to_dict(self, msg) -> dict[str, Any]:
        """Convert a message to dictionary."""
        if hasattr(msg, "__dict__"):
            return {k: v for k, v in msg.__dict__.items() if not k.startswith("_")}
        elif isinstance(msg, dict):
            return msg
        else:
            return {"data": str(msg)}

    def _dict_to_msg(self, data: dict[str, Any], msg_type: str):
        """Create a mock message from a dictionary."""
        msg = mock.MagicMock()
        for key, value in data.items():
            setattr(msg, key, value)
        msg._type = msg_type
        return msg

    async def wait_for_message(
        self, topic: str, timeout: float = 1.0
    ) -> SimulatedMessage | None:
        """Wait for a message on a topic."""
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            try:
                msg = await asyncio.wait_for(self._message_queue.get(), timeout=0.1)
                if msg.topic == topic:
                    return msg
            except TimeoutError:
                continue
        return None

    def get_message_history(self, topic: str) -> list[SimulatedMessage]:
        """Get message history for a topic."""
        if topic in self.topics:
            return self.topics[topic].message_history
        return []


class SimulatedRobot:
    """Simulates a complete robot for testing.

    This simulates a robot with sensors, actuators, and internal state
    that responds to commands like a real robot would.
    """

    def __init__(self, robot_id: str, robot_type: str = "differential_drive"):
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.node = SimulatedROS2Node(f"{robot_id}_node")
        self.state = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "linear_velocity": 0.0,
            "angular_velocity": 0.0,
        }
        self._running = True
        self._update_task: asyncio.Task | None = None

    async def start(self):
        """Start the simulated robot."""
        # Set up standard topics based on robot type
        if self.robot_type == "differential_drive":
            self._setup_differential_drive()
        elif self.robot_type == "arm":
            self._setup_arm()

        # Start state update loop
        self._update_task = asyncio.create_task(self._state_update_loop())
        logger.info(f"Simulated robot {self.robot_id} started")

    async def stop(self):
        """Stop the simulated robot."""
        self._running = False
        if self._update_task:
            self._update_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._update_task

    def _setup_differential_drive(self):
        """Set up a differential drive robot simulation."""
        # Command topic
        self.node.add_topic("/cmd_vel", "geometry_msgs/Twist")
        # Odometry topic
        self.node.add_topic("/odom", "nav_msgs/Odometry")
        # Laser scan
        self.node.add_topic("/scan", "sensor_msgs/LaserScan")
        # Camera
        self.node.add_topic("/camera/image_raw", "sensor_msgs/Image")

        # Subscribe to velocity commands
        def on_cmd_vel(msg):
            """Handle velocity commands."""
            linear = getattr(msg, "linear", {})
            angular = getattr(msg, "angular", {})
            self.state["linear_velocity"] = getattr(linear, "x", 0.0)
            self.state["angular_velocity"] = getattr(angular, "z", 0.0)

        self.node.create_subscription(mock.MagicMock(), "/cmd_vel", on_cmd_vel, None)

    def _setup_arm(self):
        """Set up a robotic arm simulation."""
        self.node.add_topic("/joint_states", "sensor_msgs/JointState")
        self.node.add_topic("/arm_controller/command", "trajectory_msgs/JointTrajectory")

    async def _state_update_loop(self):
        """Update robot state based on velocities."""
        dt = 0.1  # 10Hz update rate
        while self._running:
            # Update position based on velocity
            v = self.state["linear_velocity"]
            omega = self.state["angular_velocity"]
            theta = self.state["theta"]

            self.state["x"] += v * cos(theta) * dt
            self.state["y"] += v * sin(theta) * dt
            self.state["theta"] += omega * dt

            # Publish odometry
            odom_msg = {
                "pose": {
                    "position": {"x": self.state["x"], "y": self.state["y"], "z": 0.0},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": sin(self.state["theta"] / 2),
                        "w": cos(self.state["theta"] / 2),
                    },
                },
                "twist": {
                    "linear": {"x": v, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": omega},
                },
            }
            self.node.simulate_message("/odom", odom_msg, "nav_msgs/Odometry")

            await asyncio.sleep(dt)


class SimulatedROSEnvironment:
    """Simulates a complete ROS environment with multiple robots.

    This is the top-level simulation manager that coordinates
    multiple simulated robots and provides discovery.
    """

    def __init__(self):
        self.robots: dict[str, SimulatedRobot] = {}
        self.global_topics: dict[str, SimulatedTopic] = {}
        self._running = False

    async def add_robot(
        self, robot_id: str, robot_type: str = "differential_drive"
    ) -> SimulatedRobot:
        """Add a robot to the simulation."""
        robot = SimulatedRobot(robot_id, robot_type)
        await robot.start()
        self.robots[robot_id] = robot
        return robot

    async def remove_robot(self, robot_id: str):
        """Remove a robot from the simulation."""
        if robot_id in self.robots:
            await self.robots[robot_id].stop()
            del self.robots[robot_id]

    def get_all_topics(self) -> list[tuple]:
        """Get all topics from all robots."""
        topics = []
        for robot in self.robots.values():
            topics.extend(robot.node.get_topic_names_and_types())
        return topics

    def get_all_services(self) -> list[tuple]:
        """Get all services from all robots."""
        services = []
        for robot in self.robots.values():
            services.extend(robot.node.get_service_names_and_types())
        return services

    async def simulate_scenario(self, scenario_name: str):
        """Run a predefined test scenario."""
        scenarios = {
            "simple_navigation": self._scenario_simple_navigation,
            "multi_robot": self._scenario_multi_robot,
            "obstacle_avoidance": self._scenario_obstacle_avoidance,
        }

        if scenario_name in scenarios:
            await scenarios[scenario_name]()
        else:
            raise ValueError(f"Unknown scenario: {scenario_name}")

    async def _scenario_simple_navigation(self):
        """Simple navigation scenario with one robot."""
        robot = await self.add_robot("turtle1", "differential_drive")
        # Robot starts at origin and will respond to commands
        return robot

    async def _scenario_multi_robot(self):
        """Multi-robot scenario."""
        await self.add_robot("robot1", "differential_drive")
        await self.add_robot("robot2", "differential_drive")
        await self.add_robot("arm1", "arm")

    async def _scenario_obstacle_avoidance(self):
        """Obstacle avoidance scenario."""
        await self.add_robot("explorer", "differential_drive")
        # Add simulated obstacles that affect scan data

    async def cleanup(self):
        """Clean up all robots."""
        for robot in list(self.robots.values()):
            await robot.stop()
        self.robots.clear()


# Singleton instance for tests
_simulation_instance: SimulatedROSEnvironment | None = None


def get_simulation() -> SimulatedROSEnvironment:
    """Get or create the global simulation instance."""
    global _simulation_instance
    if _simulation_instance is None:
        _simulation_instance = SimulatedROSEnvironment()
    return _simulation_instance


def reset_simulation():
    """Reset the global simulation instance."""
    global _simulation_instance
    if _simulation_instance:
        asyncio.create_task(_simulation_instance.cleanup())
    _simulation_instance = SimulatedROSEnvironment()
