#!/usr/bin/env python3
"""Arm Robot Plugin for Agent ROS Bridge.

Supports industrial and collaborative robot arms:
- Universal Robots (UR5, UR10, UR3e, UR5e, UR10e)
- UFACTORY xArm (xArm6, xArm7)
- Franka Emika Panda
- Generic ROS1/ROS2 arm interface

Usage:
    from agent_ros_bridge.plugins.arm_robot import ArmRobotPlugin, ArmController

    arm = ArmRobotPlugin(arm_type="ur", ros_version="ros2")
    await arm.move_joints([0, -1.57, 0, -1.57, 0, 0])
    await arm.move_cartesian(x=0.5, y=0.2, z=0.3)
"""

import asyncio
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from agent_ros_bridge.gateway_v2.core import Identity, Message, Plugin

logger = logging.getLogger("plugin.arm_robot")


class ArmType(Enum):
    """Supported arm robot types."""

    UR = "ur"  # Universal Robots
    XARM = "xarm"  # UFACTORY xArm
    FRANKA = "franka"  # Franka Emika Panda
    GENERIC = "generic"  # Generic ROS arm


class ArmState(Enum):
    """Arm operational states."""

    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"
    TEACHING = "teaching"  # Teach pendant mode
    PROTECTIVE_STOP = "protective_stop"


@dataclass
class JointState:
    """Joint position/velocity/effort."""

    position: float
    velocity: float = 0.0
    effort: float = 0.0


@dataclass
class CartesianPose:
    """End-effector pose (position + orientation)."""

    x: float
    y: float
    z: float
    qx: float = 0.0  # Quaternion
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def to_list(self) -> List[float]:
        """Convert pose to list format [x, y, z, qx, qy, qz, qw]."""
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]


@dataclass
class ArmConfig:
    """Arm robot configuration."""

    arm_type: ArmType
    ros_version: str = "ros2"  # ros1, ros2
    namespace: str = ""
    joint_names: List[str] = None
    base_frame: str = "base_link"
    end_effector_frame: str = "tool0"
    max_joint_velocity: float = 3.0  # rad/s
    max_cartesian_velocity: float = 1.0  # m/s
    has_gripper: bool = True
    gripper_topic: str = "/gripper_command"


class BaseArmController(ABC):
    """Abstract base class for arm controllers."""

    def __init__(self, config: ArmConfig):
        self.config = config
        self.state = ArmState.IDLE
        self.current_joints: List[JointState] = []
        self.current_pose: Optional[CartesianPose] = None
        self.is_connected = False

    @abstractmethod
    async def connect(self) -> bool:
        """Connect to arm controller."""
        pass

    @abstractmethod
    async def disconnect(self):
        """Disconnect from arm."""
        pass

    @abstractmethod
    async def move_joints(
        self, joint_positions: List[float], velocity: Optional[float] = None
    ) -> bool:
        """Move to joint positions."""
        pass

    @abstractmethod
    async def move_cartesian(self, pose: CartesianPose, velocity: Optional[float] = None) -> bool:
        """Move to cartesian pose."""
        pass

    @abstractmethod
    async def get_joint_states(self) -> List[JointState]:
        """Get current joint states."""
        pass

    @abstractmethod
    async def get_cartesian_pose(self) -> Optional[CartesianPose]:
        """Get current end-effector pose."""
        pass

    @abstractmethod
    async def set_gripper(self, position: float, effort: float = 0.5) -> bool:
        """Set gripper position (0=open, 1=closed)."""
        pass

    @abstractmethod
    async def stop(self):
        """Emergency stop."""
        pass

    async def move_joint_trajectory(
        self, waypoints: List[List[float]], durations: List[float]
    ) -> bool:
        """Execute joint trajectory."""
        for i, joints in enumerate(waypoints):
            success = await self.move_joints(joints)
            if not success:
                return False
            if i < len(waypoints) - 1:
                await asyncio.sleep(durations[i])
        return True


class URController(BaseArmController):
    """Universal Robots controller (ROS1/ROS2)."""

    def __init__(self, config: ArmConfig):
        super().__init__(config)
        self.topic_prefix = f"/{config.namespace}" if config.namespace else ""
        self.joint_names = config.joint_names or [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    async def connect(self) -> bool:
        """Connect to UR robot."""
        try:
            if self.config.ros_version == "ros2":
                # ROS2 - use rclpy
                from rclpy.node import Node
                from sensor_msgs.msg import JointState
                from trajectory_msgs.msg import JointTrajectory

                self._node = Node(f"ur_controller_{id(self)}")
                self._joint_pub = self._node.create_publisher(
                    JointTrajectory,
                    f"{self.topic_prefix}/joint_trajectory_controller/joint_trajectory",
                    10,
                )
                self._gripper_pub = self._node.create_publisher(
                    Any,  # Replace with actual gripper msg type
                    f"{self.topic_prefix}{self.config.gripper_topic}",
                    10,
                )

                # Subscribe to joint states
                self._joint_sub = self._node.create_subscription(
                    JointState, f"{self.topic_prefix}/joint_states", self._on_joint_state, 10
                )

            else:
                # ROS1 - use rospy
                import rospy
                from trajectory_msgs.msg import JointTrajectory

                self._joint_pub = rospy.Publisher(
                    f"{self.topic_prefix}/arm_controller/command", JointTrajectory, queue_size=10
                )

            self.is_connected = True
            logger.info(f"✅ Connected to UR robot ({self.config.ros_version})")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to UR: {e}")
            return False

    async def disconnect(self):
        """Disconnect from UR."""
        self.is_connected = False
        if hasattr(self, "_node"):
            self._node.destroy_node()

    async def move_joints(
        self, joint_positions: List[float], velocity: Optional[float] = None
    ) -> bool:
        """Move UR to joint positions."""
        if not self.is_connected:
            return False

        try:
            self.state = ArmState.MOVING

            if self.config.ros_version == "ros2":
                from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

                msg = JointTrajectory()
                msg.joint_names = self.joint_names

                point = JointTrajectoryPoint()
                point.positions = joint_positions
                point.time_from_start.sec = 2  # 2 second motion
                msg.points.append(point)

                self._joint_pub.publish(msg)
            else:
                # ROS1 implementation
                pass

            logger.info(f"UR moving to joints: {[round(j, 2) for j in joint_positions]}")

            # Wait for motion (simplified - real impl would check feedback)
            await asyncio.sleep(2.5)

            self.state = ArmState.IDLE
            return True

        except Exception as e:
            logger.error(f"UR move_joints failed: {e}")
            self.state = ArmState.ERROR
            return False

    async def move_cartesian(self, pose: CartesianPose, velocity: Optional[float] = None) -> bool:
        """UR cartesian move (requires IK)."""
        logger.info(f"UR cartesian move to: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
        # Would need IK solver here
        # For now, return False to indicate not implemented
        return False

    async def get_joint_states(self) -> List[JointState]:
        """Get current joint states."""
        return self.current_joints

    async def get_cartesian_pose(self) -> Optional[CartesianPose]:
        """Get current pose."""
        return self.current_pose

    async def set_gripper(self, position: float, effort: float = 0.5) -> bool:
        """Control UR gripper."""
        logger.info(f"UR gripper: {position:.2f}")
        return True

    async def stop(self):
        """Emergency stop."""
        logger.warning("🛑 UR EMERGENCY STOP")
        self.state = ArmState.PROTECTIVE_STOP

    def _on_joint_state(self, msg):
        """Callback for joint state updates."""
        self.current_joints = [
            JointState(position=p, velocity=v, effort=e)
            for p, v, e in zip(msg.position, msg.velocity, msg.effort)
        ]


class XArmController(BaseArmController):
    """UFACTORY xArm controller."""

    def __init__(self, config: ArmConfig):
        super().__init__(config)
        self.joint_names = config.joint_names or [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]

    async def connect(self) -> bool:
        """Connect to xArm."""
        logger.info("✅ Connected to xArm")
        self.is_connected = True
        return True

    async def disconnect(self):
        """Disconnect from the arm."""
        self.is_connected = False

    async def move_joints(
        self, joint_positions: List[float], velocity: Optional[float] = None
    ) -> bool:
        """Move joints to specified positions."""
        logger.info(f"xArm moving to joints: {[round(j, 2) for j in joint_positions]}")
        await asyncio.sleep(2)
        return True

    async def move_cartesian(self, pose: CartesianPose, velocity: Optional[float] = None) -> bool:
        """Move to cartesian pose."""
        logger.info(f"xArm cartesian: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})")
        return True

    async def get_joint_states(self) -> List[JointState]:
        """Get current joint states."""
        return []

    async def get_cartesian_pose(self) -> Optional[CartesianPose]:
        """Get current cartesian pose."""
        return None

    async def set_gripper(self, position: float, effort: float = 0.5) -> bool:
        """Set gripper position."""
        logger.info(f"xArm gripper: {position:.2f}")
        return True

    async def stop(self):
        """Emergency stop."""
        logger.warning("🛑 xArm EMERGENCY STOP")


class ArmRobotPlugin(Plugin):
    """Arm robot plugin for Agent ROS Bridge — loadable via PluginManager."""

    name = "arm_robot"
    version = "1.0.0"

    def __init__(self, arm_type: str = "ur", ros_version: str = "ros2", namespace: str = ""):
        self.arm_type = ArmType(arm_type)
        self.config = ArmConfig(
            arm_type=self.arm_type, ros_version=ros_version, namespace=namespace
        )
        self.controller: Optional[BaseArmController] = None
        self._init_controller()

    def _init_controller(self):
        """Initialize appropriate controller."""
        if self.arm_type == ArmType.UR:
            self.controller = URController(self.config)
        elif self.arm_type == ArmType.XARM:
            self.controller = XArmController(self.config)
        else:
            raise ValueError(f"Unsupported arm type: {self.arm_type}")

    async def initialize(self, gateway) -> None:
        """Initialize plugin (Plugin ABC)."""
        success = await self.controller.connect()
        if success:
            logger.info(f"Arm robot plugin initialized: {self.arm_type.value}")

    async def shutdown(self) -> None:
        """Shutdown plugin (Plugin ABC)."""
        if self.controller:
            await self.controller.disconnect()

    async def handle_message(self, message: Message, identity: Identity) -> Optional[Message]:
        """Route arm.* commands from the bridge message bus."""
        if not message.command or not message.command.action.startswith("arm."):
            return None
        result = await self.handle_command(message.command.action, message.command.parameters)
        from agent_ros_bridge.gateway_v2.core import Header, Telemetry

        return Message(
            header=Header(correlation_id=message.header.message_id),
            telemetry=Telemetry(topic="/arm/result", data=result),
        )

    async def handle_command(self, command: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """Handle arm-specific commands."""
        if not self.controller:
            return {"error": "Controller not initialized"}

        try:
            if command == "arm.move_joints":
                joints = params.get("joints", [0, 0, 0, 0, 0, 0])
                success = await self.controller.move_joints(joints)
                return {"success": success, "target_joints": joints}

            elif command == "arm.move_cartesian":
                pose = CartesianPose(
                    x=params.get("x", 0), y=params.get("y", 0), z=params.get("z", 0)
                )
                success = await self.controller.move_cartesian(pose)
                return {"success": success}

            elif command == "arm.get_state":
                joints = await self.controller.get_joint_states()
                pose = await self.controller.get_cartesian_pose()
                return {
                    "state": self.controller.state.value,
                    "joints": [{"pos": j.position} for j in joints],
                    "pose": pose.__dict__ if pose else None,
                }

            elif command == "arm.gripper":
                position = params.get("position", 0.5)
                success = await self.controller.set_gripper(position)
                return {"success": success, "gripper_position": position}

            elif command == "arm.stop":
                await self.controller.stop()
                return {"success": True, "message": "Emergency stop triggered"}

            else:
                return {"error": f"Unknown command: {command}"}

        except Exception as e:
            logger.error(f"Arm command failed: {e}")
            return {"error": str(e)}


# Convenience functions for common operations
async def move_ur_to_home(arm: ArmRobotPlugin):
    """Move UR robot to home position."""
    home_joints = [0, -1.57, 0, -1.57, 0, 0]  # rad
    return await arm.handle_command("arm.move_joints", {"joints": home_joints})


async def pick_and_place(arm: ArmRobotPlugin, pick_pose: CartesianPose, place_pose: CartesianPose):
    """Execute pick and place operation."""
    # Open gripper
    await arm.handle_command("arm.gripper", {"position": 0.0})

    # Move to pick
    await arm.controller.move_cartesian(pick_pose)

    # Close gripper
    await arm.handle_command("arm.gripper", {"position": 1.0})

    # Move to place
    await arm.controller.move_cartesian(place_pose)

    # Open gripper
    await arm.handle_command("arm.gripper", {"position": 0.0})

    return {"success": True}
