"""
Standalone Robot API for Agent ROS Bridge

High-level Python API for controlling robots without external agent runtime.
This is the "standalone mode" of Agent ROS Bridge.

Usage:
    from agent_ros_bridge import Robot, NavigationGoal

    robot = Robot()
    robot.navigate_to(x=1.0, y=2.0)
    robot.pick_up("cup")
    robot.place_at("table")
"""

from dataclasses import dataclass
from typing import Any


@dataclass
class NavigationGoal:
    """Navigation goal specification."""

    x: float
    y: float
    theta: float = 0.0
    frame_id: str = "map"
    timeout_sec: float = 60.0


@dataclass
class ManipulationGoal:
    """Manipulation goal specification."""

    object_name: str
    action: str  # "pick", "place", "grasp", "release"
    location: str | None = None


@dataclass
class RobotCommandResult:
    """Result of a robot command."""

    success: bool
    execution_time: float
    error_message: str | None = None
    final_pose: dict[str, float] | None = None


class RobotController:
    """
    High-level robot control interface for standalone usage.

    This class provides a simplified API for controlling robots through
    Agent ROS Bridge without needing an external AI agent.

    Example:
        robot = Robot(ros_master="localhost:11311")

        # Navigate to location
        result = robot.navigate_to(NavigationGoal(x=1.0, y=2.0))
        if result.success:
            print(f"Arrived in {result.execution_time:.2f}s")

        # Pick up object
        robot.pick_up("cup")

        # Place at location
        robot.place_at("table")
    """

    def __init__(
        self,
        ros_master: str = "localhost:11311",
        robot_name: str = "robot_01",
        safety_enabled: bool = True,
    ):
        """
        Initialize robot connection.

        Args:
            ros_master: ROS master URI
            robot_name: Name of the robot
            safety_enabled: Whether to use safety validation
        """
        self.ros_master = ros_master
        self.robot_name = robot_name
        self.safety_enabled = safety_enabled

        # These would be initialized with ROS2
        self._nav_client = None
        self._manip_client = None
        self._state_sub = None

        # Connection status
        self._connected = False
        self._current_pose = None
        self._battery_level = 100.0

        # Try to connect
        self._connect()

    def _connect(self):
        """Establish connection to ROS."""
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav2_msgs.action import NavigateToPose
            from nav_msgs.msg import Odometry
            from rclpy.action import ActionClient
            from rclpy.node import Node

            # Initialize ROS2 if needed
            if not rclpy.ok():
                rclpy.init()

            # Create node
            self._node = Node(f"robot_api_{self.robot_name}")

            # Create action client for navigation
            self._nav_client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")

            # Create publisher for velocity commands
            self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)

            # Subscribe to odometry
            self._odom_sub = self._node.create_subscription(
                Odometry, "/odom", self._odom_callback, 10
            )

            self._connected = True
            print("✅ RobotController connected to ROS")

        except ImportError:
            # ROS2 not available, running in mock mode
            self._connected = False

    def _odom_callback(self, msg):
        """Callback for odometry messages."""
        self._current_pose = msg.pose.pose

    def is_connected(self) -> bool:
        """Check if connected to robot."""
        return self._connected

    def get_state(self) -> dict[str, Any]:
        """
        Get current robot state.

        Returns:
            Dictionary with pose, battery, status, etc.
        """
        return {
            "connected": self._connected,
            "robot_name": self.robot_name,
            "pose": self._current_pose,
            "battery_level": self._battery_level,
            "status": "idle" if self._connected else "disconnected",
        }

    def navigate_to(self, goal: NavigationGoal) -> RobotCommandResult:
        """
        Navigate to a position using Nav2.

        Args:
            goal: Navigation goal specification

        Returns:
            Command result with success status and timing
        """
        if not self._connected:
            return RobotCommandResult(
                success=False, execution_time=0.0, error_message="Not connected to robot"
            )

        try:
            import time

            from geometry_msgs.msg import PoseStamped
            from nav2_msgs.action import NavigateToPose

            start_time = time.time()

            # Wait for action server
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
                return RobotCommandResult(
                    success=False,
                    execution_time=0.0,
                    error_message="Nav2 action server not available",
                )

            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = goal.frame_id
            goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal.x
            goal_msg.pose.pose.position.y = goal.y
            goal_msg.pose.pose.orientation.w = 1.0

            # Send goal
            future = self._nav_client.send_goal_async(goal_msg)
            import rclpy

            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                return RobotCommandResult(
                    success=False,
                    execution_time=time.time() - start_time,
                    error_message="Goal rejected",
                )

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self._node, result_future, timeout_sec=goal.timeout_sec
            )

            result = result_future.result()
            execution_time = time.time() - start_time

            # Status 4 = SUCCEEDED
            success = result.status == 4 if result else False

            return RobotCommandResult(
                success=success,
                execution_time=execution_time,
                final_pose=(
                    {
                        "x": self._current_pose.position.x if self._current_pose else goal.x,
                        "y": self._current_pose.position.y if self._current_pose else goal.y,
                        "theta": goal.theta,
                    }
                    if success
                    else None
                ),
            )

        except Exception as e:
            return RobotCommandResult(success=False, execution_time=0.0, error_message=str(e))

    def pick_up(self, object_name: str) -> RobotCommandResult:
        """
        Pick up an object.

        Args:
            object_name: Name of object to pick up

        Returns:
            Command result
        """
        return self._execute_manipulation(ManipulationGoal(object_name=object_name, action="pick"))

    def place_at(self, location: str) -> RobotCommandResult:
        """
        Place held object at location.

        Args:
            location: Where to place the object

        Returns:
            Command result
        """
        return self._execute_manipulation(
            ManipulationGoal(object_name="held_object", action="place", location=location)
        )

    def _execute_manipulation(self, goal: ManipulationGoal) -> RobotCommandResult:
        """Execute manipulation action."""
        if not self._connected:
            return RobotCommandResult(
                success=False, execution_time=0.0, error_message="Not connected to robot"
            )

        # TODO: Implement using MoveIt2
        return RobotCommandResult(success=True, execution_time=1.0)

    def stop(self) -> bool:
        """Emergency stop."""
        if self._connected and self._cmd_vel_pub:
            from geometry_msgs.msg import Twist

            stop_cmd = Twist()
            self._cmd_vel_pub.publish(stop_cmd)
            return True
        return False

    def say(self, text: str) -> bool:
        """
        Make robot speak (if equipped with TTS).

        Args:
            text: Text to speak

        Returns:
            Success status
        """
        print(f"Robot says: {text}")
        return True

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self._disconnect()

    def _disconnect(self):
        """Clean up connection."""
        if self._connected:
            if self._nav_client:
                self._nav_client.destroy()
            if self._cmd_vel_pub:
                self._cmd_vel_pub.destroy()
            if self._odom_sub:
                self._odom_sub.destroy()
            if self._node:
                self._node.destroy_node()
            self._connected = False


# Export public API
__all__ = [
    "RobotController",
    "NavigationGoal",
    "ManipulationGoal",
    "RobotCommandResult",
]
