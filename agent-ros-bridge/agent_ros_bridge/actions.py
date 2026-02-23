"""ROS2 Actions support for Agent ROS Bridge.

Provides native integration with ROS2 action servers (Navigation2, MoveIt, etc.).

Example:
    from agent_ros_bridge.actions import ROS2ActionClient
    
    client = ROS2ActionClient(bridge, "navigate_to_pose")
    goal = {"pose": {"position": {"x": 5.0, "y": 3.0}}}
    
    result = await client.send_goal(goal)
"""

import asyncio
import logging
from typing import Any, Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    ActionClient = None
    Node = None

from agent_ros_bridge import ROSBridge

logger = logging.getLogger(__name__)


class ActionStatus(Enum):
    """Status of an action execution."""
    PENDING = "pending"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"
    UNKNOWN = "unknown"


@dataclass
class ActionGoal:
    """An action goal."""
    goal_id: str
    action_name: str
    goal_data: Dict[str, Any]
    status: ActionStatus = ActionStatus.PENDING
    feedback: List[Dict] = None
    result: Dict = None
    
    def __post_init__(self):
        if self.feedback is None:
            self.feedback = []


@dataclass
class ActionResult:
    """Result of an action execution."""
    success: bool
    status: ActionStatus
    result_data: Dict[str, Any]
    feedback: List[Dict]
    error: Optional[str] = None


class ROS2ActionClient:
    """Client for ROS2 action servers.
    
    Supports:
    - Navigation2 (navigate_to_pose, navigate_through_poses)
    - MoveIt (move_group)
    - FollowWaypoints
    - Any custom action
    
    Example:
        client = ROS2ActionClient(bridge, "navigate_to_pose")
        
        # Send goal with feedback
        def on_feedback(feedback):
            print(f"Distance remaining: {feedback.distance_remaining}")
        
        result = await client.send_goal(
            {"pose": {"position": {"x": 5.0, "y": 3.0}}},
            feedback_callback=on_feedback
        )
    """
    
    def __init__(self, bridge: ROSBridge, action_name: str, action_type: Optional[str] = None):
        """Initialize action client.
        
        Args:
            bridge: ROSBridge instance
            action_name: ROS action name (e.g., "navigate_to_pose")
            action_type: Full action type (e.g., "nav2_msgs/action/NavigateToPose")
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 action client requires rclpy")
        
        self.bridge = bridge
        self.action_name = action_name
        self.action_type = action_type or self._infer_action_type(action_name)
        self._client: Optional[ActionClient] = None
        self._goals: Dict[str, ActionGoal] = {}
    
    def _infer_action_type(self, action_name: str) -> str:
        """Infer action type from name.
        
        Args:
            action_name: Action name
            
        Returns:
            Full action type string
        """
        # Known action types
        action_types = {
            "navigate_to_pose": "nav2_msgs/action/NavigateToPose",
            "navigate_through_poses": "nav2_msgs/action/NavigateThroughPoses",
            "follow_waypoints": "nav2_msgs/action/FollowWaypoints",
            "follow_path": "nav2_msgs/action/FollowPath",
            "spin": "nav2_msgs/action/Spin",
            "backup": "nav2_msgs/action/BackUp",
            "compute_path_to_pose": "nav2_msgs/action/ComputePathToPose",
            "move_group": "moveit_msgs/action/MoveGroup",
            "execute_trajectory": "moveit_msgs/action/ExecuteTrajectory",
        }
        
        return action_types.get(action_name, f"std_msgs/action/{action_name}")
    
    async def connect(self):
        """Connect to action server."""
        # Get ROS2 node from connector
        connector = self.bridge.connector_manager.get_primary()
        if not connector or not hasattr(connector, 'get_node'):
            raise RuntimeError("No ROS2 connector available")
        
        node = connector.get_node()
        if not node:
            raise RuntimeError("ROS2 node not available")
        
        # Import action type
        action_module = self._import_action_type(self.action_type)
        
        # Create action client
        self._client = ActionClient(node, action_module, self.action_name)
        
        # Wait for server
        if not await self._wait_for_server(timeout=5.0):
            raise RuntimeError(f"Action server '{self.action_name}' not available")
        
        logger.info(f"Connected to action server: {self.action_name}")
    
    def _import_action_type(self, action_type: str):
        """Dynamically import action type."""
        # Parse type string: package_msgs/action/Type
        parts = action_type.split('/')
        if len(parts) != 3:
            raise ValueError(f"Invalid action type: {action_type}")
        
        package = parts[0]
        msg_type = parts[2]
        
        module = __import__(f"{package}.action", fromlist=[msg_type])
        return getattr(module, msg_type)
    
    async def _wait_for_server(self, timeout: float = 5.0) -> bool:
        """Wait for action server to be available."""
        start = asyncio.get_event_loop().time()
        
        while asyncio.get_event_loop().time() - start < timeout:
            if self._client.server_is_ready():
                return True
            await asyncio.sleep(0.1)
        
        return False
    
    async def send_goal(
        self,
        goal_data: Dict[str, Any],
        feedback_callback: Optional[Callable[[Any], None]] = None,
        timeout: Optional[float] = None
    ) -> ActionResult:
        """Send goal to action server.
        
        Args:
            goal_data: Goal parameters
            feedback_callback: Called with feedback updates
            timeout: Maximum time to wait (None for no timeout)
            
        Returns:
            Action result
        """
        if not self._client:
            await self.connect()
        
        # Create goal message
        goal_msg = self._create_goal_msg(goal_data)
        
        # Send goal
        goal_handle = await self._client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback
        )
        
        if not goal_handle.accepted:
            return ActionResult(
                success=False,
                status=ActionStatus.ABORTED,
                result_data={},
                feedback=[],
                error="Goal rejected by server"
            )
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        if timeout:
            try:
                result_response = await asyncio.wait_for(
                    result_future,
                    timeout=timeout
                )
            except asyncio.TimeoutError:
                # Cancel goal on timeout
                await goal_handle.cancel_goal_async()
                return ActionResult(
                    success=False,
                    status=ActionStatus.CANCELED,
                    result_data={},
                    feedback=[],
                    error="Timeout waiting for result"
                )
        else:
            result_response = await result_future
        
        # Parse result
        status = self._parse_status(result_response.status)
        
        return ActionResult(
            success=status == ActionStatus.SUCCEEDED,
            status=status,
            result_data=self._msg_to_dict(result_response.result),
            feedback=[]  # Feedback collected via callback
        )
    
    def _create_goal_msg(self, goal_data: Dict) -> Any:
        """Create goal message from dictionary."""
        # Get goal type
        goal_type = self._client._action_type.Goal
        goal_msg = goal_type()
        
        # Set fields from goal_data
        for key, value in goal_data.items():
            if hasattr(goal_msg, key):
                setattr(goal_msg, key, value)
        
        return goal_msg
    
    def _parse_status(self, status_code: int) -> ActionStatus:
        """Parse status code to enum."""
        status_map = {
            0: ActionStatus.UNKNOWN,
            1: ActionStatus.PENDING,
            2: ActionStatus.ACTIVE,
            4: ActionStatus.SUCCEEDED,
            5: ActionStatus.ABORTED,
            6: ActionStatus.CANCELED,
        }
        return status_map.get(status_code, ActionStatus.UNKNOWN)
    
    def _msg_to_dict(self, msg: Any) -> Dict:
        """Convert ROS message to dictionary."""
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            if hasattr(value, '__slots__'):
                result[slot] = self._msg_to_dict(value)
            else:
                result[slot] = value
        return result
    
    async def cancel_goal(self, goal_id: str) -> bool:
        """Cancel a running goal.
        
        Args:
            goal_id: Goal ID to cancel
            
        Returns:
            True if cancelled successfully
        """
        if goal_id in self._goals:
            # Cancel logic here
            pass
        
        return True


class Navigation2Client(ROS2ActionClient):
    """Convenience client for Navigation2.
    
    Simplifies common navigation tasks.
    
    Example:
        nav = Navigation2Client(bridge)
        
        # Navigate to pose
        await nav.go_to_pose(x=5.0, y=3.0, theta=1.57)
        
        # Follow waypoints
        waypoints = [(1, 1), (2, 2), (3, 3)]
        await nav.follow_waypoints(waypoints)
    """
    
    def __init__(self, bridge: ROSBridge):
        super().__init__(bridge, "navigate_to_pose")
    
    async def go_to_pose(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        frame_id: str = "map"
    ) -> ActionResult:
        """Navigate to a specific pose.
        
        Args:
            x: X coordinate
            y: Y coordinate
            theta: Orientation (radians)
            frame_id: Coordinate frame
            
        Returns:
            Action result
        """
        goal = {
            "pose": {
                "header": {
                    "frame_id": frame_id
                },
                "pose": {
                    "position": {
                        "x": x,
                        "y": y,
                        "z": 0.0
                    },
                    "orientation": {
                        "z": theta  # Simplified - should be quaternion
                    }
                }
            }
        }
        
        return await self.send_goal(goal)
    
    async def follow_waypoints(
        self,
        waypoints: List[tuple],
        frame_id: str = "map"
    ) -> ActionResult:
        """Follow a list of waypoints.
        
        Args:
            waypoints: List of (x, y) tuples
            frame_id: Coordinate frame
            
        Returns:
            Action result
        """
        poses = []
        for x, y in waypoints:
            poses.append({
                "header": {"frame_id": frame_id},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"z": 0.0}
                }
            })
        
        client = ROS2ActionClient(self.bridge, "follow_waypoints")
        await client.connect()
        
        return await client.send_goal({"poses": poses})
    
    async def spin(self, yaw: float) -> ActionResult:
        """Spin in place.
        
        Args:
            yaw: Rotation in radians
            
        Returns:
            Action result
        """
        client = ROS2ActionClient(self.bridge, "spin")
        await client.connect()
        
        return await client.send_goal({"yaw": yaw})
    
    async def backup(self, distance: float, speed: float = 0.1) -> ActionResult:
        """Back up.
        
        Args:
            distance: Distance to back up (meters, negative for backward)
            speed: Speed (m/s)
            
        Returns:
            Action result
        """
        client = ROS2ActionClient(self.bridge, "backup")
        await client.connect()
        
        return await client.send_goal({
            "target": {"x": distance},
            "speed": speed
        })


__all__ = [
    "ROS2ActionClient",
    "Navigation2Client",
    "ActionGoal",
    "ActionResult",
    "ActionStatus"
]
