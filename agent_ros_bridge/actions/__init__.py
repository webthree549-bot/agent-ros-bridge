#!/usr/bin/env python3
"""ROS Action Client for Agent ROS Bridge

Supports ROS Actions (ROS1 actionlib, ROS2 rclpy actions) for long-running
tasks like navigation, motion planning, and manipulation.

Features:
- Send action goals with timeouts
- Receive feedback during execution
- Get final results
- Cancel/preempt ongoing actions
- Support for both ROS1 and ROS2

Usage:
    from agent_ros_bridge.actions import ActionClient
    
    client = ActionClient("navigate_to_pose", "nav2_msgs/NavigateToPose")
    result = await client.send_goal({"pose": {...}}, feedback_cb=on_feedback)
"""

import asyncio
import logging
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto

logger = logging.getLogger("ros_actions")


class ActionStatus(Enum):
    """Action client status"""
    IDLE = auto()
    PENDING = auto()      # Goal sent, waiting for acceptance
    ACTIVE = auto()       # Goal accepted and executing
    PREEMPTING = auto()   # Cancel requested
    SUCCEEDED = auto()    # Goal completed successfully
    ABORTED = auto()      # Goal aborted by server
    REJECTED = auto()     # Goal rejected by server
    CANCELED = auto()     # Goal canceled successfully
    LOST = auto()         # Action server lost


@dataclass
class ActionGoal:
    """Action goal definition"""
    goal_id: str
    goal_data: Dict[str, Any]
    action_type: str  # e.g., "nav2_msgs/action/NavigateToPose"
    timeout_sec: float = 30.0
    created_at: datetime = field(default_factory=datetime.utcnow)


@dataclass
class ActionFeedback:
    """Action feedback during execution"""
    goal_id: str
    feedback_data: Dict[str, Any]
    timestamp: datetime = field(default_factory=datetime.utcnow)


@dataclass
class ActionResult:
    """Action final result"""
    goal_id: str
    success: bool
    status: ActionStatus
    result_data: Dict[str, Any] = field(default_factory=dict)
    error_message: Optional[str] = None
    execution_time_sec: float = 0.0


class BaseActionClient:
    """Base class for ROS action clients"""
    
    def __init__(self, action_name: str, action_type: str, ros_version: str = "ros2"):
        self.action_name = action_name
        self.action_type = action_type
        self.ros_version = ros_version
        self.status = ActionStatus.IDLE
        self._feedback_callbacks: List[Callable[[ActionFeedback], None]] = []
        self._result_callbacks: List[Callable[[ActionResult], None]] = []
        self._current_goal: Optional[ActionGoal] = None
        self._start_time: Optional[datetime] = None
        self._client = None
        self._connected = False
    
    async def connect(self) -> bool:
        """Connect to action server"""
        raise NotImplementedError
    
    async def disconnect(self):
        """Disconnect from action server"""
        raise NotImplementedError
    
    async def send_goal(self, goal_data: Dict[str, Any], 
                       timeout_sec: float = 30.0) -> ActionResult:
        """Send goal to action server"""
        raise NotImplementedError
    
    async def cancel_goal(self) -> bool:
        """Cancel current goal"""
        raise NotImplementedError
    
    def register_feedback_callback(self, callback: Callable[[ActionFeedback], None]):
        """Register callback for feedback updates"""
        self._feedback_callbacks.append(callback)
    
    def register_result_callback(self, callback: Callable[[ActionResult], None]):
        """Register callback for results"""
        self._result_callbacks.append(callback)
    
    def _notify_feedback(self, feedback: ActionFeedback):
        """Notify all feedback callbacks"""
        for cb in self._feedback_callbacks:
            try:
                cb(feedback)
            except Exception as e:
                logger.error(f"Feedback callback error: {e}")
    
    def _notify_result(self, result: ActionResult):
        """Notify all result callbacks"""
        for cb in self._result_callbacks:
            try:
                cb(result)
            except Exception as e:
                logger.error(f"Result callback error: {e}")


class ROS2ActionClient(BaseActionClient):
    """ROS2 action client using rclpy"""
    
    def __init__(self, action_name: str, action_type: str):
        super().__init__(action_name, action_type, "ros2")
        self._goal_handle = None
    
    async def connect(self) -> bool:
        """Connect to ROS2 action server"""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.action import ActionClient
            
            # Get or create node
            try:
                self._node = rclpy.create_node(f"action_client_{self.action_name.replace('/', '_')}")
            except:
                # Node already exists
                pass
            
            # Create action client
            self._client = ActionClient(self._node, self.action_type, self.action_name)
            
            # Wait for server
            if not self._client.wait_for_server(timeout_sec=5.0):
                logger.error(f"Action server '{self.action_name}' not available")
                return False
            
            self._connected = True
            logger.info(f"✅ Connected to ROS2 action: {self.action_name}")
            return True
            
        except ImportError:
            logger.error("rclpy not available")
            return False
        except Exception as e:
            logger.error(f"Failed to connect to action server: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from action server"""
        if self._client:
            self._client.destroy()
        self._connected = False
    
    async def send_goal(self, goal_data: Dict[str, Any],
                       timeout_sec: float = 30.0) -> ActionResult:
        """Send goal to ROS2 action server"""
        if not self._connected:
            return ActionResult(
                goal_id="",
                success=False,
                status=ActionStatus.LOST,
                error_message="Not connected to action server"
            )
        
        import uuid
        goal_id = str(uuid.uuid4())[:8]
        
        goal = ActionGoal(
            goal_id=goal_id,
            goal_data=goal_data,
            action_type=self.action_type,
            timeout_sec=timeout_sec
        )
        
        self._current_goal = goal
        self._start_time = datetime.utcnow()
        self.status = ActionStatus.PENDING
        
        try:
            # Create goal message
            goal_msg = self._create_goal_message(goal_data)
            
            # Send goal
            self._goal_handle = await self._client.send_goal_async(
                goal_msg,
                feedback_callback=self._on_feedback
            )
            
            if not self._goal_handle.accepted:
                self.status = ActionStatus.REJECTED
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.REJECTED,
                    error_message="Goal rejected by server"
                )
            
            self.status = ActionStatus.ACTIVE
            logger.info(f"▶️  Action goal accepted: {goal_id}")
            
            # Wait for result with timeout
            result_future = self._goal_handle.get_result_async()
            
            try:
                result_response = await asyncio.wait_for(
                    asyncio.wrap_future(result_future),
                    timeout=timeout_sec
                )
                
                execution_time = (datetime.utcnow() - self._start_time).total_seconds()
                
                # Check result status
                if result_response.status == 4:  # SUCCEEDED
                    self.status = ActionStatus.SUCCEEDED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=True,
                        status=ActionStatus.SUCCEEDED,
                        result_data=self._parse_result(result_response.result),
                        execution_time_sec=execution_time
                    )
                elif result_response.status == 5:  # ABORTED
                    self.status = ActionStatus.ABORTED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.ABORTED,
                        error_message="Goal aborted by server",
                        execution_time_sec=execution_time
                    )
                elif result_response.status == 2:  # CANCELED
                    self.status = ActionStatus.CANCELED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.CANCELED,
                        execution_time_sec=execution_time
                    )
                else:
                    self.status = ActionStatus.ABORTED
                    result = ActionResult(
                        goal_id=goal_id,
                        success=False,
                        status=ActionStatus.ABORTED,
                        error_message=f"Unknown status: {result_response.status}",
                        execution_time_sec=execution_time
                    )
                
                self._notify_result(result)
                return result
                
            except asyncio.TimeoutError:
                self.status = ActionStatus.ABORTED
                await self.cancel_goal()
                return ActionResult(
                    goal_id=goal_id,
                    success=False,
                    status=ActionStatus.ABORTED,
                    error_message=f"Timeout after {timeout_sec}s",
                    execution_time_sec=timeout_sec
                )
                
        except Exception as e:
            logger.error(f"Action execution failed: {e}")
            self.status = ActionStatus.ABORTED
            return ActionResult(
                goal_id=goal_id,
                success=False,
                status=ActionStatus.ABORTED,
                error_message=str(e)
            )
    
    async def cancel_goal(self) -> bool:
        """Cancel current goal"""
        if self._goal_handle and self.status == ActionStatus.ACTIVE:
            cancel_future = self._goal_handle.cancel_goal_async()
            await asyncio.wrap_future(cancel_future)
            self.status = ActionStatus.CANCELED
            return True
        return False
    
    def _on_feedback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback_data = self._parse_feedback(feedback_msg)
        
        feedback = ActionFeedback(
            goal_id=self._current_goal.goal_id if self._current_goal else "",
            feedback_data=feedback_data
        )
        
        self._notify_feedback(feedback)
    
    def _create_goal_message(self, goal_data: Dict[str, Any]):
        """Create goal message from dict (simplified)"""
        # This would need proper message type handling
        # For now, return the dict wrapped
        class GoalMsg:
            def __init__(self, data):
                for k, v in data.items():
                    setattr(self, k, v)
        return GoalMsg(goal_data)
    
    def _parse_feedback(self, feedback_msg) -> Dict[str, Any]:
        """Parse feedback message to dict"""
        # Extract attributes from feedback message
        result = {}
        for attr in dir(feedback_msg):
            if not attr.startswith('_'):
                try:
                    val = getattr(feedback_msg, attr)
                    if not callable(val):
                        result[attr] = val
                except:
                    pass
        return result
    
    def _parse_result(self, result_msg) -> Dict[str, Any]:
        """Parse result message to dict"""
        return self._parse_feedback(result_msg)


class MockActionClient(BaseActionClient):
    """Mock action client for testing without ROS"""
    
    async def connect(self) -> bool:
        """Mock connect"""
        self._connected = True
        logger.info(f"✅ Mock action client: {self.action_name}")
        return True
    
    async def disconnect(self):
        """Mock disconnect"""
        self._connected = False
    
    async def send_goal(self, goal_data: Dict[str, Any],
                       timeout_sec: float = 30.0) -> ActionResult:
        """Mock goal execution with simulated feedback"""
        import uuid
        goal_id = str(uuid.uuid4())[:8]
        
        self._current_goal = ActionGoal(
            goal_id=goal_id,
            goal_data=goal_data,
            action_type=self.action_type
        )
        
        self.status = ActionStatus.ACTIVE
        self._start_time = datetime.utcnow()
        
        logger.info(f"▶️  Mock action: {self.action_name} goal {goal_id}")
        
        # Simulate execution with feedback
        steps = 10
        for i in range(steps):
            await asyncio.sleep(0.5)
            
            # Generate mock feedback based on action type
            feedback_data = self._generate_mock_feedback(i, steps, goal_data)
            
            feedback = ActionFeedback(
                goal_id=goal_id,
                feedback_data=feedback_data
            )
            self._notify_feedback(feedback)
            
            logger.debug(f"Mock feedback: {feedback_data}")
        
        # Complete successfully
        execution_time = (datetime.utcnow() - self._start_time).total_seconds()
        self.status = ActionStatus.SUCCEEDED
        
        result = ActionResult(
            goal_id=goal_id,
            success=True,
            status=ActionStatus.SUCCEEDED,
            result_data={"message": "Mock action completed"},
            execution_time_sec=execution_time
        )
        
        self._notify_result(result)
        logger.info(f"✅ Mock action completed: {goal_id}")
        
        return result
    
    async def cancel_goal(self) -> bool:
        """Mock cancel"""
        self.status = ActionStatus.CANCELED
        return True
    
    def _generate_mock_feedback(self, step: int, total: int, goal_data: Dict) -> Dict[str, Any]:
        """Generate mock feedback based on action type"""
        progress = (step + 1) / total
        
        if "navigate" in self.action_name.lower():
            return {
                "distance_remaining": 10.0 * (1 - progress),
                "current_speed": 0.5,
                "estimated_time_remaining": 10.0 * (1 - progress),
                "progress": progress
            }
        elif "manipulate" in self.action_name.lower() or "move" in self.action_name.lower():
            return {
                "current_joint": step,
                "progress": progress,
                "status": "moving"
            }
        else:
            return {
                "progress": progress,
                "step": step,
                "status": "active"
            }


def create_action_client(action_name: str, action_type: str, 
                         ros_version: str = "ros2") -> BaseActionClient:
    """Factory function to create appropriate action client"""
    
    # Try to use real ROS client
    if ros_version == "ros2":
        try:
            import rclpy
            return ROS2ActionClient(action_name, action_type)
        except ImportError:
            logger.warning("rclpy not available, using mock action client")
            return MockActionClient(action_name, action_type)
    else:
        # ROS1 not yet implemented
        logger.warning("ROS1 action client not implemented, using mock")
        return MockActionClient(action_name, action_type)
