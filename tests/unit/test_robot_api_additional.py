"""Additional tests for robot API."""

from unittest.mock import Mock, patch, MagicMock, AsyncMock

import pytest

from agent_ros_bridge.robot_api import (
    RobotController,
    NavigationGoal,
    RobotCommandResult,
)


class TestRobotControllerConnect:
    """Test robot controller connection."""

    def test_connect_with_ros(self):
        """Test connection with ROS available."""
        with patch.dict("sys.modules", {
            "rclpy": MagicMock(),
            "geometry_msgs": MagicMock(),
            "nav2_msgs": MagicMock(),
            "nav_msgs": MagicMock(),
        }):
            import sys
            mock_rclpy = MagicMock()
            mock_rclpy.ok.return_value = True
            sys.modules["rclpy"] = mock_rclpy
            
            with patch.object(RobotController, "_connect"):
                robot = RobotController()
                robot._connected = True
                assert robot.is_connected() is True

    def test_connect_without_ros(self):
        """Test connection without ROS."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False
            assert robot.is_connected() is False


class TestRobotControllerNavigationAdditional:
    """Additional navigation tests."""

    def test_navigate_with_ros(self):
        """Test navigation with ROS."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            robot._nav_client = MagicMock()
            robot._node = MagicMock()
            
            # Mock the async behavior
            mock_future = MagicMock()
            mock_future.result.return_value = MagicMock(accepted=True)
            mock_future.result.return_value.get_result_async.return_value = MagicMock()
            robot._nav_client.wait_for_server = MagicMock(return_value=True)
            robot._nav_client.send_goal_async = MagicMock(return_value=mock_future)
            
            goal = NavigationGoal(x=1.0, y=2.0)
            result = robot.navigate_to(goal)
            
            # Should return a result
            assert isinstance(result, RobotCommandResult)


class TestRobotControllerManipulationAdditional:
    """Additional manipulation tests."""

    def test_pick_up_with_ros(self):
        """Test pick up with ROS."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            
            result = robot.pick_up("cup")
            
            assert isinstance(result, RobotCommandResult)

    def test_place_at_with_ros(self):
        """Test place at with ROS."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            
            result = robot.place_at("table")
            
            assert isinstance(result, RobotCommandResult)


class TestRobotControllerContextManager:
    """Test context manager."""

    def test_context_manager_enter(self):
        """Test context manager entry."""
        with patch.object(RobotController, "_connect"):
            with RobotController() as robot:
                assert isinstance(robot, RobotController)

    def test_context_manager_exit_cleanup(self):
        """Test context manager exit cleanup."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            robot._nav_client = MagicMock()
            robot._cmd_vel_pub = MagicMock()
            robot._odom_sub = MagicMock()
            robot._node = MagicMock()
            
            robot.__exit__(None, None, None)
            
            assert robot._connected is False
