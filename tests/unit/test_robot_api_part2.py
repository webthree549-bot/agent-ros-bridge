"""Additional tests for robot API - Part 2."""

from unittest.mock import AsyncMock, MagicMock, Mock, PropertyMock, patch

import pytest

from agent_ros_bridge.robot_api import (
    ManipulationGoal,
    NavigationGoal,
    RobotCommandResult,
    RobotController,
)


class TestRobotControllerStateDetailed:
    """Detailed state tests."""

    def test_get_state_connected_with_pose(self):
        """Test getting state with pose."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            robot._current_pose = MagicMock()
            robot._current_pose.position.x = 1.5
            robot._current_pose.position.y = 2.5
            robot._battery_level = 75.5

            state = robot.get_state()

            assert state["connected"] is True
            assert state["battery_level"] == 75.5
            assert state["pose"] is not None
            assert state["status"] == "idle"

    def test_get_state_disconnected(self):
        """Test getting state when disconnected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False

            state = robot.get_state()

            assert state["connected"] is False
            assert state["status"] == "disconnected"
            assert state["pose"] is None


class TestRobotControllerNavigationDetailed:
    """Detailed navigation tests."""

    def test_navigate_goal_rejected(self):
        """Test navigation when goal is rejected."""
        # Create proper mock for nav2_msgs.action with NavigateToPose class
        mock_navigate_to_pose = MagicMock()
        mock_navigate_to_pose.Goal = MagicMock
        mock_nav2_action = MagicMock()
        mock_nav2_action.NavigateToPose = mock_navigate_to_pose

        with patch.object(RobotController, "_connect"):
            with patch.dict(
                "sys.modules",
                {
                    "geometry_msgs.msg": MagicMock(),
                    "rclpy": MagicMock(),
                    "nav2_msgs.action": mock_nav2_action,
                },
            ):
                robot = RobotController()
                robot._connected = True
                # Use spec=True to prevent Mock from auto-creating attributes
                robot._nav_client = Mock(spec=True)
                robot._node = MagicMock()

                # Mock goal handle with rejected goal
                mock_future = MagicMock()
                mock_future.result.return_value = None  # Goal rejected
                robot._nav_client.wait_for_server = Mock(return_value=True)
                robot._nav_client.send_goal_async = Mock(return_value=mock_future)

                goal = NavigationGoal(x=5.0, y=5.0)
                result = robot.navigate_to(goal)

                assert result.success is False

    def test_navigate_server_not_available(self):
        """Test navigation when server not available."""
        # Create proper mock for nav2_msgs.action with NavigateToPose class
        mock_navigate_to_pose = MagicMock()
        mock_navigate_to_pose.Goal = MagicMock
        mock_nav2_action = MagicMock()
        mock_nav2_action.NavigateToPose = mock_navigate_to_pose

        with patch.object(RobotController, "_connect"):
            with patch.dict(
                "sys.modules",
                {
                    "geometry_msgs.msg": MagicMock(),
                    "rclpy": MagicMock(),
                    "nav2_msgs.action": mock_nav2_action,
                },
            ):
                robot = RobotController()
                robot._connected = True
                # Use spec=True to prevent Mock from auto-creating attributes
                robot._nav_client = Mock(spec=True)
                robot._node = MagicMock()

                # Server not available
                robot._nav_client.wait_for_server = Mock(return_value=False)

                goal = NavigationGoal(x=5.0, y=5.0)
                result = robot.navigate_to(goal)

                assert result.success is False


class TestRobotControllerManipulationDetailed:
    """Detailed manipulation tests."""

    def test_pick_up_connected(self):
        """Test pick up when connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True

            result = robot.pick_up("cup")

            # Returns success (implementation pending)
            assert isinstance(result, RobotCommandResult)

    def test_place_at_connected(self):
        """Test place at when connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True

            result = robot.place_at("table")

            assert isinstance(result, RobotCommandResult)

    def test_execute_manipulation_not_connected(self):
        """Test manipulation when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False

            result = robot._execute_manipulation(ManipulationGoal(object_name="cup", action="pick"))

            assert result.success is False
            assert "Not connected" in result.error_message


class TestRobotControllerStopDetailed:
    """Detailed stop tests."""

    def test_stop_connected_publishes(self):
        """Test stop publishes when connected."""
        with patch.object(RobotController, "_connect"):
            with patch.dict("sys.modules", {"geometry_msgs.msg": MagicMock()}):
                robot = RobotController()
                robot._connected = True
                robot._cmd_vel_pub = MagicMock()

                result = robot.stop()

                assert result is True
                robot._cmd_vel_pub.publish.assert_called_once()

    def test_stop_disconnected(self):
        """Test stop when disconnected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False

            result = robot.stop()

            assert result is False


class TestRobotControllerSay:
    """Test speech."""

    def test_say_connected(self):
        """Test say when connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True

            result = robot.say("Hello world")

            assert result is True

    def test_say_disconnected(self):
        """Test say when disconnected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False

            result = robot.say("Hello world")

            # Should still work (just prints)
            assert result is True


class TestRobotControllerDisconnect:
    """Test disconnection."""

    def test_disconnect_cleans_up(self):
        """Test disconnect cleans up resources."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = True
            robot._nav_client = MagicMock()
            robot._cmd_vel_pub = MagicMock()
            robot._odom_sub = MagicMock()
            robot._node = MagicMock()

            robot._disconnect()

            assert robot._connected is False
            robot._nav_client.destroy.assert_called_once()
            robot._cmd_vel_pub.destroy.assert_called_once()
            robot._odom_sub.destroy.assert_called_once()
            robot._node.destroy_node.assert_called_once()

    def test_disconnect_when_not_connected(self):
        """Test disconnect when not connected."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._connected = False

            # Should not raise
            robot._disconnect()

            assert robot._connected is False


class TestRobotControllerContextManager:
    """Test context manager."""

    def test_context_manager_cleanup_on_exit(self):
        """Test context manager cleans up on exit."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._disconnect = MagicMock()

            with robot:
                pass

            robot._disconnect.assert_called_once()

    def test_context_manager_cleanup_on_exception(self):
        """Test context manager cleans up on exception."""
        with patch.object(RobotController, "_connect"):
            robot = RobotController()
            robot._disconnect = MagicMock()

            try:
                with robot:
                    raise ValueError("Test error")
            except ValueError:
                pass

            robot._disconnect.assert_called_once()
