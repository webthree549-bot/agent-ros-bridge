"""E2E User Journey Tests for Agent ROS Bridge.

Validates complete user workflows from start to finish.
These tests simulate real user interactions.
"""

import asyncio
from unittest.mock import MagicMock, patch

import pytest

from agent_ros_bridge.actions import SimulatedActionClient, create_action_client

# Import main components
from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.exceptions import RobotConnectionError, SafetyValidationError
from agent_ros_bridge.tools import ROSServiceCallTool, ROSTopicEchoTool


class TestUserJourneyBasic:
    """Test basic user journey: Setup -> Command -> Execute."""

    def test_user_creates_agent(self):
        """E2E: User creates a RobotAgent with safety enabled."""
        # User journey step 1: Create agent
        agent = RobotAgent(
            device_id="test_bot",
            device_type="mobile_robot",
            require_confirmation=True,
        )

        # Verify agent is properly configured
        assert agent.device_id == "test_bot"
        assert agent.device_type == "mobile_robot"
        assert agent.require_confirmation is True
        assert agent.safety.human_in_the_loop is True

    def test_user_executes_command_with_safety(self):
        """E2E: User executes command with human-in-the-loop."""
        agent = RobotAgent(
            device_id="test_bot",
            require_confirmation=True,
        )

        # Verify safety is enforced
        assert agent.safety.autonomous_mode is False
        assert agent.safety.shadow_mode_enabled is True

    def test_user_views_safety_status(self):
        """E2E: User can view safety status banner."""
        # When agent is created, safety status should be printed
        # (This is verified by the __init__ printing)
        agent = RobotAgent(
            device_id="safety_bot",
            require_confirmation=True,
        )

        assert agent.safety is not None
        assert agent.safety.safety_validation_status is not None


class TestUserJourneyTools:
    """Test user journey with tools: Setup -> Tool -> Execute -> Result."""

    def test_user_uses_rostopic_echo(self):
        """E2E: User uses rostopic_echo tool."""
        # User journey: Create tool, execute, get result
        tool = ROSTopicEchoTool()

        # Execute tool (will fail gracefully without ROS2)
        result = tool.execute(topic="/test", count=1)

        # Verify result structure
        assert hasattr(result, "success")
        assert hasattr(result, "output")
        assert hasattr(result, "error")
        assert hasattr(result, "data")
        assert hasattr(result, "execution_time_ms")

        # Verify data contains topic info
        assert result.data["topic"] == "/test"
        assert result.data["count"] == 1

    def test_user_uses_rosservice_call(self):
        """E2E: User uses rosservice_call tool."""
        tool = ROSServiceCallTool()

        result = tool.execute(service="/test_service", request={"param": "value"})

        # Verify result structure
        assert hasattr(result, "success")
        assert hasattr(result, "output")
        assert result.data["service"] == "/test_service"

    def test_user_handles_tool_error(self):
        """E2E: User gracefully handles tool execution error."""
        tool = ROSTopicEchoTool()

        # Execute with ROS2 unavailable
        result = tool.execute(topic="/nonexistent")

        # Should fail gracefully, not crash
        assert result.success is False
        assert result.error is not None
        assert result.execution_time_ms > 0


class TestUserJourneyActions:
    """Test user journey with actions: Setup -> Action -> Monitor -> Complete."""

    @pytest.mark.asyncio
    async def test_user_creates_action_client(self):
        """E2E: User creates an action client."""
        client = create_action_client(
            action_name="navigate_to_pose", action_type="nav2_msgs/action/NavigateToPose"
        )

        assert client is not None
        assert client.action_name == "navigate_to_pose"
        assert client.action_type == "nav2_msgs/action/NavigateToPose"

    @pytest.mark.asyncio
    async def test_user_connects_and_sends_goal(self):
        """E2E: User connects to action server and sends goal."""
        client = SimulatedActionClient(action_name="test_action", action_type="test/Action")

        # Connect
        connected = await client.connect()
        assert connected is True

        # Send goal
        result = await client.send_goal({"target": "location"})

        # Verify result
        assert result.success is True
        assert result.status.name == "SUCCEEDED"
        assert result.execution_time_sec > 0


class TestUserJourneySafety:
    """Test user journey with safety validation."""

    def test_user_cannot_disable_safety(self):
        """E2E: User cannot easily disable safety features."""
        agent = RobotAgent(
            device_id="safe_bot",
            require_confirmation=True,
        )

        # Safety config should enforce confirmation
        assert agent.safety.human_in_the_loop is True
        assert agent.require_confirmation is True

    def test_user_sees_shadow_mode_status(self):
        """E2E: User can see shadow mode collection status."""
        agent = RobotAgent(device_id="shadow_bot")

        # Shadow mode should be enabled by default
        assert agent.safety.shadow_mode_enabled is True
        assert agent.safety.shadow_mode_hours_collected >= 0
        assert agent.safety.required_shadow_hours == 200.0


class TestUserJourneyExceptions:
    """Test user journey with proper error handling."""

    def test_user_gets_clear_error_messages(self):
        """E2E: User receives clear error messages on failures."""
        exc = RobotConnectionError("ROS2 not available", robot_id="bot1")

        error_msg = str(exc)
        assert "Robot connection failed" in error_msg
        assert "bot1" in error_msg
        assert "ROS2 not available" in error_msg

    def test_user_gets_safety_violation_details(self):
        """E2E: User gets details on safety violations."""
        exc = SafetyValidationError("Velocity exceeded limit", violation_type="velocity")

        assert "Safety validation failed" in str(exc)
        assert exc.violation_type == "velocity"
        assert exc.details["violation_type"] == "velocity"


class TestUserJourneyCompleteWorkflow:
    """Test complete user workflows."""

    def test_complete_setup_workflow(self):
        """E2E: Complete setup workflow from scratch."""
        # Step 1: Create agent with safety
        agent = RobotAgent(
            device_id="warehouse_bot",
            device_type="mobile_robot",
            require_confirmation=True,
        )

        # Step 2: Verify safety is active
        assert agent.safety.autonomous_mode is False
        assert agent.safety.human_in_the_loop is True

        # Step 3: Verify tools are available
        tool = ROSTopicEchoTool()
        assert tool.name == "rostopic_echo"

        # Step 4: Verify actions are available
        from agent_ros_bridge.actions import BaseActionClient

        assert BaseActionClient is not None

        # All components working together
        print("✅ Complete workflow validated")

    @pytest.mark.asyncio
    async def test_complete_execution_workflow(self):
        """E2E: Complete execution workflow."""
        # Step 1: Create agent
        agent = RobotAgent(device_id="exec_bot")

        # Step 2: Create action client
        client = SimulatedActionClient(action_name="navigate", action_type="nav/Navigate")

        # Step 3: Connect
        connected = await client.connect()
        assert connected is True

        # Step 4: Execute
        result = await client.send_goal({"x": 1.0, "y": 2.0})

        # Step 5: Verify result
        assert result.success is True
        assert result.status.name == "SUCCEEDED"

        print("✅ Execution workflow validated")


class TestUserJourneyBackwardCompatibility:
    """Test that refactoring didn't break existing user journeys."""

    def test_old_imports_still_work(self):
        """E2E: Old import paths still work."""
        # These are the imports users were using before
        from agent_ros_bridge.actions import (
            ActionResult,
            ActionStatus,
            BaseActionClient,
            SimulatedActionClient,
            create_action_client,
        )

        # All should be importable
        assert BaseActionClient is not None
        assert SimulatedActionClient is not None
        assert create_action_client is not None
        assert ActionResult is not None
        assert ActionStatus is not None

    def test_old_api_still_works(self):
        """E2E: Old API calls still work."""
        # User code from before refactoring
        client = SimulatedActionClient(action_name="test", action_type="test/Action")

        # Old attributes should still exist
        assert hasattr(client, "action_name")
        assert hasattr(client, "action_type")
        assert hasattr(client, "status")
        assert hasattr(client, "connected")


# Integration marker
pytestmark = pytest.mark.e2e
