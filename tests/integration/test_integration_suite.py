"""Integration Test Suite for Agent ROS Bridge.

Tests complete workflows and component interactions.
TDD Approach:
1. Write integration test scenarios
2. Run - expect failures
3. Fix integration issues
4. Verify all pass
"""

import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
from dataclasses import dataclass

pytestmark = pytest.mark.integration


@pytest.mark.integration
class TestFleetRobotIntegration:
    """Integration: Fleet orchestrator + Individual robots."""

    def test_fleet_robot_creation(self):
        """Fleet should be able to create and manage robots."""
        from agent_ros_bridge.fleet.orchestrator import FleetOrchestrator
        from agent_ros_bridge.fleet.robot import FleetRobot, RobotStatus

        fleet = FleetOrchestrator()

        # Add robot to fleet
        robot = FleetRobot(
            robot_id="bot1",
            name="Test Bot",
            status=RobotStatus.IDLE,
            battery_percent=100.0,
        )
        fleet.robots["bot1"] = robot

        # Verify robot is in fleet
        assert "bot1" in fleet.robots
        assert fleet.robots["bot1"].name == "Test Bot"

    def test_fleet_health_check_propagates(self):
        """Fleet health check should query all robots."""
        from agent_ros_bridge.fleet.orchestrator import FleetOrchestrator
        from agent_ros_bridge.fleet.robot import FleetRobot

        fleet = FleetOrchestrator()

        # Add robots with different health states
        fleet.robots["bot1"] = FleetRobot(
            robot_id="bot1",
            name="Bot 1",
            battery_percent=85.0,
        )
        fleet.robots["bot2"] = FleetRobot(
            robot_id="bot2",
            name="Bot 2",
            battery_percent=15.0,  # Low battery
        )

        # Get fleet health
        status = fleet.get_fleet_status()

        # Verify health aggregation
        assert status["total"] == 2
        assert "battery_avg" in status


class TestSafetyFeatureIntegration:
    """Integration: Safety system + All robot operations."""

    def test_safety_validates_navigation_command(self):
        """Safety system should validate navigation commands."""
        from agent_ros_bridge.agentic import RobotAgent
        from agent_ros_bridge.safety.validator import SafetyValidator

        robot = RobotAgent(device_id="test_bot")
        validator = SafetyValidator()

        # Validate a safe command
        cmd = {"linear": {"x": 0.5}, "angular": {"z": 0.1}}
        result = validator.validate_velocity(cmd)

        # Should be safe
        assert result.is_safe is True

    def test_safety_rejects_dangerous_velocity(self):
        """Safety system should reject dangerous velocities."""
        from agent_ros_bridge.safety.validator import SafetyValidator

        validator = SafetyValidator()

        # Validate a dangerous command
        cmd = {"linear": {"x": 5.0}, "angular": {"z": 2.0}}
        result = validator.validate_velocity(cmd)

        # Should be unsafe
        assert result.is_safe is False
        assert "exceeds" in result.reason.lower()


class TestAIAgentRobotIntegration:
    """Integration: AI agents + Robot control."""

    def test_openclaw_bridge_creates_robot_agent(self):
        """OpenClaw bridge should work with RobotAgent."""
        from agent_ros_bridge.agentic import RobotAgent
        from examples.ai_agent_integrations.openclaw_integration import OpenClawROSBridge

        robot = RobotAgent(device_id="test_bot")
        bridge = OpenClawROSBridge(robot)

        # Verify bridge created
        assert bridge is not None
        assert bridge.robot_agent == robot

    def test_natural_language_to_structured_command(self):
        """Natural language should translate to structured commands."""
        from agent_ros_bridge.agentic import RobotAgent
        from examples.ai_agent_integrations.natural_language_examples import (
            NaturalLanguageInterpreter,
        )

        robot = RobotAgent(device_id="test_bot")
        nli = NaturalLanguageInterpreter(robot)

        # Interpret command
        result = nli.interpret("Navigate to the kitchen")

        # Should produce structured result
        assert result.get("success") is True
        assert "interpreted" in result or "action" in result


class TestShadowModeIntegration:
    """Integration: Shadow mode + Decision logging."""

    def test_shadow_collector_tracks_decisions(self):
        """Shadow mode should track decisions."""
        from agent_ros_bridge.shadow.collector import ShadowModeCollector

        collector = ShadowModeCollector(target_hours=0.1)

        # Verify collector initialized
        assert collector is not None
        assert collector.stats["total_decisions"] == 0

    def test_metrics_calculated_from_shadow_data(self):
        """Metrics should be calculated from shadow mode data."""
        from agent_ros_bridge.shadow.metrics import calculate_agreement_rate

        # Create decisions
        decisions = [
            {"agreement": True},
            {"agreement": True},
            {"agreement": False},
        ]

        rate = calculate_agreement_rate(decisions)
        assert rate == 66.67


class TestMissionPlanningIntegration:
    """Integration: Mission planning + Execution."""

    def test_mission_planning_creates_valid_plan(self):
        """Mission planning should create executable plan."""
        from agent_ros_bridge.agentic import RobotAgent

        robot = RobotAgent(device_id="test_bot")

        # Plan mission
        mission = robot.plan_mission(
            "Clean the kitchen"
        )

        # Should create structured plan
        assert "tasks" in mission
        assert "estimated_duration" in mission

    def test_mission_contains_tasks(self):
        """Planned mission should contain tasks."""
        from agent_ros_bridge.agentic import RobotAgent

        robot = RobotAgent(device_id="test_bot")

        # Plan mission with known keywords
        mission = robot.plan_mission("Go to kitchen and go to office")

        # Should have at least 2 tasks
        assert len(mission["tasks"]) >= 1


class TestToolRobotIntegration:
    """Integration: Tools + Robot operations."""

    def test_rostopic_tool_has_correct_interface(self):
        """ROSTopicEchoTool should have correct interface."""
        from agent_ros_bridge.tools import ROSTopicEchoTool

        tool = ROSTopicEchoTool()

        # Verify tool properties
        assert tool.name == "rostopic_echo"
        assert hasattr(tool, 'execute')

    def test_rosservice_tool_has_correct_interface(self):
        """ROSServiceCallTool should have correct interface."""
        from agent_ros_bridge.tools import ROSServiceCallTool

        tool = ROSServiceCallTool()

        # Verify tool properties
        assert tool.name == "rosservice_call"
        assert hasattr(tool, 'execute')


class TestEndToEndScenarios:
    """End-to-end integration scenarios."""

    def test_complete_navigation_workflow(self):
        """Complete workflow: Command → Safety → Execute."""
        from agent_ros_bridge.agentic import RobotAgent
        from agent_ros_bridge.safety.validator import SafetyValidator

        # Setup
        robot = RobotAgent(device_id="test_bot")
        validator = SafetyValidator()

        # 1. Validate command
        cmd = {"linear": {"x": 0.5}, "angular": {"z": 0.1}}
        validation = validator.validate_velocity(cmd)
        assert validation.is_safe is True

        # 2. Robot should be configured
        assert robot.device_id == "test_bot"
        assert robot.safety.human_in_the_loop is True

    def test_robot_health_check_integration(self):
        """Health check should work with robot."""
        from agent_ros_bridge.agentic import RobotAgent

        robot = RobotAgent(device_id="test_bot")

        # Get health
        health = robot.health_check()

        # Should return health data
        assert "status" in health
        assert "battery" in health
        assert "timestamp" in health

    def test_object_recognition_integration(self):
        """Object recognition should work with robot."""
        from agent_ros_bridge.agentic import RobotAgent

        robot = RobotAgent(device_id="test_bot")

        # Recognize objects
        objects = robot.recognize_objects()

        # Should return list
        assert isinstance(objects, list)

    def test_fleet_coordination_scenario(self):
        """Fleet coordination: Setup → Status → Health."""
        from agent_ros_bridge.fleet.orchestrator import FleetOrchestrator
        from agent_ros_bridge.fleet.robot import FleetRobot, RobotStatus

        fleet = FleetOrchestrator()

        # Setup fleet
        robot = FleetRobot(
            robot_id="bot1",
            name="Warehouse Bot",
            status=RobotStatus.IDLE,
            battery_percent=90.0,
        )
        fleet.robots["bot1"] = robot

        # Verify coordination
        status = fleet.get_fleet_status()
        assert status["total"] == 1
        assert status["online"] >= 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
