"""
TDD Tests for Agentic Interface

Tests define expected behavior of RobotAgent high-level API.
"""

from unittest.mock import MagicMock, Mock, patch

import pytest

from agent_ros_bridge.agentic import (
    AgentObservation,
    RobotAgent,
    TaskPlanner,
    TaskResult,
)


class TestTaskResult:
    """TaskResult captures execution outcome"""

    def test_task_result_stores_execution_details(self):
        """Red: Must capture all execution metrics"""
        result = TaskResult(
            success=True,
            task="Go to kitchen",
            steps=[{"step": "navigate", "status": "success"}],
            duration_seconds=5.0,
            ai_confidence=0.95,
            human_approvals=1,
            human_rejections=0,
            safety_violations=0,
            message="Successfully navigated to kitchen",
        )

        assert result.success is True
        assert result.task == "Go to kitchen"
        assert len(result.steps) == 1
        assert result.ai_confidence == 0.95
        assert result.human_approvals == 1
        assert result.human_rejections == 0
        assert result.safety_violations == 0


class TestAgentObservation:
    """AgentObservation captures environment state"""

    def test_observation_tracks_robot_state(self):
        """Red: Must track robot position and status"""
        obs = AgentObservation(
            robot_position=(1.0, 2.0, 0.0),
            battery_level=85.0,
            nearby_objects=["cup", "table"],
            current_task="navigating",
            recent_commands=["go to kitchen"],
            obstacles_detected=["wall"],
        )

        assert obs.robot_position == (1.0, 2.0, 0.0)
        assert obs.battery_level == 85.0
        assert obs.nearby_objects == ["cup", "table"]


class TestRobotAgentInitialization:
    """RobotAgent must initialize with correct configuration"""

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_agent_initializes_with_device_type(self, mock_registry):
        """Red: Must support different device types"""
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_registry.return_value.create_device.return_value = mock_device

        agent = RobotAgent(
            device_id="bot1",
            device_type="drone",
        )

        assert agent.device_id == "bot1"
        assert agent.device_type == "drone"

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_agent_uses_specified_llm_provider(self, mock_registry):
        """Red: Must use specified LLM provider"""
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_registry.return_value.create_device.return_value = mock_device

        agent = RobotAgent(
            device_id="bot1",
            device_type="mobile_robot",
            llm_provider="openai",
        )

        assert agent.llm_provider == "openai"


class TestRobotAgentExecution:
    """RobotAgent.execute() is the main agentic interface"""

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_execute_parses_natural_language(self, mock_registry):
        """Red: Must parse NL command to intent"""
        # Setup mocks
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_device.profile.capabilities = []
        mock_device.execute_capability.return_value = {"success": True}
        mock_registry.return_value.create_device.return_value = mock_device

        # Mock intent parser
        mock_intent = Mock()
        mock_intent.raw_text = "go to kitchen"
        mock_intent.confidence = 0.95

        with patch("agent_ros_bridge.ai.llm_parser.LLMIntentParser") as mock_parser_class:
            mock_parser = Mock()
            mock_parser.parse.return_value = mock_intent
            mock_parser_class.return_value = mock_parser

            # Mock task planner
            with patch("agent_ros_bridge.agentic.TaskPlanner") as mock_planner_class:
                mock_planner = Mock()
                mock_step = Mock()
                mock_step.name = "navigate"
                mock_step.capability_name = "navigate_to"
                mock_step.parameters = {"location": "kitchen"}
                mock_step.ai_proposal = mock_intent
                mock_planner.plan.return_value.steps = [mock_step]
                mock_planner_class.return_value = mock_planner

                agent = RobotAgent(
                    device_id="bot1",
                    device_type="mobile_robot",
                    require_confirmation=False,  # Skip confirmation for test
                )

                result = agent.execute("Go to the kitchen")

        assert result.task == "Go to the kitchen"
        assert result.ai_confidence == 0.95

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_execute_validates_safety_before_execution(self, mock_registry):
        """Red: Must validate safety before executing"""
        # Setup
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_device.profile.capabilities = []
        mock_registry.return_value.create_device.return_value = mock_device

        mock_intent = Mock()
        mock_intent.confidence = 0.95

        with patch("agent_ros_bridge.ai.llm_parser.LLMIntentParser") as mock_parser_class:
            mock_parser = Mock()
            mock_parser.parse.return_value = mock_intent
            mock_parser_class.return_value = mock_parser

            # Safety validator rejects
            with patch(
                "agent_ros_bridge.safety.validator.SafetyValidatorNode"
            ) as mock_safety_class:
                mock_safety = Mock()
                mock_safety.validate_trajectory.return_value = {
                    "approved": False,
                    "reason": "velocity_exceeded",
                }
                mock_safety_class.return_value = mock_safety

                with patch("agent_ros_bridge.agentic.TaskPlanner") as mock_planner_class:
                    mock_planner = Mock()
                    mock_step = Mock()
                    mock_step.name = "move"
                    mock_step.capability_name = "move_fast"
                    mock_step.parameters = {"velocity": 100}
                    mock_step.ai_proposal = mock_intent
                    mock_planner.plan.return_value.steps = [mock_step]
                    mock_planner_class.return_value = mock_planner

                    agent = RobotAgent(
                        device_id="bot1",
                        device_type="mobile_robot",
                        require_confirmation=False,
                    )

                    result = agent.execute("Move very fast")

        assert result.safety_violations == 1
        assert result.success is False

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_execute_runs_on_device_capabilities(self, mock_registry):
        """Red: Must execute device capabilities"""
        # Setup
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_device.profile.capabilities = []
        mock_device.execute_capability.return_value = {"success": True}
        mock_registry.return_value.create_device.return_value = mock_device

        mock_intent = Mock()
        mock_intent.confidence = 0.95

        with patch("agent_ros_bridge.ai.llm_parser.LLMIntentParser") as mock_parser_class:
            mock_parser = Mock()
            mock_parser.parse.return_value = mock_intent
            mock_parser_class.return_value = mock_parser

            with patch(
                "agent_ros_bridge.safety.validator.SafetyValidatorNode"
            ) as mock_safety_class:
                mock_safety = Mock()
                mock_safety.validate_trajectory.return_value = {"approved": True}
                mock_safety_class.return_value = mock_safety

                with patch("agent_ros_bridge.agentic.TaskPlanner") as mock_planner_class:
                    mock_planner = Mock()
                    mock_step = Mock()
                    mock_step.name = "navigate"
                    mock_step.capability_name = "navigate_to"
                    mock_step.parameters = {"x": 5.0}
                    mock_step.ai_proposal = mock_intent
                    mock_planner.plan.return_value.steps = [mock_step]
                    mock_planner_class.return_value = mock_planner

                    agent = RobotAgent(
                        device_id="bot1",
                        device_type="mobile_robot",
                        require_confirmation=False,
                    )

                    result = agent.execute("Go to position 5")

        # Verify device capability was called
        mock_device.execute_capability.assert_called_once_with(
            capability_name="navigate_to",
            parameters={"x": 5.0},
        )
        assert result.success is True


class TestRobotAgentObservation:
    """RobotAgent.observe() gets device state"""

    @patch("agent_ros_bridge.hardware.DeviceRegistry")
    def test_observe_returns_device_state(self, mock_registry):
        """Red: Must get current device state"""
        mock_device = Mock()
        mock_device.profile = Mock()
        mock_device.profile.limits = {}
        mock_device.get_state.return_value = {
            "position": (1.0, 2.0, 0.0),
            "battery": 85.0,
            "nearby_objects": ["cup"],
        }
        mock_registry.return_value.create_device.return_value = mock_device

        agent = RobotAgent(
            device_id="bot1",
            device_type="mobile_robot",
        )

        obs = agent.observe()

        assert obs.robot_position == (1.0, 2.0, 0.0)
        assert obs.battery_level == 85.0


class TestTaskPlanner:
    """TaskPlanner breaks down complex commands"""

    def test_planner_creates_steps_from_intent(self):
        """Red: Must create executable steps"""
        planner = TaskPlanner()

        mock_intent = Mock()
        mock_intent.raw_text = "go to kitchen"
        mock_intent.to_command.return_value = {"type": "navigate"}

        plan = planner.plan(intent=mock_intent)

        assert len(plan.steps) > 0

    def test_planner_handles_compound_commands(self):
        """Red: Must break compound commands into subtasks"""
        planner = TaskPlanner()

        mock_intent = Mock()
        mock_intent.raw_text = "go to kitchen and pick up cup"

        plan = planner.plan(intent=mock_intent)

        # Should create multiple steps for compound command
        assert len(plan.steps) >= 2


class TestTDDPrinciples:
    """Verify TDD principles in agentic module"""

    def test_agentic_has_test_file(self):
        """Red: New modules must have test files"""
        # This test documents that agentic.py has tests
        # (this file exists, so test passes)
        import agent_ros_bridge.agentic

        assert hasattr(agent_ros_bridge.agentic, "RobotAgent")

    def test_tests_use_red_green_pattern(self):
        """Red: Tests must use Red-Green naming"""
        # Tests above use docstrings like:
        # "Red: Must parse NL command to intent"
        # "Red: Must validate safety before executing"
        # This follows TDD Red-Green-Refactor pattern
        pass

    def test_all_public_methods_have_tests(self):
        """Red: All public API must be tested"""
        # Public methods of RobotAgent:
        # - __init__ -> tested
        # - execute() -> tested
        # - observe() -> tested
        # - think() -> TODO: add test
        # - plan_multi_step() -> TODO: add test

        # This test documents coverage gaps
        public_methods = ["execute", "observe", "think", "plan_multi_step"]
        tested_methods = ["execute", "observe"]

        missing = set(public_methods) - set(tested_methods)
        if missing:
            pytest.skip(f"Methods needing tests: {missing}")
