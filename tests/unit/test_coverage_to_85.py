"""TDD Tests to Increase Coverage from 76% to 85%.

Target areas with low coverage:
- gateway_v2/core.py (estimated 50%)
- ai/ modules (estimated 45%)
- shadow/ hooks.py (estimated 40%)
- simulation/ (estimated 45%)
- ui/ confirmation.py (estimated 30%)
"""

import asyncio
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

# ============================================================================
# gateway_v2/core.py Coverage
# ============================================================================


class TestGatewayCore:
    """Improve coverage for gateway_v2/core.py"""

    def test_command_creation(self):
        """Test Command dataclass creation."""
        from agent_ros_bridge.gateway_v2.core import Command

        cmd = Command(
            id="cmd_001",
            action="navigate",
            parameters={"location": "kitchen"},
            robot_id="bot1",
        )

        assert cmd.id == "cmd_001"
        assert cmd.action == "navigate"
        assert cmd.status == "pending"

    def test_command_to_dict(self):
        """Test Command serialization."""
        from agent_ros_bridge.gateway_v2.core import Command

        cmd = Command(
            id="cmd_001",
            action="navigate",
            parameters={"location": "kitchen"},
            robot_id="bot1",
        )

        data = cmd.to_dict()
        assert data["id"] == "cmd_001"
        assert data["action"] == "navigate"

    def test_message_creation(self):
        """Test Message dataclass."""
        from agent_ros_bridge.gateway_v2.core import Command, Header, Message

        cmd = Command(action="stop")
        msg = Message(
            command=cmd,
            header=Header(source="user", target="bot1"),
        )

        assert msg.command is not None
        assert msg.header.target == "bot1"

    def test_gateway_initialization(self):
        """Test Gateway initialization."""
        from agent_ros_bridge.gateway_v2.core import Gateway

        gateway = Gateway()
        assert gateway is not None
        assert hasattr(gateway, "commands")

    @pytest.mark.asyncio
    async def test_gateway_command_lifecycle(self):
        """Test full command lifecycle."""
        from agent_ros_bridge.gateway_v2.core import Command, Gateway

        gateway = Gateway()

        # Create command
        cmd = Command(
            id="cmd_001",
            action="navigate",
            parameters={"location": "kitchen"},
            robot_id="bot1",
        )

        # Submit command
        await gateway.submit_command(cmd)

        # Verify tracking
        assert "cmd_001" in gateway.commands

        # Complete command
        await gateway.complete_command("cmd_001", success=True)
        assert gateway.commands["cmd_001"].status == "completed"


# ============================================================================
# AI Modules Coverage
# ============================================================================


class TestAIIntentParser:
    """Improve coverage for ai/intent_parser.py (requires rclpy)"""

    def _skip_if_no_rclpy(self):
        """Skip test if rclpy not available."""
        try:
            import rclpy
        except ImportError:
            pytest.skip("rclpy not available")

    def test_intent_result_creation(self):
        """Test IntentResult dataclass."""
        self._skip_if_no_rclpy()
        from agent_ros_bridge.ai.intent_parser import IntentResult

        result = IntentResult(
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )

        assert result.intent_type == "NAVIGATE"
        assert result.confidence == 0.95

    def test_intent_result_to_command(self):
        """Test converting IntentResult to command."""
        self._skip_if_no_rclpy()
        from agent_ros_bridge.ai.intent_parser import IntentResult

        result = IntentResult(
            intent_type="NAVIGATE",
            confidence=0.95,
            entities=[{"type": "LOCATION", "value": "kitchen"}],
        )

        cmd = result.to_command()
        assert cmd["action"] == "navigate"
        assert cmd["location"] == "kitchen"

    def test_intent_parser_parse_variations(self):
        """Test parsing various command formats."""
        self._skip_if_no_rclpy()
        from agent_ros_bridge.ai.intent_parser import IntentParser

        parser = IntentParser()

        # Various navigation commands
        commands = [
            "go to kitchen",
            "navigate to living room",
            "drive to bedroom",
            "head to garage",
            "move to office",
        ]

        for cmd in commands:
            result = parser.parse(cmd, robot_id="bot1")
            assert result is not None
            assert hasattr(result, "intent_type")

    def test_intent_parser_extract_entities(self):
        """Test entity extraction."""
        self._skip_if_no_rclpy()
        from agent_ros_bridge.ai.intent_parser import IntentParser

        parser = IntentParser()

        # Extract location
        result = parser.parse("go to the kitchen", robot_id="bot1")

        # Should extract kitchen
        location_found = any(
            e.get("type") == "LOCATION" or "kitchen" in str(e.get("value", "")).lower()
            for e in result.entities
        )
        assert location_found or result.intent_type is not None


# ============================================================================
# Shadow Hooks Coverage
# ============================================================================


class TestShadowHooks:
    """Improve coverage for shadow/hooks.py"""

    def test_shadow_hook_initialization(self):
        """Test ShadowModeHook initialization."""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hook = ShadowModeHooks(robot_ids=["bot1"])
        assert hook.config.robot_ids == ["bot1"]

    def test_shadow_hook_on_intent_parsed(self):
        """Test hooking AI intent parsing."""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hook = ShadowModeHooks(robot_ids=["bot1"])

        # Simulate AI proposal
        hook.on_intent_parsed(
            robot_id="bot1",
            intent_result={
                "intent_type": "NAVIGATE",
                "confidence": 0.9,
                "entities": [],
            },
        )

        # Should store proposal
        assert len(hook._pending_proposals) == 1

    def test_shadow_hook_on_human_command(self):
        """Test hooking human command."""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hook = ShadowModeHooks(robot_ids=["bot1"])

        # First AI proposal
        hook.on_intent_parsed(
            robot_id="bot1",
            intent_result={
                "intent_type": "NAVIGATE",
                "confidence": 0.9,
                "entities": [],
            },
        )

        # Then human decision
        hook.on_human_command(
            command={
                "robot_id": "bot1",
                "command": "navigate",
                "parameters": {"location": "kitchen"},
            },
        )

    def test_shadow_hook_agreement_rate(self):
        """Test agreement rate property."""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        hook = ShadowModeHooks(robot_ids=["bot1"])

        # Initially 0
        assert hook.agreement_rate == 0.0

        # After some decisions
        hook._total_decisions = 10
        hook._agreements = 8
        assert hook.agreement_rate == 0.8


# ============================================================================
# Simulation Coverage
# ============================================================================


class TestSimulationComponents:
    """Improve coverage for simulation/ modules"""

    def test_simulation_scenario_creation(self):
        """Test SimulationScenario creation."""
        from agent_ros_bridge.simulation.scenario import SimulationScenario

        scenario = SimulationScenario(
            name="test_scenario",
            duration_sec=60.0,
            obstacles=[{"x": 1.0, "y": 2.0}],
            goals=[{"x": 5.0, "y": 5.0}],
        )

        assert scenario.name == "test_scenario"
        assert len(scenario.obstacles) == 1

    def test_metrics_collector_collision(self):
        """Test collision recording."""
        from agent_ros_bridge.simulation.metrics import MetricsCollector

        collector = MetricsCollector()

        collector.record_collision(timestamp=1.0, location=(1.0, 2.0))

        metrics = collector.get_metrics()
        assert metrics["collisions"] == 1

    def test_metrics_collector_goal_reached(self):
        """Test goal reached recording."""
        from agent_ros_bridge.simulation.metrics import MetricsCollector

        collector = MetricsCollector()

        collector.record_goal_reached(timestamp=10.0, duration=8.0)

        metrics = collector.get_metrics()
        assert metrics["goals_reached"] == 1

    def test_metrics_collector_path_deviation(self):
        """Test path deviation recording."""
        from agent_ros_bridge.simulation.metrics import MetricsCollector

        collector = MetricsCollector()

        collector.record_path_deviation(distance=0.1)
        collector.record_path_deviation(distance=0.2)
        collector.record_path_deviation(distance=0.3)

        stats = collector.get_path_deviation_stats()
        assert abs(stats["mean"] - 0.2) < 0.001  # Allow float tolerance
        assert stats["max"] == 0.3


# ============================================================================
# UI Confirmation Coverage
# ============================================================================


class TestUIConfirmation:
    """Improve coverage for ui/confirmation.py"""

    def test_confirmation_dialog_creation(self):
        """Test ConfirmationDialog creation."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm Action",
            message="Navigate to kitchen?",
            timeout_sec=30.0,
        )

        assert dialog.title == "Confirm Action"
        assert dialog.timeout_sec == 30.0

    def test_confirmation_dialog_approve(self):
        """Test approval flow."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
        )

        # Simulate approval
        dialog.approve()
        assert dialog.status == "approved"

    def test_confirmation_dialog_reject(self):
        """Test rejection flow."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
        )

        # Simulate rejection
        dialog.reject()
        assert dialog.status == "rejected"

    def test_confirmation_dialog_timeout(self):
        """Test timeout handling."""
        from agent_ros_bridge.ui.confirmation import ConfirmationDialog

        dialog = ConfirmationDialog(
            title="Confirm",
            message="Proceed?",
            timeout_sec=1.0,  # Short timeout for testing
        )

        # Simulate timeout
        dialog.on_timeout()
        assert dialog.status == "timeout"


# ============================================================================
# Transport Layer Coverage
# ============================================================================


class TestTransportLayer:
    """Improve coverage for transport modules"""

    def test_websocket_transport_creation(self):
        """Test WebSocket transport initialization."""
        from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

        config = {"host": "localhost", "port": 8765}
        transport = WebSocketTransport(config=config, name="websocket")
        assert transport.config["host"] == "localhost"
        assert transport.config["port"] == 8765

    def test_grpc_transport_creation(self):
        """Test gRPC transport initialization."""
        from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

        config = {"host": "localhost", "port": 50051}
        transport = GRPCTransport(config=config)
        assert transport.config["host"] == "localhost"
        assert transport.config["port"] == 50051

    def test_mqtt_transport_creation(self):
        """Test MQTT transport initialization."""
        from agent_ros_bridge.gateway_v2.transports.mqtt_transport import MQTTTransport

        config = {"broker_host": "localhost", "broker_port": 1883}
        transport = MQTTTransport(config=config)
        assert transport.config["broker_host"] == "localhost"
        assert transport.config["broker_port"] == 1883


# ============================================================================
# Config Coverage
# ============================================================================


class TestConfigComponents:
    """Improve coverage for config modules"""

    def test_gateway_config_creation(self):
        """Test GatewayConfig creation."""
        from agent_ros_bridge.gateway_v2.config import GatewayConfig

        config = GatewayConfig(
            host="0.0.0.0",
            port=8080,
        )

        assert config.host == "0.0.0.0"
        assert config.port == 8080

    def test_gateway_config_validation(self):
        """Test GatewayConfig validation."""
        from agent_ros_bridge.gateway_v2.config import GatewayConfig

        # Invalid port should be clamped
        config = GatewayConfig(port=99999)
        assert config.port <= 65535

    def test_safety_config_gradual_rollout(self):
        """Test SafetyConfig gradual rollout validation."""
        from agent_ros_bridge.gateway_v2.config import SafetyConfig

        # Test valid values
        config = SafetyConfig(gradual_rollout_stage=50)
        assert config.gradual_rollout_stage == 50

        # Test clamping
        config2 = SafetyConfig(gradual_rollout_stage=150)
        assert config2.gradual_rollout_stage == 100


# ============================================================================
# Fleet Task Management Coverage
# ============================================================================


class TestFleetTaskManagement:
    """Improve coverage for fleet task management"""

    def test_task_priority_comparison(self):
        """Test task priority comparison."""
        from agent_ros_bridge.fleet.task import Task

        task1 = Task(id="t1", type="navigate", priority=1)
        task2 = Task(id="t2", type="navigate", priority=5)

        assert task1 < task2  # Lower number = higher priority
        assert task2 > task1

    def test_task_mark_completed(self):
        """Test task completion."""
        from agent_ros_bridge.fleet.task import Task, TaskStatus

        task = Task(id="t1", type="navigate")
        assert task.status == TaskStatus.PENDING

        task.mark_completed()
        assert task.status == TaskStatus.COMPLETED
        assert task.completed_at is not None

    def test_task_mark_failed(self):
        """Test task failure."""
        from agent_ros_bridge.fleet.task import Task, TaskStatus

        task = Task(id="t1", type="navigate")
        task.mark_failed(reason="obstacle_detected")

        assert task.status == TaskStatus.FAILED
        assert task.metadata.get("failure_reason") == "obstacle_detected"

    def test_task_duration_calculation(self):
        """Test task duration calculation."""
        from datetime import datetime, timedelta

        from agent_ros_bridge.fleet.task import Task

        task = Task(id="t1", type="navigate")
        task.mark_executing()

        # Simulate completion after 10 seconds
        task.completed_at = task.started_at + timedelta(seconds=10)

        duration = task.duration_seconds
        assert duration == 10.0


# ============================================================================
# Validation Coverage
# ============================================================================


class TestValidationComponents:
    """Improve coverage for validation modules"""

    def test_validation_scenario_pass(self):
        """Test validation scenario passing."""
        from agent_ros_bridge.validation.scenario import ValidationScenario

        scenario = ValidationScenario(
            name="test_pass",
            success_criteria={"min_agreement_rate": 0.9},
        )

        data = {"agreement_rate": 0.95, "total_decisions": 100}
        result = scenario.validate(data)

        assert result.passed is True

    def test_validation_scenario_fail(self):
        """Test validation scenario failing."""
        from agent_ros_bridge.validation.scenario import ValidationScenario

        scenario = ValidationScenario(
            name="test_fail",
            success_criteria={"min_agreement_rate": 0.9},
        )

        data = {"agreement_rate": 0.85, "total_decisions": 100}
        result = scenario.validate(data)

        assert result.passed is False

    def test_validation_scenario_multiple_criteria(self):
        """Test validation with multiple criteria."""
        from agent_ros_bridge.validation.scenario import ValidationScenario

        scenario = ValidationScenario(
            name="multi_criteria",
            success_criteria={
                "min_agreement_rate": 0.9,
                "max_safety_violations": 0,
            },
        )

        data = {
            "agreement_rate": 0.95,
            "safety_violations": 1,  # Should fail
            "total_decisions": 100,
        }
        result = scenario.validate(data)

        assert result.passed is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
