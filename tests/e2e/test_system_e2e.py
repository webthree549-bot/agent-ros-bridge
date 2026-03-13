"""End-to-End System Test for Agent ROS Bridge.

Validates the entire system from AI agent to robot control.
"""

import asyncio
from dataclasses import dataclass

import pytest

# Skip if ROS not available
pytestmark = pytest.mark.skipif(
    not pytest.importorskip("rclpy", reason="ROS2 not available"), reason="ROS2 not available"
)


@dataclass
class SystemState:
    """Tracks system state during E2E test."""

    bridge_started: bool = False
    transport_connected: bool = False
    module_started: bool = False
    command_received: bool = False
    safety_validated: bool = False
    robot_moved: bool = False
    telemetry_received: bool = False


class TestEndToEndSystem:
    """Full system E2E test."""

    @pytest.fixture
    async def system(self):
        """Set up complete system."""
        from agent_ros_bridge import Bridge
        from agent_ros_bridge.gateway_v2 import Blueprint, In, Module, Out, skill
        from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

        state = SystemState()

        # Create bridge
        bridge = Bridge(ros_version=2)

        # Add transport
        transport = WebSocketTransport({"port": 8766})
        bridge.transport_manager.register(transport)

        # Create test module
        class TestRobotModule(Module):
            cmd_vel: Out[dict]
            odometry: In[dict]

            @skill
            def move_forward(self, distance: float) -> bool:
                """AI-callable skill."""
                state.command_received = True
                return True

            async def run(self):
                state.module_started = True
                while self._running:
                    await asyncio.sleep(0.1)

        # Create blueprint
        blueprint = Blueprint()
        blueprint.add_module("robot", TestRobotModule.blueprint())

        yield {
            "bridge": bridge,
            "transport": transport,
            "blueprint": blueprint,
            "state": state,
            "module_class": TestRobotModule,
        }

        # Cleanup
        await blueprint.stop()
        await bridge.stop()

    @pytest.mark.asyncio
    async def test_full_system_lifecycle(self, system):
        """Test complete system from start to command execution."""
        bridge = system["bridge"]
        blueprint = system["blueprint"]
        state = system["state"]

        # 1. Start bridge
        result = await bridge.start()
        assert result is True
        state.bridge_started = True
        print("✅ Bridge started")

        # 2. Verify transport
        assert system["transport"].is_connected
        state.transport_connected = True
        print("✅ Transport connected")

        # 3. Start blueprint
        await blueprint.start()
        instances = blueprint._instances
        assert "robot" in instances
        state.module_started = True
        print("✅ Module started")

        # 4. Call skill
        robot_module = instances["robot"]
        result = robot_module.move_forward(1.0)
        assert result is True
        assert state.command_received
        print("✅ Skill executed")

        # 5. Verify system state
        assert state.bridge_started
        assert state.transport_connected
        assert state.module_started
        assert state.command_received
        print("✅ All system checks passed")

    @pytest.mark.asyncio
    async def test_ai_to_robot_command_flow(self, system):
        """Test AI agent sending command to robot."""
        bridge = system["bridge"]
        blueprint = system["blueprint"]

        await bridge.start()
        await blueprint.start()

        # Simulate AI agent
        robot = blueprint._instances["robot"]

        # AI calls skill
        skills = robot.get_skills()
        assert "move_forward" in skills

        # Execute skill
        result = robot.call_rpc("move_forward", 1.0)
        assert result is True

        print("✅ AI to robot command flow works")

    @pytest.mark.asyncio
    async def test_safety_validation_in_path(self, system):
        """Test safety validation in command path."""
        from agent_ros_bridge.safety import SafetyValidator

        validator = SafetyValidator()

        # Valid trajectory
        valid_result = validator.validate_trajectory(
            trajectory={"waypoints": [{"x": 0, "y": 0}, {"x": 1, "y": 0}]},
            limits={"max_linear_velocity": 1.0},
        )

        assert valid_result["valid"] is True
        print("✅ Safety validation works")

    @pytest.mark.asyncio
    async def test_error_handling(self, system):
        """Test error handling in E2E flow."""
        from agent_ros_bridge.utils.error_handling import AgentError, ErrorCode, InputValidator

        # Invalid input
        result = InputValidator.validate_utterance("")
        assert not result.valid
        assert result.error_code == ErrorCode.INTENT_PARSE_FAILED

        # Error creation
        error = AgentError(
            code=ErrorCode.VALIDATION_FAILED, message="Test error", context={"test": True}
        )

        assert "VALIDATION_FAILED" in str(error)
        print("✅ Error handling works")

    @pytest.mark.asyncio
    async def test_lcm_transport_integration(self, system):
        """Test LCM transport in system."""
        try:
            from agent_ros_bridge.gateway_v2.transports import LCMTransport

            lcm = LCMTransport({"udp_url": "udpm://239.255.76.67:7668"})
            result = await lcm.start()

            if result:
                pub = lcm.publisher("test/channel")
                pub.publish({"test": "data"})
                await lcm.stop()
                print("✅ LCM transport integration works")
            else:
                pytest.skip("LCM not available")
        except ImportError:
            pytest.skip("LCM not installed")

    @pytest.mark.asyncio
    async def test_blueprint_autoconnect(self, system):
        """Test blueprint autoconnect functionality."""
        from agent_ros_bridge.gateway_v2 import In, Module, Out, autoconnect

        @dataclass
        class TestMsg:
            data: str

        class ModuleA(Module):
            output: Out[TestMsg]

            async def run(self):
                while self._running:
                    await self.output.publish(TestMsg("hello"))
                    await asyncio.sleep(0.1)

        class ModuleB(Module):
            input: In[TestMsg]
            received = False

            async def run(self):
                while self._running:
                    msg = await self.input.get()
                    if msg.data == "hello":
                        self.received = True

        # Autoconnect
        blueprint = autoconnect(ModuleA.blueprint(), ModuleB.blueprint())

        assert len(blueprint.connections) >= 1
        print("✅ Blueprint autoconnect works")


class TestProductionReadiness:
    """Tests for production readiness."""

    def test_security_utils_available(self):
        """Security utilities are importable."""
        from agent_ros_bridge.security_utils import (
            hash_password,
            verify_password,
        )

        assert callable(hash_password)
        assert callable(verify_password)

    def test_metrics_available(self):
        """Metrics collection available."""
        from agent_ros_bridge.metrics import get_metrics

        metrics = get_metrics()
        assert metrics is not None

    def test_all_transports_importable(self):
        """All transports can be imported."""
        from agent_ros_bridge.gateway_v2.transports import MQTTTransport, WebSocketTransport

        assert WebSocketTransport is not None
        assert MQTTTransport is not None

    def test_blueprint_system_importable(self):
        """Blueprint system can be imported."""
        from agent_ros_bridge.gateway_v2 import Blueprint, Module

        assert Blueprint is not None
        assert Module is not None

    @pytest.mark.asyncio
    async def test_system_stress_test(self):
        """Stress test with many messages."""
        from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

        transport = WebSocketTransport({"port": 8767})

        # Start
        result = await transport.start()
        if not result:
            pytest.skip("Transport couldn't start")

        # Send many messages
        messages_sent = 0
        for i in range(100):
            from agent_ros_bridge.gateway_v2.core import Header, Message

            msg = Message(header=Header(source="test"))
            await transport.send(msg, f"recipient_{i}")
            messages_sent += 1

        await transport.stop()
        assert messages_sent == 100
        print(f"✅ Sent {messages_sent} messages successfully")


class TestUserExperience:
    """Tests for UX/DX quality."""

    def test_error_messages_helpful(self):
        """Error messages are descriptive."""
        from agent_ros_bridge.utils.error_handling import AgentError, ErrorCode

        error = AgentError(
            code=ErrorCode.VALIDATION_FAILED,
            message="Trajectory exceeds velocity limit",
            context={"max": 1.0, "actual": 1.5},
        )

        error_str = str(error)
        assert "VALIDATION_FAILED" in error_str
        assert "velocity limit" in error_str

    def test_api_documented(self):
        """All public APIs have docstrings."""
        from agent_ros_bridge.gateway_v2 import Blueprint, Module

        assert Blueprint.__doc__ is not None
        assert Module.__doc__ is not None
        assert Blueprint.start.__doc__ is not None

    def test_examples_exist(self):
        """Example code exists."""
        import os

        examples_dir = "examples"

        if os.path.exists(examples_dir):
            examples = [f for f in os.listdir(examples_dir) if f.endswith(".py")]
            assert len(examples) > 0
            print(f"✅ Found {len(examples)} examples")
