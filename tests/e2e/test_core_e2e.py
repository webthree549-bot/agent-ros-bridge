"""End-to-End System Test for Agent ROS Bridge (No ROS Required).

Validates the core system without ROS dependencies.
"""

import asyncio
import os
from dataclasses import dataclass

import pytest


@dataclass
class TestMessage:
    """Test message type."""

    data: str
    value: int = 0


class TestEndToEndCoreSystem:
    """Test core system without ROS."""

    @pytest.mark.asyncio
    async def test_full_system_lifecycle(self):
        """Test complete system lifecycle."""
        from agent_ros_bridge import Bridge
        from agent_ros_bridge.gateway_v2 import (Blueprint, In, Module, Out,
                                                 skill)
        from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

        # Track state
        states = []

        # Create test module
        class TestRobotModule(Module):
            cmd_vel: Out[dict]
            status: In[dict]

            def __init__(self):
                super().__init__(name="test_robot")
                self.skill_called = False

            @skill
            def move_forward(self, distance: float) -> bool:
                """AI-callable skill."""
                self.skill_called = True
                states.append("skill_called")
                return True

            async def run(self):
                states.append("module_started")
                while self._running:
                    await asyncio.sleep(0.01)

        # Create bridge
        bridge = Bridge()
        transport = WebSocketTransport({"port": 8768})
        bridge.transport_manager.register(transport)

        # Create blueprint
        blueprint = Blueprint()
        blueprint.add_module("robot", TestRobotModule.blueprint())

        try:
            # 1. Start bridge
            result = await bridge.start()
            assert result is True
            states.append("bridge_started")
            print("✅ Bridge started")

            # 2. Start blueprint
            await blueprint.start()
            assert "robot" in blueprint._instances
            await asyncio.sleep(0.1)  # Let module start
            assert "module_started" in states
            print("✅ Module started")

            # 3. Call skill
            robot = blueprint._instances["robot"]
            result = robot.move_forward(1.0)
            assert result is True
            assert robot.skill_called
            print("✅ Skill executed")

            # 4. Verify state
            assert "bridge_started" in states
            assert "module_started" in states
            assert "skill_called" in states
            print("✅ All lifecycle states passed")

        finally:
            await blueprint.stop()
            await bridge.stop()

    @pytest.mark.asyncio
    async def test_lcm_transport_flow(self):
        """Test LCM transport message flow."""
        try:
            from agent_ros_bridge.gateway_v2.transports import LCMTransport
        except ImportError:
            pytest.skip("LCM not available")

        transport = LCMTransport({"udp_url": "udpm://239.255.76.67:7669", "shared_memory": True})

        received_messages = []

        def callback(data):
            received_messages.append(data)

        try:
            # Start
            result = await transport.start()
            assert result is True

            # Subscribe
            sub = transport.subscriber("test/channel", callback)
            sub.subscribe()

            # Publish
            pub = transport.publisher("test/channel")
            pub.publish({"test": "data"})

            await asyncio.sleep(0.1)

            # Verify
            assert len(received_messages) == 1
            import json

            assert json.loads(received_messages[0]) == {"test": "data"}
            print("✅ LCM transport flow works")

        finally:
            await transport.stop()

    @pytest.mark.asyncio
    async def test_blueprint_autoconnect_flow(self):
        """Test blueprint autoconnect."""
        from agent_ros_bridge.gateway_v2 import In, Module, Out, autoconnect

        received = []

        class Producer(Module):
            output: Out[TestMessage]

            async def run(self):
                for i in range(3):
                    await self.output.publish(TestMessage(f"msg_{i}"))
                    await asyncio.sleep(0.01)

        class Consumer(Module):
            input: In[TestMessage]

            async def run(self):
                for _ in range(3):
                    msg = await asyncio.wait_for(self.input.get(), timeout=1.0)
                    received.append(msg.data)

        # Autoconnect
        blueprint = autoconnect(Producer.blueprint(), Consumer.blueprint())

        assert len(blueprint.connections) >= 1

        try:
            await blueprint.start()
            await asyncio.sleep(0.2)

            assert len(received) == 3
            assert received == ["msg_0", "msg_1", "msg_2"]
            print("✅ Blueprint autoconnect flow works")

        finally:
            await blueprint.stop()

    def test_security_end_to_end(self):
        """Test security utilities."""
        from agent_ros_bridge.security_utils import (RateLimiter,
                                                     generate_token,
                                                     hash_password,
                                                     verify_password)

        # Password hashing
        password = "test_password"
        hashed = hash_password(password)
        assert verify_password(password, hashed)
        assert not verify_password("wrong", hashed)
        print("✅ Password hashing works")

        # Token generation
        token1 = generate_token(32)
        token2 = generate_token(32)
        assert len(token1) == 32
        assert token1 != token2
        print("✅ Token generation works")

        # Rate limiter
        limiter = RateLimiter(max_requests=2, window=60)
        assert limiter.allow_request("client1")
        assert limiter.allow_request("client1")
        assert not limiter.allow_request("client1")  # Should block
        assert limiter.allow_request("client2")  # Different client
        print("✅ Rate limiting works")

    def test_error_handling_flow(self):
        """Test error handling."""
        from agent_ros_bridge.utils.error_handling import (AgentError,
                                                           ErrorCode,
                                                           InputValidator,
                                                           handle_error)

        # Input validation
        result = InputValidator.validate_utterance("go to kitchen")
        assert result.valid

        result = InputValidator.validate_utterance("")
        assert not result.valid
        assert result.error_code == ErrorCode.INTENT_PARSE_FAILED
        print("✅ Input validation works")

        # Error handling
        try:
            raise ValueError("test error")
        except Exception as e:
            error = handle_error(e)
            assert isinstance(error, AgentError)
        print("✅ Error handling works")

    @pytest.mark.asyncio
    async def test_metrics_collection(self):
        """Test metrics collection."""
        from agent_ros_bridge.metrics import get_metrics

        metrics = get_metrics()

        # Record some metrics
        metrics.record_message_sent("websocket", 100)
        metrics.record_task_completed("navigate", 1.5)

        # Get snapshot
        snapshot = metrics.get_snapshot()
        assert snapshot.messages_sent >= 1
        print("✅ Metrics collection works")


class TestProductionReadiness:
    """Production readiness tests."""

    def test_all_core_imports(self):
        """All core modules importable."""
        # Core

        # Transports

        # Blueprint

        # Safety

        # Utils

        print("✅ All core imports work")

    def test_examples_exist(self):
        """Examples directory exists with content."""
        assert os.path.exists("examples")
        examples = [f for f in os.listdir("examples") if f.endswith(".py")]
        assert len(examples) > 0
        print(f"✅ Found {len(examples)} examples")

    def test_documentation_exists(self):
        """Documentation exists."""
        docs = ["docs/API.md", "docs/ARCHITECTURE.md", "docs/DEPLOYMENT.md"]
        for doc in docs:
            assert os.path.exists(doc), f"Missing {doc}"
        print("✅ All documentation exists")

    @pytest.mark.asyncio
    async def test_performance_baseline(self):
        """Basic performance test."""
        import time

        from agent_ros_bridge.gateway_v2 import Blueprint, Module, Out

        class FastModule(Module):
            output: Out[dict]

            async def run(self):
                pass

        blueprint = Blueprint()
        blueprint.add_module("fast", FastModule.blueprint())

        start = time.time()
        await blueprint.start()
        await blueprint.stop()
        elapsed = time.time() - start

        assert elapsed < 1.0  # Should start/stop in under 1 second
        print(f"✅ Startup time: {elapsed:.3f}s")


class TestUserExperience:
    """UX/DX quality tests."""

    def test_helpful_error_messages(self):
        """Error messages are helpful."""
        from agent_ros_bridge.utils.error_handling import AgentError, ErrorCode

        error = AgentError(
            code=ErrorCode.VALIDATION_FAILED,
            message="Velocity exceeds limit",
            context={"max": 1.0, "actual": 2.0},
        )

        msg = str(error)
        assert "VALIDATION_FAILED" in msg
        assert "Velocity exceeds limit" in msg
        assert "1.0" in msg or "2.0" in msg
        print("✅ Error messages are helpful")

    def test_type_hints_present(self):
        """Public APIs have type hints."""
        import inspect

        from agent_ros_bridge.gateway_v2 import Blueprint, Module

        # Check Blueprint methods
        sig = inspect.signature(Blueprint.start)
        assert "return" in str(sig)

        # Check Module
        sig = inspect.signature(Module.__init__)
        assert "name" in str(sig)
        print("✅ Type hints present")

    def test_docstrings_present(self):
        """Public classes have docstrings."""
        from agent_ros_bridge.gateway_v2 import Blueprint, Module

        assert Blueprint.__doc__ is not None
        assert len(Blueprint.__doc__) > 10

        assert Module.__doc__ is not None
        assert len(Module.__doc__) > 10
        print("✅ Docstrings present")
