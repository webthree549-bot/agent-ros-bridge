"""Tests for Blueprint pattern - TDD approach.

Following TDD principles:
1. Write tests first (this file)
2. Implement minimal code to pass
3. Refactor
"""

from dataclasses import dataclass

import pytest

from agent_ros_bridge.gateway_v2.blueprint import (Blueprint, Connection,
                                                   ModuleBlueprint,
                                                   StreamDefinition,
                                                   autoconnect, rpc, skill)
from agent_ros_bridge.gateway_v2.module import In, Module, Out


# Test message types
@dataclass
class TestImage:
    width: int
    height: int
    data: bytes


@dataclass
class TestDetection:
    label: str
    confidence: float


# Test modules
class CameraModule(Module):
    """Test camera module."""

    image: Out[TestImage]

    def __init__(self, camera_id: str = "cam0"):
        super().__init__(name=f"camera_{camera_id}")
        self.camera_id = camera_id

    async def run(self) -> None:
        while self._running:
            img = TestImage(640, 480, b"test")
            await self.image.publish(img)
            await asyncio.sleep(0.1)


class DetectorModule(Module):
    """Test detector module."""

    image: In[TestImage]
    detection: Out[TestDetection]

    async def run(self) -> None:
        while self._running:
            img = await self.image.get()
            det = TestDetection("person", 0.95)
            await self.detection.publish(det)


class TestStreamDefinition:
    """Test StreamDefinition dataclass."""

    def test_stream_creation(self):
        """Can create stream definition."""
        stream = StreamDefinition(name="image", msg_type=TestImage, direction="out")

        assert stream.name == "image"
        assert stream.msg_type == TestImage
        assert stream.direction == "out"
        assert stream.transport == "lcm"

    def test_stream_invalid_direction(self):
        """Invalid direction raises error."""
        with pytest.raises(ValueError):
            StreamDefinition(name="test", msg_type=str, direction="invalid")


class TestModuleBlueprint:
    """Test ModuleBlueprint."""

    def test_blueprint_creation(self):
        """Can create module blueprint."""
        bp = ModuleBlueprint(name="test_module", factory=lambda: CameraModule())

        assert bp.name == "test_module"
        assert bp.streams == []
        assert bp.rpcs == []

    def test_blueprint_with_stream(self):
        """Can add stream to blueprint."""
        bp = ModuleBlueprint(name="camera", factory=lambda: CameraModule())

        stream = StreamDefinition("image", TestImage, "out")
        bp.with_stream(stream)

        assert len(bp.streams) == 1
        assert bp.streams[0].name == "image"

    def test_blueprint_with_config(self):
        """Can add config to blueprint."""
        bp = ModuleBlueprint(
            name="camera", factory=lambda **cfg: CameraModule(cfg.get("camera_id", "default"))
        )

        bp.with_config(camera_id="front", fps=30)

        assert bp.config["camera_id"] == "front"
        assert bp.config["fps"] == 30

    def test_blueprint_build(self):
        """Can build module from blueprint."""
        bp = ModuleBlueprint(
            name="camera", factory=lambda **cfg: CameraModule(cfg.get("camera_id", "default"))
        )

        module = bp.build(camera_id="test_cam")

        assert isinstance(module, CameraModule)
        assert module.camera_id == "test_cam"


class TestConnection:
    """Test Connection dataclass."""

    def test_connection_creation(self):
        """Can create connection."""
        conn = Connection(
            source_module="camera",
            source_stream="image",
            target_module="detector",
            target_stream="image",
        )

        assert conn.source_module == "camera"
        assert conn.target_module == "detector"


class TestBlueprint:
    """Test Blueprint class."""

    def test_blueprint_creation(self):
        """Can create blueprint."""
        blueprint = Blueprint()

        assert blueprint.modules == {}
        assert blueprint.connections == []
        assert not blueprint._running

    def test_add_module(self):
        """Can add module to blueprint."""
        blueprint = Blueprint()
        bp = CameraModule.blueprint()

        blueprint.add_module("camera", bp)

        assert "camera" in blueprint.modules

    def test_connect_modules(self):
        """Can connect modules."""
        blueprint = Blueprint()

        blueprint.connect("camera", "image", "detector", "image")

        assert len(blueprint.connections) == 1
        conn = blueprint.connections[0]
        assert conn.source_module == "camera"
        assert conn.target_module == "detector"

    def test_autoconnect_matching_streams(self):
        """Autoconnect matches streams by name and type."""
        blueprint = Blueprint()

        camera_bp = ModuleBlueprint(
            name="camera",
            factory=CameraModule,
            streams=[StreamDefinition("image", TestImage, "out")],
        )

        detector_bp = ModuleBlueprint(
            name="detector",
            factory=DetectorModule,
            streams=[StreamDefinition("image", TestImage, "in")],
        )

        blueprint.add_module("camera", camera_bp)
        blueprint.add_module("detector", detector_bp)
        blueprint.autoconnect()

        assert len(blueprint.connections) == 1

    @pytest.mark.asyncio
    async def test_build_modules(self):
        """Can build all modules."""
        blueprint = Blueprint()
        bp = CameraModule.blueprint()

        blueprint.add_module("camera", bp)
        instances = blueprint.build()

        assert "camera" in instances
        assert isinstance(instances["camera"], CameraModule)


class TestAutoconnect:
    """Test autoconnect function."""

    def test_autoconnect_creates_blueprint(self):
        """Autoconnect returns blueprint."""
        camera_bp = CameraModule.blueprint()
        detector_bp = DetectorModule.blueprint()

        blueprint = autoconnect(camera_bp, detector_bp)

        assert isinstance(blueprint, Blueprint)
        assert len(blueprint.modules) == 2


class TestDecorators:
    """Test skill and rpc decorators."""

    def test_skill_decorator(self):
        """Skill decorator marks function."""

        @skill
        def my_skill():
            return "result"

        assert hasattr(my_skill, "_is_skill")
        assert my_skill._is_skill is True
        assert hasattr(my_skill, "_rpc_definition")

    def test_rpc_decorator(self):
        """RPC decorator marks function."""

        @rpc
        def my_rpc():
            return "result"

        assert hasattr(my_rpc, "_is_rpc")
        assert my_rpc._is_rpc is True


class TestBlueprintIntegration:
    """Integration tests for Blueprint."""

    @pytest.mark.asyncio
    async def test_full_blueprint_lifecycle(self):
        """Test full blueprint lifecycle."""
        blueprint = Blueprint()

        # Add modules
        camera_bp = CameraModule.blueprint()
        detector_bp = DetectorModule.blueprint()

        blueprint.add_module("camera", camera_bp)
        blueprint.add_module("detector", detector_bp)

        # Connect
        blueprint.connect("camera", "image", "detector", "image")

        # Start
        await blueprint.start()
        assert blueprint._running

        # Stop
        await blueprint.stop()
        assert not blueprint._running


# Need to import asyncio for async tests
import asyncio
