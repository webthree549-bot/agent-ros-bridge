"""Tests for scene understanding module."""

import pytest

from agent_ros_bridge.integrations.scene_understanding import (
    DetectedObject,
    PerceptionBackend,
    SceneDescription,
    SceneUnderstanding,
    describe_the_room,
    is_path_clear,
    what_do_you_see,
)


class TestPerceptionBackend:
    """Test PerceptionBackend enum."""

    def test_none_backend(self):
        """Test NONE backend."""
        assert PerceptionBackend.NONE.value == "none"

    def test_claude_backend(self):
        """Test CLAUDE backend."""
        assert PerceptionBackend.CLAUDE.value == "claude"

    def test_openai_backend(self):
        """Test OPENAI backend."""
        assert PerceptionBackend.OPENAI.value == "openai"

    def test_local_backend(self):
        """Test LOCAL backend."""
        assert PerceptionBackend.LOCAL.value == "local"


class TestDetectedObject:
    """Test DetectedObject dataclass."""

    def test_object_creation(self):
        """Test creating detected object."""
        obj = DetectedObject(
            label="chair",
            confidence=0.95,
            bounding_box={"x": 10, "y": 20, "width": 50, "height": 50},
        )
        assert obj.label == "chair"
        assert obj.confidence == 0.95

    def test_object_without_bbox(self):
        """Test creating object without bounding box."""
        obj = DetectedObject(label="table", confidence=0.8)
        assert obj.bounding_box is None


class TestSceneDescription:
    """Test SceneDescription dataclass."""

    def test_description_creation(self):
        """Test creating scene description."""
        desc = SceneDescription(
            summary="A room with furniture",
            objects=[],
            spatial_layout="Open space",
            hazards=[],
            confidence=0.9,
        )
        assert desc.summary == "A room with furniture"
        assert desc.confidence == 0.9


class TestSceneUnderstandingInitialization:
    """Test SceneUnderstanding initialization."""

    def test_init_default_backend(self):
        """Test default NONE backend."""
        scene = SceneUnderstanding()
        assert scene.backend == PerceptionBackend.NONE
        assert scene.api_key is None

    def test_init_with_backend(self):
        """Test initialization with specific backend."""
        scene = SceneUnderstanding(backend=PerceptionBackend.CLAUDE)
        assert scene.backend == PerceptionBackend.CLAUDE

    def test_init_with_api_key(self):
        """Test initialization with API key."""
        scene = SceneUnderstanding(
            backend=PerceptionBackend.CLAUDE,
            api_key="test-key",
        )
        assert scene.api_key == "test-key"


class TestDescribeBasic:
    """Test basic description without vision API."""

    @pytest.mark.asyncio
    async def test_describe_basic(self):
        """Test basic description."""
        scene = SceneUnderstanding()
        desc = await scene.describe_scene(b"fake_image_data")
        assert "Camera image" in desc.summary
        assert desc.confidence == 0.0
        assert desc.objects == []

    @pytest.mark.asyncio
    async def test_describe_basic_with_camera_info(self):
        """Test basic description with camera info."""
        scene = SceneUnderstanding()
        camera_info = {"topic": "/front_camera", "timestamp": "2024-01-01"}
        desc = await scene.describe_scene(b"fake_image_data", camera_info)
        assert "/front_camera" in desc.summary
        assert "2024-01-01" in desc.summary


class TestAnswerQuery:
    """Test query answering."""

    @pytest.mark.asyncio
    async def test_what_do_you_see(self):
        """Test 'what do you see' query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("what do you see")
        assert "camera" in answer.lower()

    @pytest.mark.asyncio
    async def test_describe_query(self):
        """Test 'describe' query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("describe the scene")
        assert "camera" in answer.lower()

    @pytest.mark.asyncio
    async def test_in_front_query(self):
        """Test 'in front' query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("what is in front")
        assert "sensor" in answer.lower() or "forward" in answer.lower()

    @pytest.mark.asyncio
    async def test_distance_query(self):
        """Test distance query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("how far is the wall")
        assert "distance" in answer.lower() or "lidar" in answer.lower()

    @pytest.mark.asyncio
    async def test_object_query(self):
        """Test object query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("do you see any objects")
        assert "vision" in answer.lower() or "camera" in answer.lower()

    @pytest.mark.asyncio
    async def test_generic_query(self):
        """Test generic query."""
        scene = SceneUnderstanding()
        answer = await scene.answer_query("something random")
        assert "camera" in answer.lower() or "vision" in answer.lower()


class TestGetPerceptionStatus:
    """Test perception status."""

    def test_status_none_backend(self):
        """Test status with NONE backend."""
        scene = SceneUnderstanding()
        status = scene.get_perception_status()
        assert status["backend"] == "none"
        assert status["available"] is False
        assert "image_capture" in status["capabilities"]

    def test_status_claude_backend(self):
        """Test status with CLAUDE backend."""
        scene = SceneUnderstanding(backend=PerceptionBackend.CLAUDE, api_key="key")
        status = scene.get_perception_status()
        # Backend may fallback to NONE if anthropic not installed
        assert status["backend"] in ["claude", "none"]
        assert status["api_configured"] is True

    def test_status_without_api_key(self):
        """Test status without API key."""
        scene = SceneUnderstanding(backend=PerceptionBackend.CLAUDE)
        status = scene.get_perception_status()
        assert status["api_configured"] is False


class TestParseDescription:
    """Test description parsing."""

    def test_parse_simple_description(self):
        """Test parsing simple description text."""
        scene = SceneUnderstanding()
        text = "I see a room with a table and chairs."
        desc = scene._parse_description(text)
        assert desc.summary == text
        assert desc.confidence == 0.0  # NONE backend

    def test_parse_with_hazards(self):
        """Test parsing description with hazards."""
        scene = SceneUnderstanding(backend=PerceptionBackend.CLAUDE)
        text = "There is an obstacle in front of the robot."
        desc = scene._parse_description(text)
        assert len(desc.hazards) > 0
        assert desc.confidence == 0.8


class TestConvenienceFunctions:
    """Test convenience functions."""

    @pytest.mark.asyncio
    async def test_what_do_you_see_with_image(self):
        """Test what_do_you_see with image."""
        scene = SceneUnderstanding()
        result = await what_do_you_see(scene, b"fake_image")
        assert "Camera image" in result

    @pytest.mark.asyncio
    async def test_what_do_you_see_without_image(self):
        """Test what_do_you_see without image."""
        scene = SceneUnderstanding()
        result = await what_do_you_see(scene)
        assert "active" in result.lower()

    @pytest.mark.asyncio
    async def test_describe_the_room_with_image(self):
        """Test describe_the_room with image."""
        scene = SceneUnderstanding()
        result = await describe_the_room(scene, b"fake_image")
        assert "Camera image" in result

    @pytest.mark.asyncio
    async def test_describe_the_room_without_image(self):
        """Test describe_the_room without image."""
        scene = SceneUnderstanding()
        result = await describe_the_room(scene)
        assert "requires" in result.lower()


class TestIsPathClear:
    """Test path clearing check."""

    def test_path_clear_empty_data(self):
        """Test path clear with no data."""
        result = is_path_clear([])
        assert result is True

    def test_path_clear_all_far(self):
        """Test path clear when all readings are far."""
        lidar_data = [1.0, 2.0, 3.0, 2.0, 1.0] * 10
        result = is_path_clear(lidar_data, threshold=0.5)
        assert result is True

    def test_path_not_clear_obstacle(self):
        """Test path not clear with obstacle."""
        # Create data with obstacle in center
        lidar_data = [1.0] * 50 + [0.3] * 20 + [1.0] * 50
        result = is_path_clear(lidar_data, threshold=0.5)
        assert result is False

    def test_path_clear_threshold(self):
        """Test path clear with different threshold."""
        lidar_data = [0.6] * 100
        result = is_path_clear(lidar_data, threshold=0.5)
        assert result is True

    def test_path_ignores_zero_readings(self):
        """Test that zero readings are ignored."""
        lidar_data = [0.0] * 100
        result = is_path_clear(lidar_data, threshold=0.5)
        # Zero readings are ignored, so path is clear
        assert result is True
