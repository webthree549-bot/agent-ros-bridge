"""Scene understanding and visual perception.

Provides camera image interpretation and scene description.
This is an optional feature that can work with or without external vision APIs.

Fulfills SKILL promises like:
- "What do you see?"
- "Describe the room"
- "Is there anything in front?"
"""

import base64
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum


class PerceptionBackend(Enum):
    """Available perception backends."""

    NONE = "none"  # No vision API, basic metadata only
    CLAUDE = "claude"  # Claude vision API
    OPENAI = "openai"  # GPT-4V API
    LOCAL = "local"  # Local vision model (future)


@dataclass
class DetectedObject:
    """An object detected in the scene."""

    label: str
    confidence: float
    bounding_box: Optional[Dict[str, int]] = None  # x, y, width, height
    distance_estimate: Optional[float] = None  # meters


@dataclass
class SceneDescription:
    """Description of a scene."""

    summary: str
    objects: List[DetectedObject]
    spatial_layout: str
    hazards: List[str]
    confidence: float


class SceneUnderstanding:
    """Scene understanding and visual perception.

    Provides:
    - Camera image capture
    - Scene description (with optional vision API)
    - Object detection
    - Spatial analysis

    Note: Without vision API, provides basic metadata only.
    """

    def __init__(
        self, backend: PerceptionBackend = PerceptionBackend.NONE, api_key: Optional[str] = None
    ):
        """Initialize scene understanding.

        Args:
            backend: Vision API backend to use
            api_key: API key for vision service (if using external API)
        """
        self.backend = backend
        self.api_key = api_key
        self._vision_client = None

        if backend != PerceptionBackend.NONE and api_key:
            self._init_vision_client()

    def _init_vision_client(self):
        """Initialize vision API client."""
        if self.backend == PerceptionBackend.CLAUDE:
            try:
                import anthropic

                self._vision_client = anthropic.Anthropic(api_key=self.api_key)
            except ImportError:
                print(
                    "Warning: anthropic package not installed. "
                    "Scene understanding will use basic mode."
                )
                self.backend = PerceptionBackend.NONE

        elif self.backend == PerceptionBackend.OPENAI:
            try:
                import openai

                self._vision_client = openai.OpenAI(api_key=self.api_key)
            except ImportError:
                print(
                    "Warning: openai package not installed. "
                    "Scene understanding will use basic mode."
                )
                self.backend = PerceptionBackend.NONE

    async def describe_scene(self, image_data: bytes, camera_info: Dict = None) -> SceneDescription:
        """Generate description of scene from camera image.

        Args:
            image_data: Raw image bytes
            camera_info: Camera metadata (topic, timestamp, etc.)

        Returns:
            Scene description
        """
        if self.backend == PerceptionBackend.NONE or not self._vision_client:
            return self._describe_basic(camera_info)

        if self.backend == PerceptionBackend.CLAUDE:
            return await self._describe_with_claude(image_data, camera_info)

        if self.backend == PerceptionBackend.OPENAI:
            return await self._describe_with_openai(image_data, camera_info)

        return self._describe_basic(camera_info)

    def _describe_basic(self, camera_info: Dict = None) -> SceneDescription:
        """Generate basic description without vision API.

        This provides metadata only when no vision API is available.
        """
        info = camera_info or {}
        topic = info.get("topic", "/camera/image_raw")
        timestamp = info.get("timestamp", "unknown")

        summary = f"Camera image from {topic} at {timestamp}"

        return SceneDescription(
            summary=summary,
            objects=[],
            spatial_layout="No spatial analysis available (vision API not configured)",
            hazards=[],
            confidence=0.0,
        )

    async def _describe_with_claude(self, image_data: bytes, camera_info: Dict) -> SceneDescription:
        """Generate description using Claude vision API."""
        try:
            # Encode image to base64
            image_b64 = base64.b64encode(image_data).decode("utf-8")

            # Determine image format (assume JPEG for now)
            media_type = "image/jpeg"

            response = self._vision_client.messages.create(
                model="claude-3-opus-20240229",
                max_tokens=500,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": media_type,
                                    "data": image_b64,
                                },
                            },
                            {
                                "type": "text",
                                "text": """Describe what you see in this robot camera image.
                            
Provide:
1. A brief summary of the scene
2. List of visible objects
3. Spatial layout (what's in front, sides, etc.)
4. Any potential hazards or obstacles

Be concise but informative.""",
                            },
                        ],
                    }
                ],
            )

            description_text = response.content[0].text

            # Parse the response (simple parsing)
            return self._parse_description(description_text)

        except Exception as e:
            print(f"Vision API error: {e}")
            return self._describe_basic(camera_info)

    async def _describe_with_openai(self, image_data: bytes, camera_info: Dict) -> SceneDescription:
        """Generate description using GPT-4V API."""
        try:
            import openai

            image_b64 = base64.b64encode(image_data).decode("utf-8")

            response = self._vision_client.chat.completions.create(
                model="gpt-4-vision-preview",
                max_tokens=500,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": "Describe what you see in this robot camera image. "
                                "List objects, spatial layout, and any hazards.",
                            },
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                            },
                        ],
                    }
                ],
            )

            description_text = response.choices[0].message.content
            return self._parse_description(description_text)

        except Exception as e:
            print(f"Vision API error: {e}")
            return self._describe_basic(camera_info)

    def _parse_description(self, text: str) -> SceneDescription:
        """Parse vision API response into structured description."""
        # Simple parsing - in production would use more sophisticated NLP

        lines = text.strip().split("\n")

        # First non-empty line is summary
        summary = next((l for l in lines if l.strip()), "Scene observed")

        # Look for object mentions
        objects = []
        hazard_keywords = ["obstacle", "hazard", "danger", "blocked", "person"]
        hazards = []

        for line in lines:
            line_lower = line.lower()

            # Check for hazards
            for hazard in hazard_keywords:
                if hazard in line_lower:
                    hazards.append(line.strip())
                    break

            # Simple object extraction (would be better with NLP)
            if any(word in line_lower for word in ["see", "visible", "there is", "there are"]):
                # Extract potential object names
                pass  # Would need NLP for proper extraction

        return SceneDescription(
            summary=summary,
            objects=objects,
            spatial_layout="See summary for spatial details",
            hazards=hazards,
            confidence=0.8 if self.backend != PerceptionBackend.NONE else 0.0,
        )

    async def answer_query(
        self, query: str, image_data: bytes = None, camera_info: Dict = None
    ) -> str:
        """Answer a specific query about the scene.

        Args:
            query: Natural language query (e.g., "What's in front?")
            image_data: Optional camera image
            camera_info: Camera metadata

        Returns:
            Answer to the query
        """
        query_lower = query.lower()

        # Handle specific queries without vision API
        if "what do you see" in query_lower or "describe" in query_lower:
            if image_data and self.backend != PerceptionBackend.NONE:
                desc = await self.describe_scene(image_data, camera_info)
                return desc.summary
            else:
                return (
                    "I can see the camera feed, but detailed scene understanding "
                    "requires a vision API (Claude or OpenAI). "
                    "Camera is operational and capturing images."
                )

        if "in front" in query_lower:
            return (
                "Forward sensor data would be needed for obstacle detection. "
                "Use LiDAR or depth camera for precise front detection."
            )

        if "distance" in query_lower or "how far" in query_lower:
            return "Distance estimation requires LiDAR or depth camera data."

        if "object" in query_lower or "see" in query_lower:
            return "Object detection requires vision API configuration."

        # Generic response
        return (
            "I can access camera data but detailed visual analysis requires "
            "a vision API. Basic camera operations are functional."
        )

    def get_perception_status(self) -> Dict[str, Any]:
        """Get status of perception system.

        Returns:
            Status information
        """
        return {
            "backend": self.backend.value,
            "available": self.backend != PerceptionBackend.NONE,
            "api_configured": self.api_key is not None,
            "client_initialized": self._vision_client is not None,
            "capabilities": ["image_capture", "basic_metadata"]
            + (
                ["scene_description", "object_detection"]
                if self.backend != PerceptionBackend.NONE
                else []
            ),
        }


# Convenience functions for SKILL fulfillment


async def what_do_you_see(scene: SceneUnderstanding, image_data: bytes = None) -> str:
    """Answer 'What do you see?' query.

    Convenience function for SKILL fulfillment.
    """
    if image_data:
        desc = await scene.describe_scene(image_data)
        return desc.summary
    else:
        return "Camera is active. Capture an image to analyze the scene."


async def describe_the_room(scene: SceneUnderstanding, image_data: bytes = None) -> str:
    """Answer 'Describe the room' query.

    Convenience function for SKILL fulfillment.
    """
    if image_data:
        desc = await scene.describe_scene(image_data)
        return f"{desc.summary}\n\nObjects: {', '.join(o.label for o in desc.objects)}"
    else:
        return "Room description requires camera image capture."


def is_path_clear(lidar_data: List[float], threshold: float = 0.5) -> bool:
    """Check if path is clear based on LiDAR data.

    Args:
        lidar_data: Distance readings
        threshold: Minimum clear distance in meters

    Returns:
        True if path is clear
    """
    if not lidar_data:
        return True  # Assume clear if no data

    # Check forward sector (center of LiDAR sweep)
    center_idx = len(lidar_data) // 2
    forward_sector = lidar_data[center_idx - 10 : center_idx + 10]

    return all(d > threshold for d in forward_sector if d > 0)
