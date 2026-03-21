"""Tests for context-aware parser module."""

import time
from unittest.mock import patch

import pytest

from agent_ros_bridge.ai.context_aware_parser import (
    ContextAwareParser,
    ConversationContext,
    EnvironmentState,
    RobotState,
    UserPreferences,
)


class TestConversationContext:
    """Test ConversationContext dataclass."""

    def test_default_creation(self):
        """Test default initialization."""
        ctx = ConversationContext()
        assert ctx.utterances == []
        assert ctx.intents == []
        assert ctx.max_history == 10

    def test_custom_max_history(self):
        """Test custom max history."""
        ctx = ConversationContext(max_history=5)
        assert ctx.max_history == 5

    def test_add_turn(self):
        """Test adding conversation turn."""
        ctx = ConversationContext()
        ctx.add_turn("hello", "GREETING")

        assert len(ctx.utterances) == 1
        assert ctx.utterances[0] == "hello"
        assert ctx.intents[0] == "GREETING"

    def test_history_limit(self):
        """Test that history respects max limit."""
        ctx = ConversationContext(max_history=3)

        for i in range(5):
            ctx.add_turn(f"utterance {i}", f"INTENT_{i}")

        assert len(ctx.utterances) == 3
        assert ctx.utterances[0] == "utterance 2"  # Oldest kept
        assert ctx.utterances[-1] == "utterance 4"  # Most recent

    def test_timestamp(self):
        """Test timestamp is set."""
        before = time.time()
        ctx = ConversationContext()
        after = time.time()

        assert before <= ctx.timestamp <= after


class TestRobotState:
    """Test RobotState dataclass."""

    def test_default_creation(self):
        """Test default initialization."""
        state = RobotState()
        assert state.location == "unknown"
        assert state.battery_level == 100.0
        assert state.current_task == "idle"
        assert state.is_moving is False

    def test_custom_values(self):
        """Test custom initialization."""
        state = RobotState(
            location="kitchen",
            battery_level=75.0,
            current_task="navigating",
            is_moving=True,
        )
        assert state.location == "kitchen"
        assert state.battery_level == 75.0
        assert state.current_task == "navigating"
        assert state.is_moving is True


class TestEnvironmentState:
    """Test EnvironmentState dataclass."""

    def test_default_creation(self):
        """Test default initialization."""
        env = EnvironmentState()
        assert env.known_locations == []
        assert env.detected_objects == []
        assert env.people_present == []

    def test_custom_values(self):
        """Test custom initialization."""
        env = EnvironmentState(
            known_locations=["kitchen", "office"],
            detected_objects=["cup", "book"],
            people_present=["user"],
        )
        assert env.known_locations == ["kitchen", "office"]
        assert env.detected_objects == ["cup", "book"]
        assert env.people_present == ["user"]


class TestUserPreferences:
    """Test UserPreferences dataclass."""

    def test_default_creation(self):
        """Test default initialization."""
        prefs = UserPreferences()
        assert prefs.preferred_speed == "normal"
        assert prefs.preferred_language == "en"
        assert prefs.common_locations == []

    def test_custom_values(self):
        """Test custom initialization."""
        prefs = UserPreferences(
            preferred_speed="fast",
            preferred_language="es",
            common_locations=["kitchen", "bedroom"],
        )
        assert prefs.preferred_speed == "fast"
        assert prefs.preferred_language == "es"
        assert prefs.common_locations == ["kitchen", "bedroom"]


class TestContextAwareParser:
    """Test ContextAwareParser class."""

    def test_default_initialization(self):
        """Test default parser initialization."""
        parser = ContextAwareParser()
        assert parser._max_history == 10
        assert isinstance(parser._conversation, ConversationContext)
        assert isinstance(parser._robot_state, RobotState)
        assert isinstance(parser._environment, EnvironmentState)
        assert isinstance(parser._user_prefs, UserPreferences)

    def test_custom_max_history(self):
        """Test parser with custom max history."""
        parser = ContextAwareParser(max_history=5)
        assert parser._max_history == 5
        assert parser._conversation.max_history == 5

    def test_update_robot_state_all(self):
        """Test updating all robot state fields."""
        parser = ContextAwareParser()
        parser.update_robot_state(
            location="kitchen",
            battery=75.0,
            task="navigating",
            moving=True,
        )

        assert parser._robot_state.location == "kitchen"
        assert parser._robot_state.battery_level == 75.0
        assert parser._robot_state.current_task == "navigating"
        assert parser._robot_state.is_moving is True

    def test_update_robot_state_partial(self):
        """Test updating partial robot state."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="office")

        assert parser._robot_state.location == "office"
        assert parser._robot_state.battery_level == 100.0  # Unchanged

    def test_update_robot_state_none_values(self):
        """Test that None values don't update state."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="kitchen")
        parser.update_robot_state(location=None, battery=50.0)

        assert parser._robot_state.location == "kitchen"  # Unchanged
        assert parser._robot_state.battery_level == 50.0

    def test_update_robot_state_updates_timestamp(self):
        """Test that update_robot_state updates timestamp."""
        parser = ContextAwareParser()
        old_time = parser._robot_state.last_updated

        with patch("time.time", return_value=old_time + 100):
            parser.update_robot_state(location="kitchen")

        assert parser._robot_state.last_updated == old_time + 100

    def test_update_environment_all(self):
        """Test updating all environment fields."""
        parser = ContextAwareParser()
        parser.update_environment(
            locations=["kitchen", "office"],
            objects=["cup", "book"],
            people=["user"],
        )

        assert parser._environment.known_locations == ["kitchen", "office"]
        assert parser._environment.detected_objects == ["cup", "book"]
        assert parser._environment.people_present == ["user"]

    def test_update_environment_partial(self):
        """Test updating partial environment."""
        parser = ContextAwareParser()
        parser.update_environment(locations=["kitchen"])

        assert parser._environment.known_locations == ["kitchen"]
        assert parser._environment.detected_objects == []  # Unchanged

    def test_add_conversation_turn(self):
        """Test adding conversation turn."""
        parser = ContextAwareParser()
        parser.add_conversation_turn("go to kitchen", "NAVIGATE")

        assert len(parser._conversation.utterances) == 1
        assert parser._conversation.utterances[0] == "go to kitchen"
        assert parser._conversation.intents[0] == "NAVIGATE"


class TestContextResolution:
    """Test context resolution methods."""

    @pytest.fixture
    def parser(self):
        """Create parser with context."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="living_room")
        parser.update_environment(objects=["cup", "book"])
        parser.add_conversation_turn("pick up the cup", "MANIPULATE")
        return parser

    def test_resolve_no_pronouns(self, parser):
        """Test resolution with no pronouns."""
        utterance = "go to kitchen"
        resolved = parser.resolve_context(utterance)
        assert resolved == "go to kitchen"

    def test_resolve_it_from_conversation(self, parser):
        """Test resolving 'it' from conversation history."""
        resolved = parser.resolve_context("place it on table")
        assert "cup" in resolved

    def test_resolve_it_from_environment(self, parser):
        """Test resolving 'it' from environment when no conversation."""
        parser = ContextAwareParser()
        parser.update_environment(objects=["phone", "keys"])

        resolved = parser.resolve_context("pick it up")
        assert "phone" in resolved

    def test_resolve_it_no_match(self, parser):
        """Test resolving 'it' when no match found."""
        parser = ContextAwareParser()
        resolved = parser.resolve_context("pick it up")
        # Should return original if no resolution
        assert "it" in resolved

    def test_resolve_there_from_conversation(self, parser):
        """Test resolving 'there' from conversation."""
        parser.add_conversation_turn("go to kitchen", "NAVIGATE")
        resolved = parser.resolve_context("go there now")  # spaces around 'there'
        assert "kitchen" in resolved

    def test_resolve_there_from_robot_state(self, parser):
        """Test resolving 'there' from robot state."""
        resolved = parser.resolve_context("go there now")  # spaces around 'there'
        assert "living_room" in resolved

    def test_resolve_here(self, parser):
        """Test resolving 'here'."""
        resolved = parser.resolve_context("what is here now")  # spaces around 'here'
        assert "living_room" in resolved

    def test_resolve_here_unknown_location(self, parser):
        """Test resolving 'here' when location unknown."""
        parser = ContextAwareParser()
        resolved = parser.resolve_context("what is here now")  # spaces around 'here'
        assert "current location" in resolved

    def test_resolve_that_prefers_object(self, parser):
        """Test resolving 'that' prefers object over location."""
        resolved = parser.resolve_context("pick that up now")  # spaces around 'that'
        assert "cup" in resolved

    def test_resolve_that_fallback_to_location(self, parser):
        """Test resolving 'that' falls back to location."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="kitchen")
        resolved = parser.resolve_context("go to that place")  # spaces around 'that'
        assert "kitchen" in resolved


class TestIntentEnhancement:
    """Test intent enhancement."""

    @pytest.fixture
    def parser(self):
        """Create parser with context."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="kitchen", battery=85.0, task="idle")
        parser._user_prefs.common_locations = ["bedroom", "office", "living_room"]
        return parser

    def test_enhance_basic_intent(self, parser):
        """Test basic intent enhancement."""
        enhanced = parser.enhance_intent("QUERY", [], "where are you")

        assert enhanced["intent_type"] == "QUERY"
        assert enhanced["context"]["robot_location"] == "kitchen"
        assert enhanced["context"]["robot_battery"] == 85.0
        assert enhanced["context"]["current_task"] == "idle"

    def test_enhance_navigate_without_location(self, parser):
        """Test enhancement adds suggestions for navigate without location."""
        enhanced = parser.enhance_intent("NAVIGATE", [], "go somewhere")

        assert "suggested_locations" in enhanced
        assert enhanced["suggested_locations"] == ["bedroom", "office", "living_room"]

    def test_enhance_navigate_with_location(self, parser):
        """Test enhancement doesn't add suggestions when location present."""
        entities = [{"type": "LOCATION", "value": "kitchen"}]
        enhanced = parser.enhance_intent("NAVIGATE", entities, "go to kitchen")

        assert "suggested_locations" not in enhanced

    def test_enhance_low_battery_warning(self, parser):
        """Test low battery warning for navigation."""
        parser.update_robot_state(battery=15.0)
        enhanced = parser.enhance_intent("NAVIGATE", [], "go far away")

        assert "warnings" in enhanced
        assert any("battery" in w.lower() for w in enhanced["warnings"])

    def test_enhance_low_battery_manipulate(self, parser):
        """Test low battery warning for manipulation."""
        parser.update_robot_state(battery=10.0)
        enhanced = parser.enhance_intent("MANIPULATE", [], "pick up object")

        assert "warnings" in enhanced

    def test_enhance_no_warning_high_battery(self, parser):
        """Test no warning when battery sufficient."""
        parser.update_robot_state(battery=50.0)
        enhanced = parser.enhance_intent("NAVIGATE", [], "go somewhere")

        assert "warnings" not in enhanced

    def test_enhance_follow_up_detection(self, parser):
        """Test follow-up detection based on conversation history."""
        parser.add_conversation_turn("go to kitchen", "NAVIGATE")
        enhanced = parser.enhance_intent("QUERY", [], "what is there")

        assert enhanced["context"].get("likely_follow_up") is True


class TestContextSummary:
    """Test context summary generation."""

    def test_empty_summary(self):
        """Test summary with no context."""
        parser = ContextAwareParser()
        summary = parser.get_context_summary()

        assert summary["conversation_turns"] == 0
        assert summary["robot_location"] == "unknown"
        assert summary["robot_battery"] == 100.0
        assert summary["current_task"] == "idle"
        assert summary["known_locations"] == 0
        assert summary["detected_objects"] == 0

    def test_populated_summary(self):
        """Test summary with context."""
        parser = ContextAwareParser()
        parser.update_robot_state(location="office", battery=75.0, task="working")
        parser.update_environment(
            locations=["kitchen", "bedroom"],
            objects=["cup"],
        )
        parser.add_conversation_turn("hello", "GREETING")
        parser.add_conversation_turn("go to kitchen", "NAVIGATE")

        summary = parser.get_context_summary()

        assert summary["conversation_turns"] == 2
        assert summary["robot_location"] == "office"
        assert summary["robot_battery"] == 75.0
        assert summary["current_task"] == "working"
        assert summary["known_locations"] == 2
        assert summary["detected_objects"] == 1
