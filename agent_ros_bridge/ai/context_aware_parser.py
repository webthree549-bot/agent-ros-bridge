#!/usr/bin/env python3
"""
Context-Aware Intent Parser
Agent ROS Bridge v0.6.1 - Week 6 Advanced Features

Enhances intent parsing with contextual awareness:
- Conversation history
- Robot state (location, battery, current task)
- Environment state (objects, locations, people)
- User preferences and patterns
"""

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ConversationContext:
    """Context from conversation history."""

    utterances: list[str] = field(default_factory=list)
    intents: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)
    max_history: int = 10

    def add_turn(self, utterance: str, intent: str):
        """Add a conversation turn."""
        self.utterances.append(utterance)
        self.intents.append(intent)
        # Keep only last max_history turns
        if len(self.utterances) > self.max_history:
            self.utterances.pop(0)
            self.intents.pop(0)


@dataclass
class RobotState:
    """Current robot state."""

    location: str = "unknown"
    battery_level: float = 100.0
    current_task: str = "idle"
    is_moving: bool = False
    last_updated: float = field(default_factory=time.time)


@dataclass
class EnvironmentState:
    """Current environment state."""

    known_locations: list[str] = field(default_factory=list)
    detected_objects: list[str] = field(default_factory=list)
    people_present: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass
class UserPreferences:
    """User-specific preferences."""

    preferred_speed: str = "normal"
    preferred_language: str = "en"
    common_locations: list[str] = field(default_factory=list)
    last_updated: float = field(default_factory=time.time)


class ContextAwareParser:
    """
    Context-aware intent parser.

    Uses conversation history, robot state, and environment to
    improve intent parsing accuracy for ambiguous utterances.
    """

    def __init__(self, max_history: int = 10):
        """
        Initialize context-aware parser.

        Args:
            max_history: Maximum conversation history to maintain
        """
        self._max_history = max_history
        self._conversation = ConversationContext(max_history=max_history)
        self._robot_state = RobotState()
        self._environment = EnvironmentState()
        self._user_prefs = UserPreferences()

        # Context resolution rules
        self._pronoun_replacements = {
            "it": self._resolve_it,
            "there": self._resolve_there,
            "here": self._resolve_here,
            "that": self._resolve_that,
        }

    def update_robot_state(
        self,
        location: str | None = None,
        battery: float | None = None,
        task: str | None = None,
        moving: bool | None = None,
    ):
        """Update robot state."""
        if location:
            self._robot_state.location = location
        if battery is not None:
            self._robot_state.battery_level = battery
        if task:
            self._robot_state.current_task = task
        if moving is not None:
            self._robot_state.is_moving = moving
        self._robot_state.last_updated = time.time()

    def update_environment(
        self,
        locations: list[str] | None = None,
        objects: list[str] | None = None,
        people: list[str] | None = None,
    ):
        """Update environment state."""
        if locations:
            self._environment.known_locations = locations
        if objects:
            self._environment.detected_objects = objects
        if people:
            self._environment.people_present = people
        self._environment.timestamp = time.time()

    def add_conversation_turn(self, utterance: str, intent: str):
        """Add a conversation turn to history."""
        self._conversation.add_turn(utterance, intent)

    def resolve_context(self, utterance: str) -> str:
        """
        Resolve contextual references in utterance.

        Args:
            utterance: Input with potential contextual references

        Returns:
            Utterance with resolved references
        """
        resolved = utterance.lower()

        # Resolve pronouns
        for pronoun, resolver in self._pronoun_replacements.items():
            if f" {pronoun} " in f" {resolved} ":
                replacement = resolver()
                if replacement:
                    resolved = resolved.replace(f" {pronoun} ", f" {replacement} ")

        return resolved

    def _resolve_it(self) -> str | None:
        """Resolve 'it' to last mentioned object."""
        # Look for objects in recent conversation
        for utterance in reversed(self._conversation.utterances):
            # Simple heuristic: look for "the X" patterns
            words = utterance.lower().split()
            for i, word in enumerate(words):
                if word == "the" and i + 1 < len(words):
                    obj = words[i + 1]
                    if obj not in ["kitchen", "room", "office"]:  # Not a location
                        return obj

        # Fall back to environment objects
        if self._environment.detected_objects:
            return self._environment.detected_objects[0]

        return None

    def _resolve_there(self) -> str | None:
        """Resolve 'there' to a location."""
        # Use last mentioned location or current robot location
        for utterance in reversed(self._conversation.utterances):
            if "to" in utterance.lower():
                # Extract location after "to"
                parts = utterance.lower().split("to")
                if len(parts) > 1:
                    location = parts[1].strip().split()[0]
                    return location

        return self._robot_state.location if self._robot_state.location != "unknown" else None

    def _resolve_here(self) -> str | None:
        """Resolve 'here' to current location."""
        return (
            self._robot_state.location
            if self._robot_state.location != "unknown"
            else "current location"
        )

    def _resolve_that(self) -> str | None:
        """Resolve 'that' to last mentioned object or location."""
        # Try object first
        obj = self._resolve_it()
        if obj:
            return obj

        # Fall back to location
        return self._resolve_there()

    def enhance_intent(
        self, intent_type: str, entities: list[dict[str, Any]], utterance: str
    ) -> dict[str, Any]:
        """
        Enhance parsed intent with contextual information.

        Args:
            intent_type: Base intent type
            entities: Extracted entities
            utterance: Original utterance

        Returns:
            Enhanced intent with context
        """
        enhanced = {
            "intent_type": intent_type,
            "entities": entities,
            "context": {
                "robot_location": self._robot_state.location,
                "robot_battery": self._robot_state.battery_level,
                "current_task": self._robot_state.current_task,
                "conversation_turns": len(self._conversation.utterances),
            },
        }

        # Add location context for navigation
        if intent_type == "NAVIGATE":
            # If no location specified, suggest common locations
            if not any(e.get("type") == "LOCATION" for e in entities):
                enhanced["suggested_locations"] = self._user_prefs.common_locations[:3]

        # Add battery warning for long tasks
        if intent_type in ["NAVIGATE", "MANIPULATE"]:
            if self._robot_state.battery_level < 20:
                enhanced["warnings"] = ["Low battery - consider charging first"]

        # Add follow-up suggestions based on conversation history
        if self._conversation.intents:
            last_intent = self._conversation.intents[-1]
            if last_intent == "NAVIGATE" and intent_type == "QUERY":
                enhanced["context"]["likely_follow_up"] = True

        return enhanced

    def get_context_summary(self) -> dict[str, Any]:
        """Get summary of current context."""
        return {
            "conversation_turns": len(self._conversation.utterances),
            "robot_location": self._robot_state.location,
            "robot_battery": self._robot_state.battery_level,
            "current_task": self._robot_state.current_task,
            "known_locations": len(self._environment.known_locations),
            "detected_objects": len(self._environment.detected_objects),
        }


def main():
    """Test context-aware parser."""
    parser = ContextAwareParser()

    # Set up context
    parser.update_robot_state(location="living_room", battery=85.0, task="idle")
    parser.update_environment(
        locations=["kitchen", "bedroom", "office", "living_room"],
        objects=["cup", "book", "phone"],
        people=["user"],
    )

    # Add conversation history
    parser.add_conversation_turn("go to the kitchen", "NAVIGATE")
    parser.add_conversation_turn("pick up the cup", "MANIPULATE")

    # Test context resolution
    test_utterances = [
        "place it on the table",  # "it" should resolve to "cup"
        "go there",  # "there" should resolve to "kitchen"
        "what is here",  # "here" should resolve to "living_room"
    ]

    print("Context-Aware Intent Parsing\n")
    print(f"Context: {parser.get_context_summary()}\n")

    for utterance in test_utterances:
        resolved = parser.resolve_context(utterance)
        print(f"Original: '{utterance}'")
        print(f"Resolved: '{resolved}'")
        print()


if __name__ == "__main__":
    main()
