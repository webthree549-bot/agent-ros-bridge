"""Conversation manager with context retention."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ConversationManager:
    """Manages multi-turn conversations with context retention."""

    context: dict[str, Any] = field(default_factory=dict)
    history: list[dict[str, Any]] = field(default_factory=list)

    def process_input(self, user_input: str) -> dict[str, Any]:
        """Process user input with context awareness.

        Returns:
            Result with resolved references and intent
        """
        result = {
            "input": user_input,
            "intent": None,
            "resolved_references": [],
            "needs_clarification": False,
        }

        # Check for pronoun references
        if "it" in user_input.lower() or "there" in user_input.lower():
            # Resolve reference from context
            last_location = self.context.get("last_location")
            if last_location:
                result["resolved_references"].append(last_location)
            else:
                result["needs_clarification"] = True
                result["clarification_question"] = "What location are you referring to?"

        # Check for corrections
        if any(word in user_input.lower() for word in ["actually", "instead", "no"]):
            result["intent"] = "correction"
            # Extract new target
            result["new_target"] = self._extract_target(user_input)

        # Store in history
        self.history.append(
            {
                "input": user_input,
                "timestamp": "now",
                "result": result,
            }
        )

        # Update context
        if "go to" in user_input.lower() or "navigate" in user_input.lower():
            location = self._extract_target(user_input)
            if location:
                self.context["last_location"] = location

        return result

    def _extract_target(self, text: str) -> str | None:
        """Extract target location from text."""
        # Simple extraction - in real implementation, use NLP
        locations = [
            "kitchen",
            "living room",
            "bedroom",
            "office",
            "conference room",
            "garage",
            "basement",
        ]

        text_lower = text.lower()
        for loc in locations:
            if loc in text_lower:
                return loc

        return None

    def get_contextual_suggestions(self, context: dict[str, Any]) -> list[str]:
        """Get suggestions based on current context."""
        suggestions = []

        # Low battery suggestion
        battery = context.get("battery", 100)
        if battery < 20:
            suggestions.append(f"Battery at {battery}%. Consider charging.")
        elif battery < 50:
            suggestions.append("Battery is getting low. You may want to charge soon.")

        # Location-based suggestions
        location = context.get("current_location")
        if location == "kitchen":
            suggestions.append("Would you like to check the refrigerator?")

        # General suggestions
        suggestions.append("I can help you navigate, check status, or pick up objects.")

        return suggestions
