#!/usr/bin/env python3
"""Service types for agent_ros_bridge_msgs.

This module provides Python implementations of ROS service types.
"""
from dataclasses import dataclass, field
from typing import List

from agent_ros_bridge_msgs import Intent


@dataclass
class ContextQuery:
    """Context query for intent parsing."""
    key: str = ""
    value: str = ""


@dataclass
class ParseIntentRequest:
    """Request for ParseIntent service."""
    utterance: str = ""
    robot_id: str = ""
    context: List[ContextQuery] = field(default_factory=list)
    session_id: str = ""
    language: str = "en"


@dataclass
class ParseIntentResponse:
    """Response for ParseIntent service."""
    intent: Intent = field(default_factory=Intent)
    success: bool = False
    error_message: str = ""
    suggestions: List[str] = field(default_factory=list)
    latency_ms: float = 0.0


# Service class for type checking
class ParseIntent:
    """ParseIntent service type."""
    Request = ParseIntentRequest
    Response = ParseIntentResponse
