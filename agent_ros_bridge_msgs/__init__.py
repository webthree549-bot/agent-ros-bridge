#!/usr/bin/env python3
"""Python stubs for agent_ros_bridge_msgs.

This module provides Python classes for ROS messages that can be imported
without requiring a full ROS build. Used for testing and development.
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class Entity:
    """Entity message for agent_ros_bridge.
    
    Represents an extracted entity from user utterance.
    """
    # Entity type (LOCATION, OBJECT, ACTION, NUMBER, etc.)
    type: str = ""
    
    # Entity value
    value: str = ""
    
    # Confidence score (0.0 to 1.0)
    confidence: float = 0.0
    
    # Start position in original utterance
    start_pos: int = 0
    
    # End position in original utterance
    end_pos: int = 0


@dataclass
class Intent:
    """Intent message for agent_ros_bridge.
    
    Represents a parsed user intent.
    """
    # Intent type (NAVIGATE, MANIPULATE, SAFETY, PERCEIVE, CONFIGURE, etc.)
    type: str = ""
    
    # Confidence score (0.0 to 1.0)
    confidence: float = 0.0
    
    # Original utterance
    utterance: str = ""
    
    # Extracted entities
    entities: List[Entity] = field(default_factory=list)
    
    # Additional parameters as key-value pairs
    param_keys: List[str] = field(default_factory=list)
    param_values: List[str] = field(default_factory=list)
