"""OPC Journal Suite package.

This package provides a unified entry point for the OPC Journal Suite,
routing user intents to appropriate sub-skills.
"""
from .scripts.coordinate import main as coordinate
from .scripts.coordinate import detect_intent, get_skill_for_intent

__all__ = ['coordinate', 'detect_intent', 'get_skill_for_intent']
