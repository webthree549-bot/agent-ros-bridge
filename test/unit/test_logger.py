#!/usr/bin/env python3
"""Unit tests for Logger"""

def test_get_logger():
    from openclaw_ros_bridge.base.logger import get_logger
    logger = get_logger("test")
    assert logger is not None