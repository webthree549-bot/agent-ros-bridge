#!/usr/bin/env python3
"""Unit tests for Fault Recovery"""

def test_recovery_manager():
    from openclaw_ros_bridge.fault.recovery_manager import recovery_manager
    assert recovery_manager is not None