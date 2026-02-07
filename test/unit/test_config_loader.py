#!/usr/bin/env python3
"""Unit tests for Config Loader"""

def test_config_loader():
    from openclaw_ros_bridge.base.config_loader import config_loader
    assert config_loader is not None