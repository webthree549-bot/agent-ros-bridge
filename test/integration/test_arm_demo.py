#!/usr/bin/env python3
"""Integration Tests for Arm Manipulation Plugin"""

def test_arm_plugin_init(arm_plugin):
    assert arm_plugin.plugin_name == "arm_manipulation"