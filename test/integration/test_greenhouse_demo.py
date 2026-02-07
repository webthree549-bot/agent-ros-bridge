#!/usr/bin/env python3
"""Integration Tests for Greenhouse Plugin"""

def test_greenhouse_plugin_init(greenhouse_plugin):
    assert greenhouse_plugin.plugin_name == "greenhouse"