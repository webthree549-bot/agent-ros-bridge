#!/usr/bin/env python3
"""Unit tests for Data Converter"""

def test_data_converter():
    from openclaw_ros_bridge.converter.data_converter import data_converter
    assert data_converter is not None