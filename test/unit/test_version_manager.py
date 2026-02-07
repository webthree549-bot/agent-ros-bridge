#!/usr/bin/env python3
"""Unit Tests for VersionManager"""

def test_version_manager_init(version_manager):
    assert version_manager._initialized is True
    assert version_manager.MOCK_MODE is True