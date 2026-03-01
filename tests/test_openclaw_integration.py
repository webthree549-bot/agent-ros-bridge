#!/usr/bin/env python3
"""OpenClaw/ClawHub Integration Test

Verifies that Agent ROS Bridge integrates properly with OpenClaw:
- Skill manifest validation
- Package import
- Basic functionality

Usage:
    python tests/test_openclaw_integration.py
"""

import os
import sys

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_skill_manifest():
    """Test that skill.yaml is valid"""
    print("Testing skill.yaml manifest...")

    skill_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "skill.yaml")
    assert os.path.exists(skill_path), f"skill.yaml not found at {skill_path}"

    with open(skill_path) as f:
        content = f.read()

    assert "name: agent-ros-bridge" in content, "Missing skill name"
    assert "version:" in content, "Missing version"
    assert "description:" in content, "Missing description"

    print("✅ skill.yaml manifest is valid")
    return True


def test_package_import():
    """Test that the package can be imported"""
    print("Testing package import...")

    try:
        from agent_ros_bridge import Bridge  # noqa: F401

        print("✅ Bridge imported successfully")

        from agent_ros_bridge import Command, Header, Message  # noqa: F401

        print("✅ Message classes imported successfully")

        from agent_ros_bridge.gateway_v2.transports.websocket import (
            WebSocketTransport,  # noqa: F401
        )

        print("✅ WebSocketTransport imported successfully")

        return True
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False


def test_fleet_import():
    """Test fleet module import"""
    print("Testing fleet module...")

    try:
        from agent_ros_bridge.fleet import FleetOrchestrator, FleetRobot  # noqa: F401

        print("✅ Fleet module imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Fleet import failed: {e}")
        return False


def test_actions_import():
    """Test actions module import"""
    print("Testing actions module...")

    try:
        from agent_ros_bridge.actions import create_action_client  # noqa: F401

        print("✅ Actions module imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Actions import failed: {e}")
        return False


def test_metrics_import():
    """Test metrics module import"""
    print("Testing metrics module...")

    try:
        from agent_ros_bridge.metrics import MetricsServer, get_metrics  # noqa: F401

        print("✅ Metrics module imported successfully")
        return True
    except ImportError as e:
        print(f"❌ Metrics import failed: {e}")
        return False


def test_basic_functionality():
    """Test basic bridge creation"""
    print("Testing basic functionality...")

    try:
        from agent_ros_bridge import Bridge

        # Create bridge instance
        bridge = Bridge()
        assert bridge is not None, "Bridge creation failed"

        print("✅ Bridge created successfully")
        return True
    except Exception as e:
        print(f"❌ Basic functionality test failed: {e}")
        return False


def main():
    """Run all integration tests"""
    print("=" * 60)
    print("🧪 OpenClaw Integration Tests")
    print("=" * 60)
    print()

    tests = [
        ("Skill Manifest", test_skill_manifest),
        ("Package Import", test_package_import),
        ("Fleet Module", test_fleet_import),
        ("Actions Module", test_actions_import),
        ("Metrics Module", test_metrics_import),
        ("Basic Functionality", test_basic_functionality),
    ]

    results = []
    for name, test_func in tests:
        print(f"\n{'=' * 60}")
        print(f"Test: {name}")
        print("=" * 60)
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"❌ Test failed with exception: {e}")
            results.append((name, False))

    # Summary
    print("\n" + "=" * 60)
    print("📊 Test Summary")
    print("=" * 60)

    passed = sum(1 for _, r in results if r)
    total = len(results)

    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")

    print()
    print(f"Result: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All OpenClaw integration tests passed!")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
