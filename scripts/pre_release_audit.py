#!/usr/bin/env python3
"""Pre-release audit for version 0.6.0

Comprehensive audit before release.
"""

import subprocess
import sys
from pathlib import Path


def run_command(cmd, description):
    """Run a command and report status."""
    print(f"\n{'='*60}")
    print(f"🔍 {description}")
    print(f"{'='*60}")
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"✅ {description} PASSED")
        if result.stdout:
            print(result.stdout[:500])  # Limit output
        return True
    else:
        print(f"❌ {description} FAILED")
        print(result.stderr[:500])
        return False


def main():
    """Run all pre-release checks."""
    print("\n" + "="*60)
    print("🚀 AGENT ROS BRIDGE v0.6.0 PRE-RELEASE AUDIT")
    print("="*60)
    
    checks = [
        # Security
        ("python3 scripts/security_audit.py", "Security Audit"),
        
        # Code Quality
        ("python3 -m ruff check agent_ros_bridge/ --output-format=concise", "Ruff Linting"),
        ("python3 -m black --check agent_ros_bridge/", "Black Formatting"),
        
        # Tests
        ("python3 -m pytest tests/unit -v --tb=short -q", "Unit Tests"),
        ("python3 -m pytest tests/skills -v --tb=short -q", "Skill Tests"),
        
        # Type Checking
        ("python3 -m mypy agent_ros_bridge/ --ignore-missing-imports", "Type Checking"),
        
        # Dependencies
        ("python3 -c 'import agent_ros_bridge'", "Import Test"),
        
        # Documentation
        ("test -f README.md && test -f docs/TROUBLESHOOTING.md", "Documentation Check"),
        
        # Configuration
        ("test -f pyproject.toml && test -f config/bridge.yaml", "Configuration Files"),
        
        # Docker
        ("test -f Dockerfile && test -f docker-compose.yml", "Docker Files"),
        
        # Helm
        ("test -f helm/agent-ros-bridge/Chart.yaml", "Helm Chart"),
    ]
    
    results = []
    for cmd, desc in checks:
        results.append((desc, run_command(cmd, desc)))
    
    # Summary
    print("\n" + "="*60)
    print("📊 AUDIT SUMMARY")
    print("="*60)
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    
    for desc, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {desc}")
    
    print(f"\nTotal: {passed}/{total} checks passed")
    
    if passed == total:
        print("\n🎉 ALL CHECKS PASSED!")
        print("Ready to release v0.6.0")
        return 0
    else:
        print(f"\n⚠️  {total - passed} checks failed")
        print("Please fix issues before releasing")
        return 1


if __name__ == "__main__":
    sys.exit(main())
