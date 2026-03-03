#!/usr/bin/env python3
"""Pre-commit hook to check for tests with new code.

This script checks if Python files have corresponding test files.
It's a basic TDD compliance check.
"""

import subprocess
import sys
from pathlib import Path


def get_changed_python_files():
    """Get list of changed Python files in the staging area."""
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only", "--diff-filter=ACM"],
        capture_output=True,
        text=True,
    )
    files = result.stdout.strip().split("\n")
    return [f for f in files if f.endswith(".py") and not f.startswith("tests/")]


def has_test_file(source_file: str) -> bool:
    """Check if a source file has a corresponding test file."""
    # Convert source path to test path
    # agent_ros_bridge/module.py -> tests/unit/test_module.py
    parts = Path(source_file).parts

    if parts[0] == "agent_ros_bridge":
        # Check for unit test
        module_name = Path(source_file).stem
        test_file = f"tests/unit/test_{module_name}.py"
        if Path(test_file).exists():
            return True

        # Check for integration test
        test_file = f"tests/integration/test_{module_name}.py"
        if Path(test_file).exists():
            return True

    return False


def main():
    """Main entry point."""
    changed_files = get_changed_python_files()

    if not changed_files:
        print("✓ No Python files changed")
        return 0

    files_without_tests = []

    for file in changed_files:
        # Skip __init__.py, conftest.py, and test files
        if "__init__" in file or "conftest" in file or file.startswith("tests/"):
            continue

        # Skip proto files (auto-generated)
        if "_pb2.py" in file or "_pb2_grpc.py" in file:
            continue

        if not has_test_file(file):
            files_without_tests.append(file)

    if files_without_tests:
        print("⚠️  TDD Warning: The following files may need tests:")
        for file in files_without_tests:
            print(f"   - {file}")
        print("\n💡 Tip: Add tests in tests/unit/ or tests/integration/")
        print("   Use: pytest tests/unit/test_<module>.py")

        # Don't fail, just warn (can be made strict later)
        print("\n✓ Continuing (warning only)")
        return 0

    print("✓ All changed files have corresponding tests")
    return 0


if __name__ == "__main__":
    sys.exit(main())
