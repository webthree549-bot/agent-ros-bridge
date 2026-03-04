#!/usr/bin/env python3
"""Validate all examples compile and have required files.

Usage:
    python scripts/validate_examples.py

Exit codes:
    0 - All examples valid
    1 - Some examples have issues
"""

import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple


def get_examples_dir() -> Path:
    """Get examples directory."""
    return Path(__file__).parent.parent / "examples"


def validate_python_file(filepath: Path) -> Tuple[bool, str]:
    """Validate a Python file compiles."""
    try:
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(filepath)],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            return True, ""
        return False, result.stderr
    except subprocess.TimeoutExpired:
        return False, "Timeout"
    except Exception as e:
        return False, str(e)


def validate_docker_compose(filepath: Path) -> Tuple[bool, str]:
    """Validate Docker Compose file syntax."""
    try:
        result = subprocess.run(
            ["docker-compose", "-f", str(filepath), "config"],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            return True, ""
        return False, result.stderr
    except FileNotFoundError:
        return False, "docker-compose not installed"
    except subprocess.TimeoutExpired:
        return False, "Timeout"
    except Exception as e:
        return False, str(e)


def main() -> int:
    """Main validation function."""
    examples_dir = get_examples_dir()
    
    if not examples_dir.exists():
        print(f"❌ Examples directory not found: {examples_dir}")
        return 1
    
    results: Dict[str, List[Dict]] = {
        "python": [],
        "docker": [],
        "readme": []
    }
    
    print("=" * 60)
    print("🔍 Validating Agent ROS Bridge Examples")
    print("=" * 60)
    
    # Find all Python files
    python_files = list(examples_dir.rglob("*.py"))
    print(f"\n📁 Found {len(python_files)} Python files")
    
    for py_file in sorted(python_files):
        valid, error = validate_python_file(py_file)
        rel_path = py_file.relative_to(examples_dir)
        results["python"].append({
            "file": str(rel_path),
            "valid": valid,
            "error": error
        })
        status = "✅" if valid else "❌"
        print(f"  {status} {rel_path}")
    
    # Find all Docker Compose files
    compose_files = list(examples_dir.rglob("docker-compose.yml"))
    print(f"\n🐳 Found {len(compose_files)} Docker Compose files")
    
    for compose_file in sorted(compose_files):
        valid, error = validate_docker_compose(compose_file)
        rel_path = compose_file.relative_to(examples_dir)
        results["docker"].append({
            "file": str(rel_path),
            "valid": valid,
            "error": error
        })
        status = "✅" if valid else "❌"
        print(f"  {status} {rel_path}")
    
    # Check for README files
    readme_files = list(examples_dir.rglob("README.md"))
    print(f"\n📖 Found {len(readme_files)} README files")
    
    for readme in sorted(readme_files):
        rel_path = readme.relative_to(examples_dir)
        results["readme"].append({
            "file": str(rel_path),
            "valid": True,
            "error": ""
        })
        print(f"  ✅ {rel_path}")
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 Validation Summary")
    print("=" * 60)
    
    python_valid = sum(1 for r in results["python"] if r["valid"])
    python_total = len(results["python"])
    print(f"\nPython Files: {python_valid}/{python_total} valid")
    
    docker_valid = sum(1 for r in results["docker"] if r["valid"])
    docker_total = len(results["docker"])
    print(f"Docker Compose: {docker_valid}/{docker_total} valid")
    
    readme_total = len(results["readme"])
    print(f"README Files: {readme_total} found")
    
    # Errors
    errors = []
    for category in ["python", "docker"]:
        for result in results[category]:
            if not result["valid"]:
                errors.append(f"{result['file']}: {result['error']}")
    
    if errors:
        print("\n❌ Errors Found:")
        for error in errors:
            print(f"  - {error}")
        return 1
    
    print("\n✅ All examples validated successfully!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
