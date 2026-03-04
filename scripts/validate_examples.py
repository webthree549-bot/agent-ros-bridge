#!/usr/bin/env python3
"""Validate all examples compile and have required files.

Includes TDD compliance checks per project standards.

Usage:
    python scripts/validate_examples.py [--tdd]

Exit codes:
    0 - All examples valid
    1 - Some examples have issues
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional


def get_examples_dir() -> Path:
    """Get examples directory."""
    return Path(__file__).parent.parent / "examples"


def get_tests_dir() -> Path:
    """Get tests directory."""
    return Path(__file__).parent.parent / "tests"


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


def check_tdd_compliance(example_path: Path) -> Tuple[bool, List[str]]:
    """Check if example has corresponding tests (TDD compliance).
    
    Per TDD workflow, every feature/example should have tests.
    Examples are documentation, so they don't need full test coverage,
    but critical examples should have integration tests.
    
    Returns:
        (is_compliant, list_of_issues)
    """
    issues = []
    
    # Get example name
    example_name = example_path.stem  # e.g., "langchain_example"
    
    # Check if there's a corresponding test
    tests_dir = get_tests_dir()
    
    # Look for test files that might test this example
    possible_test_names = [
        f"test_{example_name}.py",
        f"test_{example_name.replace('_example', '')}.py",
        f"test_{example_path.parent.name}.py",
    ]
    
    test_exists = False
    for test_name in possible_test_names:
        if (tests_dir / "integration" / test_name).exists():
            test_exists = True
            break
        if (tests_dir / "e2e" / test_name).exists():
            test_exists = True
            break
    
    # Examples don't strictly require tests, but we warn if missing
    # for complex examples
    is_complex = any(x in example_name for x in ["langchain", "autogpt", "mcp", "fleet", "grpc"])
    
    if is_complex and not test_exists:
        issues.append(f"Complex example '{example_name}' lacks integration test")
    
    return len(issues) == 0, issues


def check_example_structure(example_dir: Path) -> Tuple[bool, List[str]]:
    """Check if example directory follows TDD structure.
    
    Structure should be:
    - README.md (what it does)
    - docker-compose.yml (how to run)
    - *.py (the example)
    - requirements.txt (optional, dependencies)
    """
    issues = []
    
    # Check for README
    if not (example_dir / "README.md").exists():
        issues.append("Missing README.md")
    
    # Check for docker-compose or requirements
    has_docker = (example_dir / "docker-compose.yml").exists()
    has_requirements = (example_dir / "requirements.txt").exists()
    
    if not has_docker and not has_requirements:
        issues.append("Missing docker-compose.yml or requirements.txt")
    
    # Check for Python files
    py_files = list(example_dir.glob("*.py"))
    if not py_files:
        issues.append("No Python files found")
    
    return len(issues) == 0, issues


def main() -> int:
    """Main validation function."""
    parser = argparse.ArgumentParser(description="Validate Agent ROS Bridge examples")
    parser.add_argument("--tdd", action="store_true", help="Enable TDD compliance checks")
    args = parser.parse_args()
    
    examples_dir = get_examples_dir()
    
    if not examples_dir.exists():
        print(f"❌ Examples directory not found: {examples_dir}")
        return 1
    
    results: Dict[str, List[Dict]] = {
        "python": [],
        "docker": [],
        "readme": [],
        "tdd": []
    }
    
    print("=" * 60)
    print("🔍 Validating Agent ROS Bridge Examples")
    if args.tdd:
        print("📋 TDD Compliance Mode Enabled")
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
    
    # TDD Compliance Checks
    if args.tdd:
        print(f"\n🧪 TDD Compliance Checks")
        
        # Check example directories
        for subdir in examples_dir.iterdir():
            if subdir.is_dir() and not subdir.name.startswith(".") and subdir.name != "__pycache__":
                valid, issues = check_example_structure(subdir)
                rel_path = subdir.relative_to(examples_dir)
                
                # Check for tests on Python files
                for py_file in subdir.glob("*.py"):
                    tdd_valid, tdd_issues = check_tdd_compliance(py_file)
                    issues.extend(tdd_issues)
                
                results["tdd"].append({
                    "dir": str(rel_path),
                    "valid": valid and len(issues) == 0,
                    "issues": issues
                })
                
                if issues:
                    print(f"  ⚠️  {rel_path}")
                    for issue in issues:
                        print(f"      - {issue}")
                else:
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
    
    if args.tdd and results["tdd"]:
        tdd_valid = sum(1 for r in results["tdd"] if r["valid"])
        tdd_total = len(results["tdd"])
        print(f"TDD Compliance: {tdd_valid}/{tdd_total} compliant")
    
    # Errors
    errors = []
    for category in ["python", "docker"]:
        for result in results[category]:
            if not result["valid"]:
                errors.append(f"{result['file']}: {result['error']}")
    
    if args.tdd:
        for result in results["tdd"]:
            if not result["valid"]:
                for issue in result.get("issues", []):
                    errors.append(f"{result['dir']}: {issue}")
    
    if errors:
        print("\n❌ Errors Found:")
        for error in errors:
            print(f"  - {error}")
        return 1
    
    print("\n✅ All examples validated successfully!")
    if args.tdd:
        print("✅ TDD compliance verified!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
