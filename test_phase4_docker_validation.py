#!/usr/bin/env python3
"""
Phase 4 Testing: Docker Infrastructure Validation
Tests the standardized Jazzy Docker setup end-to-end.
"""

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent

def run_command(cmd, check=True):
    """Run shell command and return result."""
    result = subprocess.run(
        cmd,
        shell=True,
        capture_output=True,
        text=True,
        cwd=PROJECT_ROOT
    )
    if check and result.returncode != 0:
        print(f"❌ Command failed: {cmd}")
        print(f"   Error: {result.stderr}")
        return None
    return result

def test_docker_files_exist():
    """Test 1: All required Docker files exist."""
    print("\n" + "=" * 60)
    print("Test 1: Docker Files Exist")
    print("=" * 60)
    
    required_files = [
        "docker/Dockerfile.ros2.jazzy",
        "docker/Dockerfile.ros2.humble",  # Legacy kept
        "docker/Dockerfile.simulation",
        "scripts/docker/docker-manager.sh",
        "scripts/docker/build-ros2-image.sh",
    ]
    
    all_exist = True
    for file in required_files:
        path = PROJECT_ROOT / file
        if path.exists():
            print(f"✅ {file}")
        else:
            print(f"❌ {file} - MISSING")
            all_exist = False
    
    return all_exist

def test_scripts_executable():
    """Test 2: Scripts are executable."""
    print("\n" + "=" * 60)
    print("Test 2: Scripts Executable")
    print("=" * 60)
    
    scripts = [
        "scripts/docker/docker-manager.sh",
        "scripts/docker/build-ros2-image.sh",
    ]
    
    all_executable = True
    for script in scripts:
        path = PROJECT_ROOT / script
        if path.exists() and path.stat().st_mode & 0o111:
            print(f"✅ {script}")
        else:
            print(f"❌ {script} - Not executable")
            all_executable = False
    
    return all_executable

def test_no_humble_references():
    """Test 3: Critical files don't reference humble."""
    print("\n" + "=" * 60)
    print("Test 3: No Humble References in New Scripts")
    print("=" * 60)
    
    files_to_check = [
        "scripts/docker/docker-manager.sh",
        "scripts/docker/build-ros2-image.sh",
        "docker/Dockerfile.simulation",
    ]
    
    all_clean = True
    for file in files_to_check:
        path = PROJECT_ROOT / file
        if not path.exists():
            print(f"⚠️  {file} - File not found")
            continue
        
        content = path.read_text()
        # Check for old naming (case insensitive)
        humble_refs = []
        if 'ros2_humble' in content.lower():
            humble_refs.append('ros2_humble')
        if 'ros-humble-' in content.lower():
            humble_refs.append('ros-humble-*')
        if 'humble-desktop' in content.lower():
            humble_refs.append('humble-desktop')
        
        if humble_refs:
            print(f"❌ {file} - Contains: {', '.join(humble_refs)}")
            all_clean = False
        else:
            print(f"✅ {file} - No Humble references")
    
    return all_clean

def test_jazzy_references():
    """Test 4: Files correctly reference Jazzy."""
    print("\n" + "=" * 60)
    print("Test 4: Jazzy References Present")
    print("=" * 60)
    
    files_to_check = [
        ("scripts/docker/docker-manager.sh", ["ros2_jazzy", "jazzy-desktop"]),
        ("scripts/docker/build-ros2-image.sh", ["ros2-jazzy", "jazzy-ros-base"]),
        ("docker/Dockerfile.simulation", ["jazzy-desktop-full", "ros-jazzy-"]),
    ]
    
    all_present = True
    for file, refs in files_to_check:
        path = PROJECT_ROOT / file
        if not path.exists():
            print(f"⚠️  {file} - File not found")
            continue
        
        content = path.read_text().lower()
        found_all = True
        for ref in refs:
            if ref.lower() not in content:
                print(f"❌ {file} - Missing: {ref}")
                found_all = False
                all_present = False
        
        if found_all:
            print(f"✅ {file} - All Jazzy references present")
    
    return all_present

def test_documentation_updated():
    """Test 5: Key documentation files updated."""
    print("\n" + "=" * 60)
    print("Test 5: Documentation Updated")
    print("=" * 60)
    
    docs_to_check = [
        ("docs/MIGRATION_HUMBLE_TO_JAZZY.md", True),  # Should exist
        ("docs/DOCKER_RESEARCH_AND_PLAN.md", True),   # Should exist
        ("DOCKER_FIX_SUMMARY.md", True),              # Should exist
    ]
    
    all_exist = True
    for doc, should_exist in docs_to_check:
        path = PROJECT_ROOT / doc
        if should_exist and path.exists():
            print(f"✅ {doc}")
        elif should_exist and not path.exists():
            print(f"❌ {doc} - Missing")
            all_exist = False
        elif not should_exist and path.exists():
            print(f"⚠️  {doc} - Should be removed")
    
    return all_exist

def test_deprecated_files():
    """Test 6: Old files properly deprecated."""
    print("\n" + "=" * 60)
    print("Test 6: Deprecated Files")
    print("=" * 60)
    
    deprecated = [
        "scripts/docker/start-ros2.sh.deprecated",
    ]
    
    for file in deprecated:
        path = PROJECT_ROOT / file
        if path.exists():
            print(f"✅ {file} - Marked deprecated")
        else:
            print(f"⚠️  {file} - Not found (may be removed)")
    
    # Check that start-ros2.sh is deprecated
    if (PROJECT_ROOT / "scripts/docker/start-ros2.sh").exists():
        print("⚠️  start-ros2.sh still exists (should be deprecated)")
        return False
    
    return True

def test_docker_manager_wrapper():
    """Test 7: Root docker-manager.sh redirects to new location."""
    print("\n" + "=" * 60)
    print("Test 7: Docker Manager Wrapper")
    print("=" * 60)
    
    wrapper = PROJECT_ROOT / "docker-manager.sh"
    if not wrapper.exists():
        print("❌ Root docker-manager.sh missing")
        return False
    
    content = wrapper.read_text()
    
    checks = [
        ("DEPRECATED" in content, "Deprecation warning"),
        ("scripts/docker/docker-manager.sh" in content, "Redirects to new location"),
        ("$@" in content, "Passes arguments through"),
    ]
    
    all_good = True
    for check, desc in checks:
        if check:
            print(f"✅ {desc}")
        else:
            print(f"❌ {desc}")
            all_good = False
    
    return all_good

def main():
    print("\n" + "=" * 60)
    print("Phase 4: Docker Infrastructure Validation")
    print("=" * 60)
    print()
    
    tests = [
        ("Docker Files Exist", test_docker_files_exist),
        ("Scripts Executable", test_scripts_executable),
        ("No Humble References", test_no_humble_references),
        ("Jazzy References Present", test_jazzy_references),
        ("Documentation Updated", test_documentation_updated),
        ("Deprecated Files", test_deprecated_files),
        ("Docker Manager Wrapper", test_docker_manager_wrapper),
    ]
    
    results = []
    for name, test_fn in tests:
        try:
            result = test_fn()
            results.append((name, result))
        except Exception as e:
            print(f"\n❌ {name} - Exception: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {name}")
    
    print()
    print(f"Result: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All Phase 4 tests passed!")
        print("Docker infrastructure is properly standardized on Jazzy.")
        return 0
    else:
        print("\n⚠️  Some tests failed. Review output above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
