#!/usr/bin/env python3
"""Native ROS Setup Validation Script.

Validates that Agent ROS Bridge works correctly with native ROS installation.
Run this before production deployment to verify your environment.

Usage:
    python scripts/validate_ros_setup.py [--ros2] [--ros1]

Examples:
    # Auto-detect ROS version
    python scripts/validate_ros_setup.py

    # Validate specific version
    python scripts/validate_ros_setup.py --ros2
    python scripts/validate_ros_setup.py --ros1

    # Validate both
    python scripts/validate_ros_setup.py --ros2 --ros1
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

# Colors for terminal output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"


def print_header(text):
    print(f"\n{BLUE}{'=' * 60}{RESET}")
    print(f"{BLUE}{text}{RESET}")
    print(f"{BLUE}{'=' * 60}{RESET}")


def print_success(text):
    print(f"{GREEN}✅{RESET} {text}")


def print_error(text):
    print(f"{RED}❌{RESET} {text}")


def print_warning(text):
    print(f"{YELLOW}⚠️{RESET} {text}")


def print_info(text):
    print(f"{BLUE}ℹ️{RESET} {text}")


def run_command(cmd, capture_output=True):
    """Run shell command and return (success, output)."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=capture_output, text=True, timeout=10
        )
        return result.returncode == 0, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return False, "Command timed out"
    except Exception as e:
        return False, str(e)


def check_ros2():
    """Validate ROS2 installation."""
    print_header("Checking ROS2 Installation")

    results = {"passed": 0, "failed": 0, "warnings": 0}

    # Check ROS2 is sourced
    if "ROS_DISTRO" in os.environ:
        distro = os.environ["ROS_DISTRO"]
        print_success(f"ROS_DISTRO set: {distro}")
        results["passed"] += 1
    else:
        print_error("ROS_DISTRO not set — source your ROS2 setup.bash")
        results["failed"] += 1
        return results

    # Check ROS2 installation directory
    ros2_path = f"/opt/ros/{distro}"
    if os.path.exists(ros2_path):
        print_success(f"ROS2 installation found: {ros2_path}")
        results["passed"] += 1
    else:
        print_error(f"ROS2 installation not found at {ros2_path}")
        results["failed"] += 1

    # Check ros2 command
    success, output = run_command("which ros2")
    if success:
        print_success(f"ros2 CLI available: {output.strip()}")
        results["passed"] += 1
    else:
        print_error("ros2 CLI not found — check PATH")
        results["failed"] += 1

    # Check rclpy
    print_info("Checking rclpy Python module...")
    success, output = run_command("python3 -c 'import rclpy; print(rclpy.__file__)'")
    if success:
        print_success(f"rclpy available: {output.strip()}")
        results["passed"] += 1
    else:
        print_error("rclpy not available — install ros-humble-rclpy")
        results["failed"] += 1

    # Check common ROS2 packages
    packages = ["std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs"]
    for pkg in packages:
        success, _ = run_command(f"ros2 pkg list | grep {pkg}", capture_output=True)
        if success:
            print_success(f"Package {pkg} installed")
            results["passed"] += 1
        else:
            print_warning(f"Package {pkg} not found — install ros-{distro}-{pkg.replace('_', '-')}")
            results["warnings"] += 1

    return results


def check_ros1():
    """Validate ROS1 installation."""
    print_header("Checking ROS1 Installation")

    results = {"passed": 0, "failed": 0, "warnings": 0}

    # Check ROS1 environment
    if "ROS_MASTER_URI" in os.environ:
        print_success(f"ROS_MASTER_URI: {os.environ['ROS_MASTER_URI']}")
        results["passed"] += 1
    else:
        print_warning("ROS_MASTER_URI not set — may need to source setup.bash")
        results["warnings"] += 1

    # Check roscore
    success, output = run_command("which roscore")
    if success:
        print_success(f"roscore available: {output.strip()}")
        results["passed"] += 1
    else:
        print_error("roscore not found — ROS1 not installed or not in PATH")
        results["failed"] += 1

    # Check rospy
    print_info("Checking rospy Python module...")
    success, output = run_command("python3 -c 'import rospy; print(rospy.__file__)'")
    if success:
        print_success(f"rospy available: {output.strip()}")
        results["passed"] += 1
    else:
        print_error("rospy not available — install ros-noetic-desktop")
        results["failed"] += 1

    return results


def check_agent_ros_bridge():
    """Validate Agent ROS Bridge installation."""
    print_header("Checking Agent ROS Bridge")

    results = {"passed": 0, "failed": 0, "warnings": 0}

    # Check if installed via pip
    success, output = run_command("pip3 show agent-ros-bridge")
    if success:
        print_success("agent-ros-bridge installed via pip")
        # Extract version
        for line in output.split("\n"):
            if line.startswith("Version:"):
                print_info(f"Version: {line.split(':')[1].strip()}")
        results["passed"] += 1
    else:
        # Check if running from source
        project_dir = Path(__file__).parent.parent
        if (project_dir / "agent_ros_bridge").exists():
            print_success(f"Running from source: {project_dir}")
            results["passed"] += 1
        else:
            print_error("agent-ros-bridge not found — install with 'pip install agent-ros-bridge'")
            results["failed"] += 1
            return results

    # Test imports using temporary file to avoid shell escaping issues
    print_info("Testing core imports...")
    import tempfile

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as f:
        f.write("from agent_ros_bridge import Bridge, Message, Header, Command\n")
        f.write("from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport\n")
        f.write("print('Core imports: OK')\n")
        test_file = f.name

    success, output = run_command(f"python3 {test_file}")
    os.unlink(test_file)

    if success:
        print_success("Core bridge imports work")
        results["passed"] += 1
    else:
        print_error(f"Import failed: {output}")
        results["failed"] += 1

    # Test ROS connector imports
    print_info("Testing ROS connector imports...")

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as f:
        f.write("try:\n")
        f.write(
            "    from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector\n"
        )
        f.write("    print('ROS2 Connector: OK')\n")
        f.write("except ImportError as e:\n")
        f.write("    print(f'ROS2 Connector Error: {e}')\n")
        test_file = f.name

    success, output = run_command(f"python3 {test_file}")
    os.unlink(test_file)

    if success and "OK" in output:
        print_success("ROS2 connector imports work")
        results["passed"] += 1
    else:
        print_warning("ROS2 connector import issue (expected if ROS2 not installed)")
        results["warnings"] += 1

    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as f:
        f.write("try:\n")
        f.write(
            "    from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector\n"
        )
        f.write("    print('ROS1 Connector: OK')\n")
        f.write("except ImportError as e:\n")
        f.write("    print(f'ROS1 Connector Error: {e}')\n")
        test_file = f.name

    success, output = run_command(f"python3 {test_file}")
    os.unlink(test_file)

    if success and "OK" in output:
        print_success("ROS1 connector imports work")
        results["passed"] += 1
    else:
        print_warning("ROS1 connector import issue (expected if ROS1 not installed)")
        results["warnings"] += 1

    return results


def test_basic_functionality():
    """Test basic bridge functionality."""
    print_header("Testing Basic Functionality")

    results = {"passed": 0, "failed": 0, "warnings": 0}
    import tempfile

    # Test bridge instantiation
    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as f:
        f.write("from agent_ros_bridge import Bridge\n")
        f.write("bridge = Bridge()\n")
        f.write("print(f'Bridge created: {bridge}')\n")
        f.write("print('Bridge instantiation: OK')\n")
        test_file = f.name

    success, output = run_command(f"python3 {test_file}")
    os.unlink(test_file)

    if success:
        print_success("Bridge instantiation works")
        results["passed"] += 1
    else:
        print_error(f"Bridge instantiation failed: {output}")
        results["failed"] += 1

    # Test message creation
    with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as f:
        f.write("from agent_ros_bridge import Message, Header, Command\n")
        f.write("import json\n")
        f.write("msg = Message(\n")
        f.write("    header=Header(),\n")
        f.write("    command=Command(action='test', parameters={'key': 'value'})\n")
        f.write(")\n")
        f.write("print(f'Message created with command: {msg.command.action}')\n")
        f.write("print('Message creation: OK')\n")
        test_file = f.name

    success, output = run_command(f"python3 {test_file}")
    os.unlink(test_file)

    if success:
        print_success("Message creation works")
        results["passed"] += 1
    else:
        print_error(f"Message creation failed: {output}")
        results["failed"] += 1

    return results


def print_summary(all_results):
    """Print validation summary."""
    print_header("Validation Summary")

    total_passed = sum(r["passed"] for r in all_results.values())
    total_failed = sum(r["failed"] for r in all_results.values())
    total_warnings = sum(r["warnings"] for r in all_results.values())

    print(f"\n{GREEN}Passed:{RESET}   {total_passed}")
    if total_warnings > 0:
        print(f"{YELLOW}Warnings:{RESET} {total_warnings}")
    if total_failed > 0:
        print(f"{RED}Failed:{RESET}   {total_failed}")

    print("\n" + "=" * 60)
    if total_failed == 0:
        print(f"{GREEN}✅ All validations passed!{RESET}")
        print("\nYour native ROS setup is ready for production.")
        print("Start the bridge with: python run_bridge.py")
    else:
        print(f"{RED}❌ Some validations failed{RESET}")
        print("\nPlease fix the issues above before deploying.")
        print("See docs/NATIVE_ROS.md for installation help.")
    print("=" * 60)

    return total_failed == 0


def main():
    parser = argparse.ArgumentParser(description="Validate native ROS setup for Agent ROS Bridge")
    parser.add_argument("--ros2", action="store_true", help="Validate ROS2 specifically")
    parser.add_argument("--ros1", action="store_true", help="Validate ROS1 specifically")
    args = parser.parse_args()

    print_header("🤖 Agent ROS Bridge - Native ROS Validation")
    print("\nThis script validates your ROS environment for production use.")
    print("Run this before deploying to native Ubuntu servers.")

    all_results = {}

    # Auto-detect or validate specific versions
    if not args.ros1 and not args.ros2:
        # Auto-detect
        if "ROS_DISTRO" in os.environ:
            distro = os.environ["ROS_DISTRO"]
            if distro in ["humble", "jazzy", "galactic", "foxy"]:
                args.ros2 = True
            elif distro in ["noetic", "melodic", "kinetic"]:
                args.ros1 = True

    # Run validations
    if args.ros2 or (not args.ros1 and not args.ros2):
        all_results["ros2"] = check_ros2()

    if args.ros1 or (not args.ros1 and not args.ros2):
        all_results["ros1"] = check_ros1()

    all_results["bridge"] = check_agent_ros_bridge()
    all_results["functionality"] = test_basic_functionality()

    # Summary
    success = print_summary(all_results)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
