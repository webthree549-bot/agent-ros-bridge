#!/usr/bin/env python3
"""Physical Robot Testing Framework.

Safe, structured testing with real robots. Includes:
- Pre-flight safety checks
- Graduated test scenarios (non-motion → motion)
- Real-time monitoring and logging
- Automatic emergency stop triggers
- Test report generation

Usage:
    # Test with ROS2 robot (e.g., TurtleBot3)
    $ python tests/physical/test_physical_robot.py --ros2 --uri ros2://0/ --robot turtlebot3

    # Test with ROS1 robot (e.g., Fetch)
    $ python tests/physical/test_physical_robot.py --ros1 --uri ros1:/// --robot fetch

    # Dry run (no actual commands sent)
    $ python tests/physical/test_physical_robot.py --dry-run --ros2 --uri ros2://0/

    # Specific test suite
    $ python tests/physical/test_physical_robot.py --ros2 --uri ros2://0/ --suite basic
"""

import argparse
import asyncio
import json
import logging
import sys
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from agent_ros_bridge.gateway_v2.core import Command

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("physical_test")


@dataclass
class PhysicalPhysicalTestResult:
    """Result of a single test."""

    name: str
    passed: bool
    duration_ms: float
    message: str
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())


@dataclass
class RobotRobotTestSuite:
    """A suite of tests."""

    name: str
    description: str
    tests: List[Callable]
    requires_motion: bool = False
    safety_level: str = "low"  # low, medium, high


class SafetyMonitor:
    """Monitors robot safety during testing."""

    def __init__(self, robot, emergency_stop_callback: Callable):
        self.robot = robot
        self.emergency_stop = emergency_stop_callback
        self.active = False
        self._task = None
        self.limits = {
            "max_linear_velocity": 1.0,  # m/s
            "max_angular_velocity": 2.0,  # rad/s
            "max_joint_velocity": 1.0,  # rad/s
            "timeout_no_telemetry": 5.0,  # seconds
        }
        self.violations: List[Dict] = []

    async def start(self):
        """Start safety monitoring."""
        self.active = True
        self._task = asyncio.create_task(self._monitor_loop())
        logger.info("✓ Safety monitor started")

    async def stop(self):
        """Stop safety monitoring."""
        self.active = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        logger.info("✓ Safety monitor stopped")

    async def _monitor_loop(self):
        """Main monitoring loop."""
        last_telemetry = time.time()

        while self.active:
            try:
                # Check telemetry timeout
                if time.time() - last_telemetry > self.limits["timeout_no_telemetry"]:
                    logger.error("SAFETY VIOLATION: No telemetry received")
                    self.violations.append(
                        {"type": "telemetry_timeout", "timestamp": datetime.now().isoformat()}
                    )
                    await self.emergency_stop("Telemetry timeout")

                # Add more safety checks here based on telemetry data
                # This would check velocity limits, joint limits, etc.

                await asyncio.sleep(0.1)

            except Exception as e:
                logger.error(f"Safety monitor error: {e}")

    def record_telemetry(self):
        """Record that telemetry was received."""
        self.last_telemetry = time.time()


class PhysicalRobotTester:
    """Main testing framework for physical robots."""

    def __init__(self, bridge, robot, dry_run: bool = False):
        self.bridge = bridge
        self.robot = robot
        self.dry_run = dry_run
        self.results: List[PhysicalTestResult] = []
        self.safety_monitor: Optional[SafetyMonitor] = None
        self.test_data: Dict[str, Any] = {
            "start_time": datetime.now().isoformat(),
            "robot_id": robot.robot_id,
            "robot_type": robot.connector_type,
            "dry_run": dry_run,
            "tests": [],
        }

    async def setup(self):
        """Setup test environment."""
        logger.info("=" * 70)
        logger.info("PHYSICAL ROBOT TEST FRAMEWORK")
        logger.info("=" * 70)
        logger.info(f"Robot: {self.robot.name} ({self.robot.robot_id})")
        logger.info(f"Type: {self.robot.connector_type}")
        logger.info(f"Capabilities: {self.robot.capabilities}")
        logger.info(f"Dry Run: {self.dry_run}")
        logger.info("")

        # Initialize safety monitor
        if not self.dry_run:
            self.safety_monitor = SafetyMonitor(self.robot, self._emergency_stop)
            await self.safety_monitor.start()

        logger.info("✓ Setup complete")
        return True

    async def teardown(self):
        """Cleanup after testing."""
        if self.safety_monitor:
            await self.safety_monitor.stop()

        # Generate report
        await self._generate_report()

        logger.info("\n" + "=" * 70)
        logger.info("Test session complete")
        logger.info("=" * 70)

    async def _emergency_stop(self, reason: str):
        """Trigger emergency stop."""
        logger.critical(f"🚨 EMERGENCY STOP: {reason}")

        if not self.dry_run:
            try:
                # Send stop command
                await self.robot.execute(
                    Command(
                        action="publish",
                        parameters={
                            "topic": "/cmd_vel",
                            "type": "geometry_msgs/Twist",
                            "data": {
                                "linear": {"x": 0, "y": 0, "z": 0},
                                "angular": {"x": 0, "y": 0, "z": 0},
                            },
                        },
                    )
                )
                logger.info("✓ Stop command sent")
            except Exception as e:
                logger.error(f"Failed to send stop: {e}")

        self.test_data["emergency_stop"] = {
            "triggered": True,
            "reason": reason,
            "timestamp": datetime.now().isoformat(),
        }

    async def run_test(self, test_func: Callable, name: str) -> PhysicalTestResult:
        """Run a single test with timing and error handling."""
        logger.info(f"\n{'='*70}")
        logger.info(f"TEST: {name}")
        logger.info(f"{'='*70}")

        start_time = time.time()

        try:
            if self.dry_run:
                logger.info("[DRY RUN] Would execute: " + name)
                result = PhysicalTestResult(
                    name=name, passed=True, duration_ms=0, message="Dry run - no command executed"
                )
            else:
                # Execute actual test
                await test_func()
                duration = (time.time() - start_time) * 1000

                result = PhysicalTestResult(
                    name=name,
                    passed=True,
                    duration_ms=duration,
                    message="Test completed successfully",
                )
                logger.info(f"✓ PASSED ({duration:.1f}ms)")

        except Exception as e:
            duration = (time.time() - start_time) * 1000
            result = PhysicalTestResult(
                name=name,
                passed=False,
                duration_ms=duration,
                message=str(e),
                data={"traceback": traceback.format_exc()},
            )
            logger.error(f"✗ FAILED: {e}")

        self.results.append(result)
        self.test_data["tests"].append(
            {
                "name": result.name,
                "passed": result.passed,
                "duration_ms": result.duration_ms,
                "message": result.message,
            }
        )

        return result

    async def _generate_report(self):
        """Generate test report."""
        self.test_data["end_time"] = datetime.now().isoformat()
        self.test_data["summary"] = {
            "total": len(self.results),
            "passed": sum(1 for r in self.results if r.passed),
            "failed": sum(1 for r in self.results if not r.passed),
            "success_rate": (
                sum(1 for r in self.results if r.passed) / len(self.results) if self.results else 0
            ),
        }

        # Save to file
        report_dir = Path("test_reports")
        report_dir.mkdir(exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = report_dir / f"physical_test_{self.robot.robot_id}_{timestamp}.json"

        with open(report_file, "w") as f:
            json.dump(self.test_data, f, indent=2)

        logger.info(f"\n📊 Test Report: {report_file}")
        logger.info(f"   Total: {self.test_data['summary']['total']}")
        logger.info(f"   Passed: {self.test_data['summary']['passed']}")
        logger.info(f"   Failed: {self.test_data['summary']['failed']}")
        logger.info(f"   Success Rate: {self.test_data['summary']['success_rate']*100:.1f}%")

    # =================================================================
    # TEST SCENARIOS
    # =================================================================

    async def test_connection(self):
        """Test 1: Verify robot connection."""

        async def _test():
            assert self.robot.connected, "Robot not connected"
            logger.info(f"✓ Robot connected: {self.robot.name}")
            logger.info(f"✓ Capabilities: {self.robot.capabilities}")

        return await self.run_test(_test, "Connection Verification")

    async def test_telemetry_reception(self):
        """Test 2: Verify telemetry is being received."""

        async def _test():
            topics = await self.robot.execute(Command(action="get_topics", parameters={}))
            logger.info(f"Available topics: {len(topics)}")

            # Subscribe to a common telemetry topic
            if self.robot.connector_type == "ros2":
                test_topic = "/odom"
            else:
                test_topic = "/odom"

            msg_count = 0
            async for telemetry in self.robot.subscribe(test_topic, timeout=5.0):
                logger.info(f"  Received: {telemetry.topic}")
                msg_count += 1
                if msg_count >= 3:
                    break

            assert msg_count >= 3, f"Only received {msg_count} messages"
            logger.info(f"✓ Received {msg_count} telemetry messages")

        return await self.run_test(_test, "Telemetry Reception")

    async def test_non_motion_commands(self):
        """Test 3: Non-motion commands (safe)."""

        async def _test():
            # Get topics
            result = await self.robot.execute(Command(action="get_topics", parameters={}))
            logger.info(f"✓ Topics: {len(result)} found")

            # Get nodes (ROS1/ROS2 specific)
            try:
                result = await self.robot.execute(Command(action="get_nodes", parameters={}))
                logger.info(f"✓ Nodes: {len(result)} found")
            except Exception:
                pass

            # Get services
            try:
                result = await self.robot.execute(Command(action="get_services", parameters={}))
                logger.info(f"✓ Services: {len(result)} found")
            except Exception:
                pass

        return await self.run_test(_test, "Non-Motion Commands")

    async def test_tool_discovery(self):
        """Test 4: Tool discovery and AI integration."""

        async def _test():
            discovery = self.bridge.get_tool_discovery()
            tools = discovery.discover_all()

            logger.info(f"✓ Discovered {len(tools)} AI tools")

            # Show dangerous tools
            dangerous = discovery.get_dangerous_tools()
            if dangerous:
                logger.warning(f"⚠️  {len(dangerous)} dangerous tools:")
                for tool in dangerous:
                    logger.warning(f"   - {tool.name}")

            # Export to formats
            mcp_tools = discovery.to_mcp_tools(tools[:5])
            logger.info(f"✓ Exported {len(mcp_tools)} tools to MCP format")

        return await self.run_test(_test, "Tool Discovery")

    async def test_minimal_motion(self):
        """Test 5: Minimal motion test (small movement)."""

        async def _test():
            logger.warning("⚠️  EXECUTING MOTION - Ensure safe area!")
            await asyncio.sleep(2)  # Give time to abort

            # Small forward motion (0.1 m/s for 1 second)
            cmd_vel_topic = "/cmd_vel"
            twist_msg = {
                "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }

            logger.info("Sending: 0.1 m/s forward for 1 second")
            result = await self.robot.execute(
                Command(
                    action="publish",
                    parameters={
                        "topic": cmd_vel_topic,
                        "type": "geometry_msgs/Twist",
                        "data": twist_msg,
                    },
                )
            )
            logger.info(f"Command result: {result}")

            # Wait 1 second
            await asyncio.sleep(1.0)

            # Stop
            logger.info("Sending: Stop command")
            result = await self.robot.execute(
                Command(
                    action="publish",
                    parameters={
                        "topic": cmd_vel_topic,
                        "type": "geometry_msgs/Twist",
                        "data": {
                            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                        },
                    },
                )
            )
            logger.info(f"Stop result: {result}")

            logger.info("✓ Motion test complete")

        return await self.run_test(_test, "Minimal Motion Test")

    async def test_rotation(self):
        """Test 6: Rotation test (in place)."""

        async def _test():
            logger.warning("⚠️  EXECUTING ROTATION - Ensure safe area!")
            await asyncio.sleep(2)

            # Small rotation (0.3 rad/s for 2 seconds = ~34 degrees)
            cmd_vel_topic = "/cmd_vel"
            twist_msg = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.3},
            }

            logger.info("Sending: 0.3 rad/s rotation for 2 seconds")
            result = await self.robot.execute(
                Command(
                    action="publish",
                    parameters={
                        "topic": cmd_vel_topic,
                        "type": "geometry_msgs/Twist",
                        "data": twist_msg,
                    },
                )
            )

            await asyncio.sleep(2.0)

            # Stop
            await self.robot.execute(
                Command(
                    action="publish",
                    parameters={
                        "topic": cmd_vel_topic,
                        "type": "geometry_msgs/Twist",
                        "data": {
                            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                        },
                    },
                )
            )

            logger.info("✓ Rotation test complete")

        return await self.run_test(_test, "Rotation Test")

    async def test_square_pattern(self):
        """Test 7: Square pattern (advanced motion)."""

        async def _test():
            logger.warning("⚠️  EXECUTING PATTERN - Requires 1m x 1m clear space!")
            await asyncio.sleep(3)

            cmd_vel_topic = "/cmd_vel"

            # Move in a square: forward, turn, forward, turn...
            for i in range(4):
                logger.info(f"Side {i+1}/4: Forward 0.5s")
                await self.robot.execute(
                    Command(
                        action="publish",
                        parameters={
                            "topic": cmd_vel_topic,
                            "type": "geometry_msgs/Twist",
                            "data": {
                                "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                            },
                        },
                    )
                )
                await asyncio.sleep(0.5)

                # Stop
                await self.robot.execute(
                    Command(
                        action="publish",
                        parameters={
                            "topic": cmd_vel_topic,
                            "type": "geometry_msgs/Twist",
                            "data": {
                                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                            },
                        },
                    )
                )
                await asyncio.sleep(0.5)

                logger.info("Turn 90°")
                await self.robot.execute(
                    Command(
                        action="publish",
                        parameters={
                            "topic": cmd_vel_topic,
                            "type": "geometry_msgs/Twist",
                            "data": {
                                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.5},
                            },
                        },
                    )
                )
                await asyncio.sleep(1.57)  # ~90 degrees at 0.5 rad/s

                # Stop
                await self.robot.execute(
                    Command(
                        action="publish",
                        parameters={
                            "topic": cmd_vel_topic,
                            "type": "geometry_msgs/Twist",
                            "data": {
                                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                            },
                        },
                    )
                )
                await asyncio.sleep(0.5)

            logger.info("✓ Square pattern complete")

        return await self.run_test(_test, "Square Pattern")


# =================================================================
# TEST SUITES
# =================================================================

TEST_SUITES = {
    "basic": RobotTestSuite(
        name="Basic Connectivity",
        description="Non-motion tests only - completely safe",
        tests=[
            "test_connection",
            "test_telemetry_reception",
            "test_non_motion_commands",
            "test_tool_discovery",
        ],
        requires_motion=False,
        safety_level="low",
    ),
    "minimal": RobotTestSuite(
        name="Minimal Motion",
        description="Basic connectivity + minimal motion tests",
        tests=[
            "test_connection",
            "test_telemetry_reception",
            "test_non_motion_commands",
            "test_tool_discovery",
            "test_minimal_motion",
        ],
        requires_motion=True,
        safety_level="medium",
    ),
    "full": RobotTestSuite(
        name="Full Test Suite",
        description="All tests including motion patterns",
        tests=[
            "test_connection",
            "test_telemetry_reception",
            "test_non_motion_commands",
            "test_tool_discovery",
            "test_minimal_motion",
            "test_rotation",
            "test_square_pattern",
        ],
        requires_motion=True,
        safety_level="high",
    ),
}


async def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Physical Robot Testing Framework")
    parser.add_argument("--ros2", action="store_true", help="Use ROS2 connector")
    parser.add_argument("--ros1", action="store_true", help="Use ROS1 connector")
    parser.add_argument("--uri", required=True, help="Robot URI (e.g., ros2://0/ or ros1:///)")
    parser.add_argument("--robot", default="unknown", help="Robot name/model")
    parser.add_argument("--dry-run", action="store_true", help="Dry run (no commands executed)")
    parser.add_argument(
        "--suite", default="basic", choices=list(TEST_SUITES.keys()), help="Test suite to run"
    )
    parser.add_argument(
        "--confirm", action="store_true", help="Skip confirmation prompts (for automation)"
    )

    args = parser.parse_args()

    # Validate args
    if not args.ros2 and not args.ros1:
        parser.error("Must specify --ros2 or --ros1")

    # Import based on ROS version
    from agent_ros_bridge.gateway_v2.core import Bridge

    if args.ros2:
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector

        connector_class = ROS2Connector
    else:
        from agent_ros_bridge.gateway_v2.connectors.ros1_connector import ROS1Connector

        connector_class = ROS1Connector

    # Get test suite
    suite = TEST_SUITES[args.suite]

    # Safety confirmation
    print("\n" + "=" * 70)
    print("PHYSICAL ROBOT TEST - SAFETY CONFIRMATION")
    print("=" * 70)
    print(f"Robot: {args.robot}")
    print(f"URI: {args.uri}")
    print(f"Suite: {suite.name}")
    print(f"Motion Required: {'YES' if suite.requires_motion else 'NO'}")
    print(f"Safety Level: {suite.safety_level.upper()}")
    print(f"Dry Run: {'YES' if args.dry_run else 'NO'}")
    print("")

    if suite.requires_motion and not args.dry_run:
        print("⚠️  WARNING: This test suite will command robot motion!")
        print("   Ensure:")
        print("   • Robot is in a safe, clear area")
        print("   • Emergency stop is within reach")
        print("   • No humans or obstacles nearby")
        print("   • Robot battery is charged")
        print("")

    if not args.confirm:
        response = input("Proceed? (yes/no): ")
        if response.lower() != "yes":
            print("Aborted.")
            return 1

    # Create bridge and connect
    bridge = Bridge()
    connector = connector_class()
    bridge.connector_registry.register(connector)

    logger.info(f"Connecting to {args.uri}...")
    robot = await connector.connect(args.uri, name=args.robot)

    if not robot:
        logger.error("Failed to connect to robot")
        return 1

    logger.info(f"✓ Connected to {robot.name}")

    # Create tester and run tests
    tester = PhysicalRobotTester(bridge, robot, dry_run=args.dry_run)

    try:
        await tester.setup()

        # Run selected tests
        for test_name in suite.tests:
            test_func = getattr(tester, test_name, None)
            if test_func:
                await test_func()
            else:
                logger.error(f"Unknown test: {test_name}")

    except KeyboardInterrupt:
        logger.info("\n⚠️  Interrupted by user")
        await tester._emergency_stop("User interrupt")

    finally:
        await tester.teardown()
        await robot.disconnect()

    # Return exit code based on results
    failed = sum(1 for r in tester.results if not r.passed)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
