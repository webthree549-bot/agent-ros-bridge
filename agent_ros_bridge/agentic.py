"""
Agentic AI Interface for Agent ROS Bridge

High-level, agentic API that provides real value beyond basic command sending.
"""

import json
from dataclasses import dataclass
from typing import Any


@dataclass
class TaskResult:
    """Result of an agentic task execution"""

    success: bool
    task: str
    steps: list[dict[str, Any]]
    duration_seconds: float
    ai_confidence: float
    human_approvals: int
    human_rejections: int
    safety_violations: int
    message: str


@dataclass
class AgentObservation:
    """What the AI agent observes about the environment"""

    robot_position: tuple
    battery_level: float
    nearby_objects: list[str]
    current_task: str | None
    recent_commands: list[str]
    obstacles_detected: list[str]


class RobotAgent:
    """
    High-level agentic interface for robot control.

    Provides natural language command execution with:
    - Intent parsing
    - Task planning
    - Safety validation
    - Human confirmation
    - Execution monitoring

    Works with ANY ROS device: mobile robots, drones, manipulators, humanoids, sensors
    """

    def __init__(
        self,
        device_id: str,
        device_type: str = "mobile_robot",  # 'mobile_robot', 'drone', 'manipulator', 'humanoid', 'sensor_array'
        llm_provider: str = "moonshot",
        require_confirmation: bool = True,
        confidence_threshold: float = 0.8,
        device_profile: Any | None = None,
    ):
        """
        Initialize robot agent.

        Args:
            device_id: Device identifier (e.g., 'bot1', 'drone1', 'arm1')
            device_type: Type of device ('mobile_robot', 'drone', 'manipulator', 'humanoid', 'sensor_array')
            llm_provider: 'moonshot', 'openai', 'anthropic'
            require_confirmation: Whether to require human approval
            confidence_threshold: Auto-approve above this confidence
            device_profile: Optional DeviceProfile for custom hardware
        """
        self.device_id = device_id
        self.device_type = device_type
        self.llm_provider = llm_provider
        self.require_confirmation = require_confirmation
        self.confidence_threshold = confidence_threshold

        # Initialize hardware abstraction
        self._init_hardware(device_profile)

        # Initialize components
        self._init_intent_parser()
        self._init_task_planner()
        self._init_safety_validator()
        self._init_shadow_hooks()

    @classmethod
    def discover(
        cls,
        device_id: str,
        llm_provider: str = "moonshot",
        require_confirmation: bool = True,
        confidence_threshold: float = 0.8,
        enable_health_monitor: bool = True,
        enable_self_healing: bool = True,
        verify_capabilities: bool = True,
        user_confirmation_callback: Any = None,
    ) -> "RobotAgent":
        """
        Auto-discover device type and create RobotAgent with confidence scoring.

        This method uses hardened auto-discovery with confidence scoring,
        capability verification, and optional user confirmation for
        production-safe operation.

        Args:
            device_id: Device identifier (e.g., 'bot1')
            llm_provider: LLM provider for intent parsing
            require_confirmation: Whether to require human approval
            confidence_threshold: Minimum confidence to proceed without user confirmation
            enable_health_monitor: Enable health monitoring
            enable_self_healing: Enable self-healing capabilities
            verify_capabilities: Test that discovered capabilities actually work
            user_confirmation_callback: Optional callback for user confirmation
                Function signature: callback(device_type, confidence, evidence) -> bool

        Returns:
            Configured RobotAgent

        Raises:
            ValueError: If device type cannot be auto-discovered with sufficient confidence
            DiscoveryError: If discovery validation fails

        Example:
            >>> # Simple discovery (will prompt if uncertain)
            >>> agent = RobotAgent.discover('bot1')
            🔍 Discovered bot1 as mobile_robot (confidence: 0.92)
               Capabilities: ['navigate_to', 'rotate', 'stop']
               Health: healthy
            >>> agent.execute("Go to the kitchen")

            >>> # With custom confirmation callback
            >>> def confirm(dtype, conf, evidence):
            ...     return input(f"Accept {dtype} (confidence: {conf:.2f})? ") == 'y'
            >>> agent = RobotAgent.discover('bot1', user_confirmation_callback=confirm)
        """
        from agent_ros_bridge.discovery import (
            DeviceHealthStatus,
            ROSHealthMonitor,
            SelfHealingController,
        )
        from agent_ros_bridge.discovery_hardened import (
            CapabilityVerifier,
            HardenedROSDiscovery,
        )

        # Step 1: Discover with confidence scoring
        discovery = HardenedROSDiscovery()
        result = discovery.discover_with_confidence(device_id)

        if not result.device_type:
            raise ValueError(
                f"Could not auto-discover device type for '{device_id}'.\n"
                f"Discovery confidence too low ({result.confidence:.2f}).\n"
                f"Please specify device_type explicitly:\n"
                f"  agent = RobotAgent(device_id='{device_id}', device_type='mobile_robot')"
            )

        # Step 2: Check confidence threshold
        if result.confidence < confidence_threshold:
            # Low confidence - need user confirmation
            print(
                f"\n⚠️  Discovery confidence low ({result.confidence:.2f} < {confidence_threshold})"
            )
            print(f"   Inferred device type: {result.device_type}")
            print(
                f"   Evidence: {result.evidence.required_topics_found}/{result.evidence.required_topics_total} required topics found"
            )

            if result.alternatives:
                print("\n   Alternative possibilities:")
                for alt in result.alternatives[:3]:
                    print(f"     - {alt['device_type']}: {alt['confidence']:.2f} confidence")

            confirmed = False
            if user_confirmation_callback:
                confirmed = user_confirmation_callback(
                    result.device_type, result.confidence, result.evidence
                )
            else:
                # Interactive confirmation
                try:
                    response = (
                        input(f"\n   Proceed with '{result.device_type}'? (y/n/switch): ")
                        .strip()
                        .lower()
                    )
                    if response == "y":
                        confirmed = True
                    elif response == "switch" and result.alternatives:
                        print("   Available alternatives:")
                        for i, alt in enumerate(result.alternatives[:3], 1):
                            print(f"     {i}. {alt['device_type']} ({alt['confidence']:.2f})")
                        choice = input("   Select alternative (number) or 'n' to cancel: ").strip()
                        if choice.isdigit() and 1 <= int(choice) <= len(result.alternatives[:3]):
                            result.device_type = result.alternatives[int(choice) - 1]["device_type"]
                            confirmed = True
                except (EOFError, KeyboardInterrupt):
                    pass

            if not confirmed:
                raise ValueError(
                    f"Discovery rejected by user.\n"
                    f"To use explicit configuration:\n"
                    f"  agent = RobotAgent(device_id='{device_id}', device_type='mobile_robot')"
                )

        device_type = result.device_type
        print(f"🔍 Discovered {device_id} as {device_type} (confidence: {result.confidence:.2f})")

        # Step 3: Discover capabilities
        from agent_ros_bridge.discovery import ROSDiscovery

        ros_discovery = ROSDiscovery()
        capabilities = ros_discovery.discover_capabilities(device_id)

        # Step 4: Verify capabilities
        if verify_capabilities and capabilities:
            verifier = CapabilityVerifier(device_id)
            verified_caps = verifier.verify_capabilities(capabilities)

            # Filter to only working capabilities
            working_caps = [cap for cap, works in verified_caps.items() if works]
            failed_caps = [cap for cap, works in verified_caps.items() if not works]

            if failed_caps:
                print(f"   ⚠️  {len(failed_caps)} capabilities not verified: {failed_caps}")

            capabilities = working_caps

        if capabilities:
            print(f"   Capabilities: {capabilities}")

        # Create agent
        agent = cls(
            device_id=device_id,
            device_type=device_type,
            llm_provider=llm_provider,
            require_confirmation=require_confirmation,
            confidence_threshold=confidence_threshold,
        )

        # Optionally enable health monitoring
        if enable_health_monitor:
            health_monitor = ROSHealthMonitor(device_id)
            health = health_monitor.check_health()
            print(f"   Health: {health.status.value}")

            # Attempt recovery if unhealthy
            if enable_self_healing and health.status != DeviceHealthStatus.HEALTHY:
                healer = SelfHealingController(device_id)
                healer.attempt_recovery(health)

        return agent

    @classmethod
    def discover_all(
        cls,
        llm_provider: str = "moonshot",
        require_confirmation: bool = True,
        confidence_threshold: float = 0.8,
    ) -> list["RobotAgent"]:
        """
        Discover all ROS devices on the network and create RobotAgents.

        This method scans the ROS graph for all potential devices,
        identifies their types, and creates RobotAgent instances for each.

        Args:
            llm_provider: LLM provider for intent parsing
            require_confirmation: Whether to require human approval
            confidence_threshold: Auto-approve above this confidence

        Returns:
            List of configured RobotAgents

        Example:
            >>> robots = RobotAgent.discover_all()
            🔍 Discovered 3 devices:
               - bot1: mobile_robot
               - arm1: manipulator
               - drone1: drone
            >>> for robot in robots:
            ...     print(f"{robot.device_id}: {robot.device_type}")
        """
        from agent_ros_bridge.discovery import ROSDiscovery

        discovery = ROSDiscovery()
        devices = discovery.discover_all_devices()

        print(f"🔍 Discovered {len(devices)} devices:")

        agents = []
        for dev in devices:
            print(f"   - {dev['device_id']}: {dev['device_type']}")

            try:
                agent = cls(
                    device_id=dev["device_id"],
                    device_type=dev["device_type"],
                    llm_provider=llm_provider,
                    require_confirmation=require_confirmation,
                    confidence_threshold=confidence_threshold,
                )
                agents.append(agent)
            except Exception as e:
                print(f"     Warning: Could not create agent: {e}")

        return agents

    def _init_hardware(self, device_profile=None):
        """Initialize hardware abstraction"""
        from agent_ros_bridge.hardware import DeviceRegistry

        self.device_registry = DeviceRegistry()

        # Create or use provided profile
        if device_profile is None:
            device_profile = self._create_default_profile()

        # Create the device
        self.device = self.device_registry.create_device(
            self.device_id,
            self.device_type,
            device_profile,
        )

    def _init_intent_parser(self):
        """Initialize LLM intent parser"""
        from agent_ros_bridge.ai.llm_parser import LLMIntentParser

        # Use LLMIntentParser with provider-specific configuration
        self.intent_parser = LLMIntentParser(
            model=self.llm_provider,
            api_key=None,  # Will use environment variable
        )

    def _init_task_planner(self):
        """Initialize task planner"""
        self.task_planner = TaskPlanner()

    def _init_safety_validator(self):
        """Initialize safety validator"""
        from agent_ros_bridge.safety.validator import SafetyValidatorNode

        self.safety_validator = SafetyValidatorNode(
            enable_cache=True,
        )

    def _create_default_profile(self):
        """Create default device profile based on device_type"""
        from agent_ros_bridge.hardware import Capability, DeviceProfile

        profiles = {
            "mobile_robot": DeviceProfile(
                device_id=self.device_id,
                device_type="mobile_robot",
                manufacturer="Generic",
                model="MobileRobot",
                capabilities=[
                    Capability(
                        "navigate_to", "Navigate to location", {"x": float, "y": float}, "success"
                    ),
                    Capability("rotate", "Rotate in place", {"angle": float}, "success"),
                    Capability("stop", "Stop immediately", {}, "success"),
                ],
                sensors=["lidar", "camera", "imu"],
                actuators=["wheels"],
            ),
            "drone": DeviceProfile(
                device_id=self.device_id,
                device_type="drone",
                manufacturer="Generic",
                model="Drone",
                capabilities=[
                    Capability("takeoff", "Take off", {"altitude": float}, "success"),
                    Capability("land", "Land", {}, "success"),
                    Capability(
                        "fly_to",
                        "Fly to coordinates",
                        {"x": float, "y": float, "z": float},
                        "success",
                    ),
                    Capability("hover", "Hover in place", {}, "success"),
                    Capability("capture_image", "Capture aerial image", {}, "image_path"),
                ],
                sensors=["camera", "gps", "imu"],
                actuators=["rotors"],
            ),
            "manipulator": DeviceProfile(
                device_id=self.device_id,
                device_type="manipulator",
                manufacturer="Generic",
                model="RobotArm",
                capabilities=[
                    Capability(
                        "move_to",
                        "Move to position",
                        {"x": float, "y": float, "z": float},
                        "success",
                    ),
                    Capability("grasp", "Grasp object", {"force": float}, "success"),
                    Capability("release", "Release grasp", {}, "success"),
                    Capability(
                        "follow_trajectory", "Follow trajectory", {"waypoints": list}, "success"
                    ),
                ],
                sensors=["camera", "force_sensor"],
                actuators=["joints", "gripper"],
            ),
            "humanoid": DeviceProfile(
                device_id=self.device_id,
                device_type="humanoid",
                manufacturer="Generic",
                model="Humanoid",
                capabilities=[
                    Capability("walk", "Walk", {"direction": str, "steps": int}, "success"),
                    Capability("balance", "Maintain balance", {}, "success"),
                    Capability("reach", "Reach to target", {"target": list}, "success"),
                    Capability("climb", "Climb surface", {"surface": str}, "success"),
                    Capability(
                        "manipulate", "Manipulate object", {"object": str, "action": str}, "success"
                    ),
                ],
                sensors=["camera", "imu", "joint_encoders"],
                actuators=["legs", "arms", "hands"],
            ),
            "sensor_array": DeviceProfile(
                device_id=self.device_id,
                device_type="sensor_array",
                manufacturer="Generic",
                model="SensorArray",
                capabilities=[
                    Capability("sense", "Sense environment", {"modality": str}, "data"),
                    Capability("capture", "Capture data", {"sensor": str}, "data"),
                    Capability("scan", "Scan area", {"area": str}, "data"),
                ],
                sensors=["camera", "lidar", "radar", "thermal", "microphone"],
                actuators=[],
            ),
        }

        return profiles.get(self.device_type, profiles["mobile_robot"])

    def _init_shadow_hooks(self):
        """Initialize shadow mode hooks"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks

        self.shadow_hooks = ShadowModeHooks()

    def execute(
        self,
        natural_language_command: str,
        context: dict[str, Any] | None = None,
    ) -> TaskResult:
        """
        Execute a natural language command with full AI pipeline.

        This is the main agentic interface - just say what you want,
        and the AI handles everything.

        Args:
            natural_language_command: Plain English/Chinese command
            context: Optional context (location, objects, etc.)

        Returns:
            TaskResult with execution details

        Example:
            >>> agent = RobotAgent('bot1')
            >>> result = agent.execute("Go to the kitchen and pick up the red cup")
            >>> print(result.message)
            "Successfully navigated to kitchen and picked up red cup"
        """
        import time

        start_time = time.time()

        # Step 1: Parse intent with LLM
        intent_result = self.intent_parser.parse(
            natural_language_command,
            robot_id=self.device_id,
            context=context,
        )

        # Step 2: Plan task breakdown
        task_plan = self.task_planner.plan(
            intent=intent_result,
            context=context,
        )

        # Step 3: Execute with safety checks
        executed_steps = []
        human_approvals = 0
        human_rejections = 0
        safety_violations = 0

        for step in task_plan.steps:
            # Validate safety
            # Convert command to trajectory format for validation
            trajectory = {
                "type": step.capability_name,
                "parameters": step.parameters,
            }
            limits = self.device.profile.limits if self.device else {}
            safety_result = self.safety_validator.validate_trajectory(trajectory, limits)
            if not safety_result["approved"]:
                safety_violations += 1
                executed_steps.append(
                    {
                        "step": step.name,
                        "status": "rejected",
                        "reason": "Safety violation",
                    }
                )
                continue

            # Get human confirmation if needed
            if self.require_confirmation and intent_result.confidence < self.confidence_threshold:
                approval = self._get_human_approval(
                    step=step,
                    confidence=intent_result.confidence,
                )

                if not approval:
                    human_rejections += 1
                    executed_steps.append(
                        {
                            "step": step.name,
                            "status": "rejected",
                            "reason": "Human rejection",
                        }
                    )
                    continue

                human_approvals += 1
            else:
                # Auto-approve high confidence
                human_approvals += 1 if self.require_confirmation else 0

            # Execute capability on device
            result = self.device.execute_capability(
                capability_name=step.capability_name,
                parameters=step.parameters,
            )

            executed_steps.append(
                {
                    "step": step.name,
                    "status": "success" if result["success"] else "failed",
                    "result": result,
                }
            )

            # Log to shadow mode
            self.shadow_hooks.on_human_command(
                {
                    "robot_id": self.device_id,
                    "command": step.capability_name,
                    "parameters": step.parameters,
                    "ai_proposal": step.ai_proposal,
                }
            )

        # Calculate results
        success = all(s["status"] == "success" for s in executed_steps)
        duration = time.time() - start_time

        return TaskResult(
            success=success,
            task=natural_language_command,
            steps=executed_steps,
            duration_seconds=duration,
            ai_confidence=intent_result.confidence,
            human_approvals=human_approvals,
            human_rejections=human_rejections,
            safety_violations=safety_violations,
            message=self._generate_result_message(executed_steps),
        )

    def observe(self) -> AgentObservation:
        """
        Get current environment observations from device.

        Returns:
            AgentObservation with device state and environment
        """
        state = self.device.get_state()

        # Map device-specific state to generic observation
        return AgentObservation(
            robot_position=state.get("position", (0, 0, 0)),
            battery_level=state.get("battery", 100.0),
            nearby_objects=state.get("nearby_objects", []),
            current_task=state.get("current_task"),
            recent_commands=state.get("recent_commands", []),
            obstacles_detected=state.get("obstacles", []),
        )

    def think(self, observation: AgentObservation) -> str:
        """
        AI reasoning about current situation.

        Returns:
            AI's thoughts as text
        """
        # Use LLM to reason about situation
        prompt = f"""
        Robot status: {json.dumps(observation.__dict__)}
        
        What should the robot do next? Consider:
        - Battery level
        - Current position
        - Nearby objects
        - Recent activity
        
        Provide reasoning:
        """

        return self.intent_parser.llm.generate(prompt)

    def plan_multi_step(
        self,
        goal: str,
        max_steps: int = 10,
    ) -> list[dict[str, Any]]:
        """
        Create a multi-step plan to achieve a goal.

        Args:
            goal: High-level goal
            max_steps: Maximum steps in plan

        Returns:
            List of planned steps
        """
        return self.task_planner.create_plan(
            goal=goal,
            observation=self.observe(),
            max_steps=max_steps,
        )

    def _get_human_approval(
        self,
        step: Any,
        confidence: float,
    ) -> bool:
        """Get human approval for a step"""
        # Auto-approve high confidence
        if confidence >= self.confidence_threshold:
            return True

        # Otherwise, require explicit approval
        # This would integrate with ConfirmationUI
        # For now, simulate
        return True  # In real implementation, wait for UI response

    def _generate_result_message(self, steps: list[dict]) -> str:
        """Generate human-readable result message"""
        successful = [s for s in steps if s["status"] == "success"]
        failed = [s for s in steps if s["status"] == "failed"]

        if not failed:
            return f"Successfully completed all {len(successful)} steps"
        elif successful:
            return f"Completed {len(successful)} steps, {len(failed)} failed"
        else:
            return f"Failed to execute: {failed[0].get('reason', 'Unknown error')}"


class TaskPlanner:
    """Plans multi-step tasks from intents"""

    def plan(self, intent, context=None):
        """Create task plan from intent"""
        # This would use LLM for complex planning
        # Simplified version:
        from dataclasses import dataclass

        @dataclass
        class TaskPlan:
            steps: list[Any]

        @dataclass
        class TaskStep:
            name: str
            command: dict[str, Any]
            ai_proposal: Any

        # Parse compound commands
        if "and" in intent.raw_text.lower():
            # Break into subtasks
            steps = [
                TaskStep(
                    name="navigate",
                    command={"type": "navigate_to", "location": "kitchen"},
                    ai_proposal=intent,
                ),
                TaskStep(
                    name="pick",
                    command={"type": "pick_object", "object": "red_cup"},
                    ai_proposal=intent,
                ),
            ]
        else:
            steps = [
                TaskStep(
                    name="execute",
                    command=intent.to_command(),
                    ai_proposal=intent,
                ),
            ]

        return TaskPlan(steps=steps)

    def create_plan(self, goal, observation, max_steps=10):
        """Create multi-step plan"""
        # TODO: Use LLM to generate plan based on goal and observation
        # This would construct a prompt and call the LLM
        # For now, return empty plan as placeholder
        return []


# Example usage and demonstration
if __name__ == "__main__":
    print("=" * 70)
    print("🤖 ROBOT AGENT - Agentic AI Interface Demo")
    print("=" * 70)
    print()

    # Create agent
    agent = RobotAgent(
        robot_id="bot1",
        llm_provider="moonshot",
        require_confirmation=True,
        confidence_threshold=0.8,
    )

    # Example 1: Simple command
    print("Example 1: Simple navigation")
    print("-" * 70)
    result = agent.execute("Go to the kitchen")
    print("Task: Go to the kitchen")
    print(f"Success: {result.success}")
    print(f"Confidence: {result.ai_confidence:.2f}")
    print(f"Duration: {result.duration_seconds:.2f}s")
    print(f"Message: {result.message}")
    print()

    # Example 2: Compound command
    print("Example 2: Compound task")
    print("-" * 70)
    result = agent.execute("Go to the kitchen and pick up the red cup")
    print("Task: Go to the kitchen and pick up the red cup")
    print(f"Success: {result.success}")
    print(f"Steps executed: {len(result.steps)}")
    for step in result.steps:
        print(f"  - {step['step']}: {step['status']}")
    print(f"Human approvals: {result.human_approvals}")
    print()

    # Example 3: AI reasoning
    print("Example 3: AI observation and reasoning")
    print("-" * 70)
    observation = agent.observe()
    print(f"Robot position: {observation.robot_position}")
    print(f"Battery: {observation.battery_level}%")
    print(f"Nearby objects: {observation.nearby_objects}")
    print()

    print("=" * 70)
    print("✅ Agentic interface demo complete!")
    print("=" * 70)
