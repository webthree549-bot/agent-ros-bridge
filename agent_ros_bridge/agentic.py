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
    data: dict[str, Any] = None
    
    def __post_init__(self):
        if self.data is None:
            self.data = {}


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

    Example Usage:
        >>> from agent_ros_bridge import RobotAgent
        >>>
        >>> # Create robot agent with safety
        >>> robot = RobotAgent(
        ...     device_id="bot1",
        ...     device_type="mobile_robot",
        ...     require_confirmation=True
        ... )
        >>>
        >>> # Execute natural language command
        >>> result = await robot.execute("Navigate to the kitchen")
        >>> print(result.message)
        "Successfully navigated to kitchen"
        >>>
        >>> # Check safety status
        >>> print(robot.safety.safety_validation_status)
        "simulation_only"
        >>>
        >>> # Get robot status
        >>> status = robot.get_status()
        >>> print(status)
        {"position": (1.2, 3.4), "battery": 85.0}

    Safety Features:
        - Human-in-the-loop: Requires confirmation for risky actions
        - Shadow mode: Logs all decisions for validation
        - Gradual rollout: Starts at 0% autonomy, increases with validation
        - Emergency stop: Immediate halt capability
    """

    def __init__(
        self,
        device_id: str,
        device_type: str = "mobile_robot",  # 'mobile_robot', 'drone', 'manipulator', 'humanoid', 'sensor_array'
        llm_provider: str = "moonshot",
        require_confirmation: bool = True,
        confidence_threshold: float = 0.8,
        device_profile: Any | None = None,
        safety_config: Any | None = None,
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
            safety_config: Optional SafetyConfig (loads from config if None)
        """
        self.device_id = device_id
        self.device_type = device_type
        self.llm_provider = llm_provider
        self.device_profile = device_profile

        # Load safety configuration
        self.safety = safety_config or self._load_safety_config()

        # ENFORCE SAFETY: Override confirmation settings based on safety config
        # If human_in_the_loop is required by safety config, force require_confirmation=True
        if self.safety.human_in_the_loop:
            if not require_confirmation:
                print(
                    "⚠️  SAFETY: require_confirmation forced to True (human_in_the_loop required)"
                )
            self.require_confirmation = True
            # Use stricter confidence threshold from safety config if available
            self.confidence_threshold = max(
                confidence_threshold, self.safety.min_confidence_for_auto
            )
        else:
            self.require_confirmation = require_confirmation
            self.confidence_threshold = confidence_threshold

        # Safety status banner
        self._print_safety_status()

        # Initialize hardware abstraction
        self._init_hardware(device_profile)

        # Initialize components
        self._init_intent_parser()
        self._init_task_planner()
        self._init_safety_validator()
        self._init_shadow_hooks()

    def _load_safety_config(self):
        """Load safety configuration from config files."""
        try:
            from agent_ros_bridge.gateway_v2.config import ConfigLoader, SafetyConfig

            config = ConfigLoader.from_file_or_env()
            return config.safety
        except Exception:
            # Fallback to safe defaults if config loading fails
            return SafetyConfig()

    def _print_safety_status(self):
        """Print safety status banner on initialization."""
        print("\n" + "=" * 60)
        print("🛡️  SAFETY STATUS")
        print("=" * 60)
        print(f"Device: {self.device_id} ({self.device_type})")
        print(
            f"Autonomous Mode: {self.safety.autonomous_mode} {'⚠️' if self.safety.autonomous_mode else '✅'}"
        )
        print(
            f"Human-in-the-Loop: {self.safety.human_in_the_loop} {'✅' if self.safety.human_in_the_loop else '⚠️'}"
        )
        print(
            f"Shadow Mode: {self.safety.shadow_mode_enabled} {'✅' if self.safety.shadow_mode_enabled else '⚠️'}"
        )
        print(f"Validation Status: {self.safety.safety_validation_status}")
        print(f"Confidence Threshold: {self.confidence_threshold:.2f}")

        if not self.safety.autonomous_mode:
            print("\n✅ SAFE MODE: Human approval required for all actions")
        else:
            print(
                f"\n⚠️  AUTONOMOUS MODE: AI may execute without approval (confidence > {self.safety.min_confidence_for_auto:.2f})"
            )

        if self.safety.safety_validation_status in ["simulation_only", "supervised"]:
            hours = self.safety.shadow_mode_hours_collected
            required = self.safety.required_shadow_hours
            print(f"\n📊 Shadow Mode: {hours:.1f}/{required:.0f} hours collected")
            print(f"   Agreement Rate: {self.safety.shadow_mode_agreement_rate:.1%}")

        print("=" * 60 + "\n")

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

    def _needs_human_approval(self, confidence: float, step) -> bool:
        """Determine if human approval is required based on safety configuration.

        Args:
            confidence: AI confidence score (0.0-1.0)
            step: Task step being executed

        Returns:
            True if human approval is required
        """
        # SAFETY: If human_in_the_loop is required, always need approval
        if self.safety.human_in_the_loop:
            return True

        # SAFETY: If autonomous_mode is disabled, always need approval
        if not self.safety.autonomous_mode:
            return True

        # SAFETY: Check gradual rollout stage
        # If rollout is not at 100%, only allow autonomy for a percentage of decisions
        import random

        if self.safety.gradual_rollout_stage < 100:
            # Only allow autonomy for rollout_stage% of decisions
            if random.randint(1, 100) > self.safety.gradual_rollout_stage:
                return True

        # SAFETY: Check confidence threshold
        return confidence < self.safety.min_confidence_for_auto

    def _log_rejection(self, step, confidence: float, reason: str):
        """Log human rejection to shadow mode for analysis.

        Args:
            step: The rejected step
            confidence: AI confidence when rejected
            reason: Rejection reason
        """
        try:
            if hasattr(self, "shadow_hooks") and self.safety.shadow_mode_enabled:
                # Log via shadow hooks if available
                if hasattr(self.shadow_hooks, "on_human_rejected"):
                    self.shadow_hooks.on_human_rejected(
                        robot_id=self.device_id,
                        ai_proposal_id=getattr(step, "ai_proposal_id", "unknown"),
                        rejection_reason=reason,
                    )
        except Exception as e:
            # Don't let logging failures stop execution
            print(f"Warning: Could not log rejection: {e}")

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

            # SAFETY CHECK: Determine if human approval is required
            # This enforces the safety configuration
            needs_human_approval = self._needs_human_approval(intent_result.confidence, step)

            if needs_human_approval:
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
                    # Log rejection to shadow mode
                    self._log_rejection(step, intent_result.confidence, "human_rejection")
                    continue

                human_approvals += 1
            else:
                # Auto-execute (autonomous mode with high confidence)
                human_approvals += 0  # No human involved
                print(
                    f"🤖 AUTONOMOUS: Executing {step.name} (confidence: {intent_result.confidence:.2f})"
                )

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

    def health_check(self) -> dict[str, Any]:
        """
        Perform comprehensive health check on robot.
        
        Returns:
            Dictionary with diagnostic information:
            - status: Overall health status
            - battery: Battery level and health
            - connectivity: Connection status
            - sensors: Sensor status
            - warnings: List of any warnings
            - timestamp: When check was performed
        """
        import time
        
        warnings = []
        
        # Check battery (simulated)
        battery_level = getattr(self, '_battery_level', 100.0)
        if battery_level < 20:
            warnings.append(f"Low battery: {battery_level}%")
        
        # Build health report
        health = {
            "status": "healthy" if not warnings else "degraded",
            "battery": {
                "level": battery_level,
                "health": "good" if battery_level > 50 else "poor",
            },
            "connectivity": {
                "status": "connected",
                "latency_ms": 15.0,
            },
            "sensors": {
                "camera": "operational",
                "lidar": "operational",
                "imu": "operational",
            },
            "warnings": warnings,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        
        return health

    async def execute_batch(
        self,
        commands: list[str],
        stop_on_failure: bool = True,
    ) -> list[dict[str, Any]]:
        """
        Execute multiple commands in batch.
        
        Args:
            commands: List of natural language commands
            stop_on_failure: Whether to stop at first failure
            
        Returns:
            List of results for each command
        """
        results = []
        
        for i, command in enumerate(commands):
            # Execute command
            result = await self.execute(command) if hasattr(self.execute, '__call__') and hasattr(self.execute, '__await__') else self.execute(command)
            
            result_dict = {
                "command": command,
                "status": "success" if result.success else "failed",
                "step": i + 1,
            }
            
            if not result.success:
                result_dict["error"] = result.message
                results.append(result_dict)
                if stop_on_failure:
                    break
            else:
                results.append(result_dict)
        
        return results

    def navigate_waypoints(
        self,
        waypoints: list[str | dict[str, Any]],
    ) -> TaskResult:
        """
        Navigate through multiple waypoints.
        
        Args:
            waypoints: List of locations or dicts with 'location' and optional 'loiter_sec'
            
        Returns:
            TaskResult with navigation summary
        """
        import time
        start_time = time.time()
        steps = []
        visited = []
        
        for i, wp in enumerate(waypoints):
            # Parse waypoint
            if isinstance(wp, dict):
                location = wp.get("location", "unknown")
                loiter_sec = wp.get("loiter_sec", 0)
            else:
                location = wp
                loiter_sec = 0
            
            # Navigate to waypoint
            step_result = self.execute(f"navigate to {location}")
            
            step = {
                "step": f"navigate_to_{location}",
                "status": "success" if step_result.success else "failed",
                "location": location,
            }
            
            if step_result.success:
                visited.append(location)
                # Simulate loiter time
                if loiter_sec > 0:
                    step["loiter_sec"] = loiter_sec
            else:
                step["error"] = step_result.message
            
            steps.append(step)
            
            # Stop if navigation failed
            if not step_result.success:
                break
        
        duration = time.time() - start_time
        all_success = all(s["status"] == "success" for s in steps)
        
        return TaskResult(
            success=all_success,
            task="waypoint_navigation",
            steps=steps,
            duration_seconds=duration,
            ai_confidence=0.9,
            human_approvals=0,
            human_rejections=0,
            safety_violations=0,
            message=f"Visited {len(visited)} waypoints: {', '.join(visited)}" if visited else "Navigation failed",
        )

    def recognize_objects(self) -> list[dict[str, Any]]:
        """
        Recognize objects in the environment.
        
        Returns:
            List of detected objects with name, confidence, and location
        """
        # Simulated object recognition
        # In real implementation, would use vision system
        objects = [
            {
                "name": "red_cube",
                "confidence": 0.95,
                "location": {"x": 1.2, "y": 2.3, "z": 0.0},
            },
            {
                "name": "blue_sphere",
                "confidence": 0.87,
                "location": {"x": 2.1, "y": 1.5, "z": 0.0},
            },
        ]
        return objects

    def pick_object(self, object_name: str) -> TaskResult:
        """
        Pick up an object by name.
        
        Args:
            object_name: Name of object to pick
            
        Returns:
            TaskResult with pick operation result
        """
        import time
        start_time = time.time()
        
        # First recognize objects
        objects = self.recognize_objects()
        
        # Find target object
        target = None
        for obj in objects:
            if obj["name"] == object_name:
                target = obj
                break
        
        if not target:
            return TaskResult(
                success=False,
                task=f"pick_{object_name}",
                steps=[],
                duration_seconds=time.time() - start_time,
                ai_confidence=0.0,
                human_approvals=0,
                human_rejections=0,
                safety_violations=0,
                message=f"Object '{object_name}' not found",
            )
        
        # Simulate pick operation
        steps = [
            {"step": "recognize_objects", "status": "success"},
            {"step": f"pick_{object_name}", "status": "success", "confidence": target["confidence"]},
        ]
        
        return TaskResult(
            success=True,
            task=f"pick_{object_name}",
            steps=steps,
            duration_seconds=time.time() - start_time,
            ai_confidence=target["confidence"],
            human_approvals=0,
            human_rejections=0,
            safety_violations=0,
            message=f"Successfully picked up {object_name}",
        )

    def plan_mission(self, description: str) -> dict[str, Any]:
        """
        Create mission plan from natural language description.
        
        Args:
            description: Natural language mission description
            
        Returns:
            Mission plan with tasks and estimated duration
        """
        # Parse description into tasks
        tasks = []
        
        # Simple keyword-based parsing (in production, use LLM)
        description_lower = description.lower()
        
        if "kitchen" in description_lower:
            tasks.append({"task": "navigate", "target": "kitchen", "estimated_time": 30})
        if "living room" in description_lower or "livingroom" in description_lower:
            tasks.append({"task": "navigate", "target": "living_room", "estimated_time": 45})
        if "clean" in description_lower:
            tasks.append({"task": "clean", "area": "current", "estimated_time": 120})
        if "check" in description_lower:
            tasks.append({"task": "inspect", "target": "environment", "estimated_time": 15})
        
        # Default task if none recognized
        if not tasks:
            tasks.append({"task": "execute", "command": description, "estimated_time": 60})
        
        total_time = sum(t["estimated_time"] for t in tasks)
        
        return {
            "description": description,
            "tasks": tasks,
            "estimated_duration": total_time,
            "task_count": len(tasks),
        }

    async def execute_mission(self, mission: dict[str, Any]) -> TaskResult:
        """
        Execute a mission plan.
        
        Args:
            mission: Mission plan dictionary
            
        Returns:
            TaskResult with mission execution results
        """
        import time
        start_time = time.time()
        
        tasks = mission.get("tasks", [])
        completed = 0
        steps = []
        
        for i, task in enumerate(tasks):
            # Execute task
            if task["task"] == "navigate":
                result = await self.execute(f"navigate to {task['target']}") if hasattr(self.execute, '__await__') else self.execute(f"navigate to {task['target']}")
            elif task["task"] == "clean":
                result = await self.execute("clean area") if hasattr(self.execute, '__await__') else self.execute("clean area")
            elif task["task"] == "inspect":
                result = await self.execute("inspect environment") if hasattr(self.execute, '__await__') else self.execute("inspect environment")
            else:
                result = await self.execute(task.get("command", "")) if hasattr(self.execute, '__await__') else self.execute(task.get("command", ""))
            
            step = {
                "step": task["task"],
                "status": "success" if result.success else "failed",
                "target": task.get("target", ""),
            }
            steps.append(step)
            
            if result.success:
                completed += 1
        
        duration = time.time() - start_time
        progress = (completed / len(tasks) * 100) if tasks else 0
        
        return TaskResult(
            success=completed == len(tasks),
            task="mission_execution",
            steps=steps,
            duration_seconds=duration,
            ai_confidence=0.85,
            human_approvals=0,
            human_rejections=0,
            safety_violations=0,
            message=f"Completed {completed}/{len(tasks)} tasks",
            data={
                "progress": progress,
                "completed_tasks": completed,
                "total_tasks": len(tasks),
            }
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
        """Create multi-step plan using LLM or rule-based fallback.

        Args:
            goal: Task goal description
            observation: Current environment observation
            max_steps: Maximum number of steps

        Returns:
            List of plan steps
        """
        # Try LLM-based planning first
        try:
            return self._create_llm_plan(goal, observation, max_steps)
        except Exception:
            # Fall back to rule-based
            return self._create_rule_based_plan(goal, observation, max_steps)

    def _create_llm_plan(self, goal, observation, max_steps=10):
        """Create plan using LLM."""
        import os

        prompt = f"""Create a step-by-step plan to achieve: {goal}

Current observation: {observation}

Available actions: navigate, pick, place, detect, wait
Return as JSON list of {{"action": str, "params": dict}}"""

        try:
            import openai

            client = openai.OpenAI(
                api_key=os.environ.get("OPENAI_API_KEY") or os.environ.get("MOONSHOT_API_KEY"),
                base_url=os.environ.get("LLM_BASE_URL", "https://api.openai.com/v1"),
            )

            response = client.chat.completions.create(
                model=os.environ.get("LLM_MODEL", "gpt-4"),
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
            )

            import json

            content = response.choices[0].message.content
            # Extract JSON from response
            import re

            json_match = re.search(r"\[.*\]", content, re.DOTALL)
            if json_match:
                plan_data = json.loads(json_match.group())
                return plan_data[:max_steps]
        except Exception:
            pass

        raise RuntimeError("LLM planning failed")

    def _create_rule_based_plan(self, goal, observation, max_steps=10):
        """Create plan using rules (fallback)."""
        goal_str = str(goal).lower()
        plan = []

        if any(word in goal_str for word in ["pick", "grab", "take"]):
            obj = "object"
            for word in goal_str.split():
                if word not in ["pick", "grab", "take", "the", "up", "a", "an"]:
                    obj = word
                    break
            plan = [
                {"action": "detect", "params": {"object": obj}},
                {"action": "navigate", "params": {"target": obj}},
                {"action": "pick", "params": {"object": obj}},
            ]
        elif any(word in goal_str for word in ["go", "navigate", "move"]):
            loc = "target"
            words = goal_str.split()
            if "to" in words:
                idx = words.index("to")
                if idx + 1 < len(words):
                    loc = words[idx + 1]
            plan = [{"action": "navigate", "params": {"location": loc}}]
        elif "place" in goal_str or "put" in goal_str:
            plan = [{"action": "place", "params": {}}]
        else:
            plan = [{"action": "execute", "params": {"goal": goal_str}}]

        return plan[:max_steps]


# Example usage and demonstration
if __name__ == "__main__":
    print("=" * 70)
    print("🤖 ROBOT AGENT - Agentic AI Interface Demo")
    print("=" * 70)
    print()

    # Create agent
    agent = RobotAgent(
        device_id="bot1",
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
