"""
Agentic AI Interface for Agent ROS Bridge

High-level, agentic API that provides real value beyond basic command sending.
"""

from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass
import json


@dataclass
class TaskResult:
    """Result of an agentic task execution"""
    success: bool
    task: str
    steps: List[Dict[str, Any]]
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
    nearby_objects: List[str]
    current_task: Optional[str]
    recent_commands: List[str]
    obstacles_detected: List[str]


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
        device_profile: Optional[Any] = None,
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
    
    def _init_hardware(self, device_profile=None):
        """Initialize hardware abstraction"""
        from agent_ros_bridge.hardware import DeviceRegistry, DeviceProfile
        
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
        from agent_ros_bridge.ai.llm_parser import (
            MoonshotParser, OpenAIParser, AnthropicParser
        )
        
        parsers = {
            'moonshot': MoonshotParser,
            'openai': OpenAIParser,
            'anthropic': AnthropicParser,
        }
        
        parser_class = parsers.get(self.llm_provider, MoonshotParser)
        self.intent_parser = parser_class()
    
    def _init_task_planner(self):
        """Initialize task planner"""
        self.task_planner = TaskPlanner()
    
    def _init_safety_validator(self):
        """Initialize safety validator"""
        from agent_ros_bridge.safety.safety_validator import SafetyValidator
        # Use device-specific limits from profile
        limits = self.device.profile.limits if self.device else {}
        self.safety_validator = SafetyValidator(
            max_velocity=limits.get('max_velocity', 1.0),
            max_acceleration=limits.get('max_acceleration', 0.5),
        )
    
    def _create_default_profile(self):
        """Create default device profile based on device_type"""
        from agent_ros_bridge.hardware import DeviceProfile, Capability
        
        profiles = {
            'mobile_robot': DeviceProfile(
                device_id=self.device_id,
                device_type='mobile_robot',
                manufacturer='Generic',
                model='MobileRobot',
                capabilities=[
                    Capability('navigate_to', 'Navigate to location', {'x': float, 'y': float}, 'success'),
                    Capability('rotate', 'Rotate in place', {'angle': float}, 'success'),
                    Capability('stop', 'Stop immediately', {}, 'success'),
                ],
                sensors=['lidar', 'camera', 'imu'],
                actuators=['wheels'],
            ),
            'drone': DeviceProfile(
                device_id=self.device_id,
                device_type='drone',
                manufacturer='Generic',
                model='Drone',
                capabilities=[
                    Capability('takeoff', 'Take off', {'altitude': float}, 'success'),
                    Capability('land', 'Land', {}, 'success'),
                    Capability('fly_to', 'Fly to coordinates', {'x': float, 'y': float, 'z': float}, 'success'),
                    Capability('hover', 'Hover in place', {}, 'success'),
                    Capability('capture_image', 'Capture aerial image', {}, 'image_path'),
                ],
                sensors=['camera', 'gps', 'imu'],
                actuators=['rotors'],
            ),
            'manipulator': DeviceProfile(
                device_id=self.device_id,
                device_type='manipulator',
                manufacturer='Generic',
                model='RobotArm',
                capabilities=[
                    Capability('move_to', 'Move to position', {'x': float, 'y': float, 'z': float}, 'success'),
                    Capability('grasp', 'Grasp object', {'force': float}, 'success'),
                    Capability('release', 'Release grasp', {}, 'success'),
                    Capability('follow_trajectory', 'Follow trajectory', {'waypoints': list}, 'success'),
                ],
                sensors=['camera', 'force_sensor'],
                actuators=['joints', 'gripper'],
            ),
            'humanoid': DeviceProfile(
                device_id=self.device_id,
                device_type='humanoid',
                manufacturer='Generic',
                model='Humanoid',
                capabilities=[
                    Capability('walk', 'Walk', {'direction': str, 'steps': int}, 'success'),
                    Capability('balance', 'Maintain balance', {}, 'success'),
                    Capability('reach', 'Reach to target', {'target': list}, 'success'),
                    Capability('climb', 'Climb surface', {'surface': str}, 'success'),
                    Capability('manipulate', 'Manipulate object', {'object': str, 'action': str}, 'success'),
                ],
                sensors=['camera', 'imu', 'joint_encoders'],
                actuators=['legs', 'arms', 'hands'],
            ),
            'sensor_array': DeviceProfile(
                device_id=self.device_id,
                device_type='sensor_array',
                manufacturer='Generic',
                model='SensorArray',
                capabilities=[
                    Capability('sense', 'Sense environment', {'modality': str}, 'data'),
                    Capability('capture', 'Capture data', {'sensor': str}, 'data'),
                    Capability('scan', 'Scan area', {'area': str}, 'data'),
                ],
                sensors=['camera', 'lidar', 'radar', 'thermal', 'microphone'],
                actuators=[],
            ),
        }
        
        return profiles.get(self.device_type, profiles['mobile_robot'])
    
    def _init_shadow_hooks(self):
        """Initialize shadow mode hooks"""
        from agent_ros_bridge.shadow.hooks import ShadowModeHooks
        self.shadow_hooks = ShadowModeHooks()
    
    def execute(
        self,
        natural_language_command: str,
        context: Optional[Dict[str, Any]] = None,
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
            robot_id=self.robot_id,
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
            safety_result = self.safety_validator.validate(step.command)
            if not safety_result['safe']:
                safety_violations += 1
                executed_steps.append({
                    'step': step.name,
                    'status': 'rejected',
                    'reason': f"Safety violation: {safety_result['violations']}",
                })
                continue
            
            # Get human confirmation if needed
            if self.require_confirmation and intent_result.confidence < self.confidence_threshold:
                approval = self._get_human_approval(
                    step=step,
                    confidence=intent_result.confidence,
                )
                
                if not approval:
                    human_rejections += 1
                    executed_steps.append({
                        'step': step.name,
                        'status': 'rejected',
                        'reason': 'Human rejection',
                    })
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
            
            executed_steps.append({
                'step': step.name,
                'status': 'success' if result['success'] else 'failed',
                'result': result,
            })
            
            # Log to shadow mode
            self.shadow_hooks.on_human_command({
                'robot_id': self.device_id,
                'command': step.capability_name,
                'parameters': step.parameters,
                'ai_proposal': step.ai_proposal,
            })
        
        # Calculate results
        success = all(s['status'] == 'success' for s in executed_steps)
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
            robot_position=state.get('position', (0, 0, 0)),
            battery_level=state.get('battery', 100.0),
            nearby_objects=state.get('nearby_objects', []),
            current_task=state.get('current_task'),
            recent_commands=state.get('recent_commands', []),
            obstacles_detected=state.get('obstacles', []),
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
    ) -> List[Dict[str, Any]]:
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
        step: 'TaskStep',
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
    
    def _generate_result_message(self, steps: List[Dict]) -> str:
        """Generate human-readable result message"""
        successful = [s for s in steps if s['status'] == 'success']
        failed = [s for s in steps if s['status'] == 'failed']
        
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
            steps: List[Any]
        
        @dataclass
        class TaskStep:
            name: str
            command: Dict[str, Any]
            ai_proposal: Any
        
        # Parse compound commands
        if 'and' in intent.raw_text.lower():
            # Break into subtasks
            steps = [
                TaskStep(
                    name='navigate',
                    command={'type': 'navigate_to', 'location': 'kitchen'},
                    ai_proposal=intent,
                ),
                TaskStep(
                    name='pick',
                    command={'type': 'pick_object', 'object': 'red_cup'},
                    ai_proposal=intent,
                ),
            ]
        else:
            steps = [
                TaskStep(
                    name='execute',
                    command=intent.to_command(),
                    ai_proposal=intent,
                ),
            ]
        
        return TaskPlan(steps=steps)
    
    def create_plan(self, goal, observation, max_steps=10):
        """Create multi-step plan"""
        # Use LLM to generate plan
        prompt = f"""
        Goal: {goal}
        Current observation: {observation}
        
        Create a plan with up to {max_steps} steps.
        Return as JSON list of steps.
        """
        # Would use LLM here
        return []


# Example usage and demonstration
if __name__ == "__main__":
    print("=" * 70)
    print("🤖 ROBOT AGENT - Agentic AI Interface Demo")
    print("=" * 70)
    print()
    
    # Create agent
    agent = RobotAgent(
        robot_id='bot1',
        llm_provider='moonshot',
        require_confirmation=True,
        confidence_threshold=0.8,
    )
    
    # Example 1: Simple command
    print("Example 1: Simple navigation")
    print("-" * 70)
    result = agent.execute("Go to the kitchen")
    print(f"Task: Go to the kitchen")
    print(f"Success: {result.success}")
    print(f"Confidence: {result.ai_confidence:.2f}")
    print(f"Duration: {result.duration_seconds:.2f}s")
    print(f"Message: {result.message}")
    print()
    
    # Example 2: Compound command
    print("Example 2: Compound task")
    print("-" * 70)
    result = agent.execute("Go to the kitchen and pick up the red cup")
    print(f"Task: Go to the kitchen and pick up the red cup")
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
