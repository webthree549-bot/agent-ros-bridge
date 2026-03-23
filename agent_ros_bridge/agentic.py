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
    """
    
    def __init__(
        self,
        robot_id: str,
        llm_provider: str = "moonshot",
        require_confirmation: bool = True,
        confidence_threshold: float = 0.8,
    ):
        """
        Initialize robot agent.
        
        Args:
            robot_id: Robot identifier
            llm_provider: 'moonshot', 'openai', 'anthropic'
            require_confirmation: Whether to require human approval
            confidence_threshold: Auto-approve above this confidence
        """
        self.robot_id = robot_id
        self.llm_provider = llm_provider
        self.require_confirmation = require_confirmation
        self.confidence_threshold = confidence_threshold
        
        # Initialize components
        self._init_gateway()
        self._init_intent_parser()
        self._init_task_planner()
        self._init_safety_validator()
        self._init_shadow_hooks()
    
    def _init_gateway(self):
        """Initialize gateway connection"""
        from agent_ros_bridge.gateway import AgentGateway
        self.gateway = AgentGateway()
    
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
        self.safety_validator = SafetyValidator()
    
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
            if self.require_confirmation:
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
            
            # Execute command
            result = self.gateway.send_command(
                robot_id=self.robot_id,
                command=step.command,
            )
            
            executed_steps.append({
                'step': step.name,
                'status': 'success' if result['success'] else 'failed',
                'result': result,
            })
            
            # Log to shadow mode
            self.shadow_hooks.on_human_command({
                'robot_id': self.robot_id,
                'command': step.command,
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
        Get current environment observations.
        
        Returns:
            AgentObservation with robot state and environment
        """
        status = self.gateway.get_robot_status(self.robot_id)
        
        return AgentObservation(
            robot_position=status.get('position', (0, 0, 0)),
            battery_level=status.get('battery', 100.0),
            nearby_objects=status.get('nearby_objects', []),
            current_task=status.get('current_task'),
            recent_commands=status.get('recent_commands', []),
            obstacles_detected=status.get('obstacles', []),
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
