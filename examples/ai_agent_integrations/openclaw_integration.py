"""OpenClaw Integration Example for Agent ROS Bridge.

Demonstrates how OpenClaw agents can use Agent ROS Bridge
to interact with ROS robots via natural language.
"""

import asyncio
import json
from dataclasses import dataclass
from typing import Any, Callable

from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.actions import create_action_client
from agent_ros_bridge.tools import ROSTopicEchoTool, ROSServiceCallTool
from agent_ros_bridge.gateway_v2.core import Command, Message


@dataclass
class OpenClawContext:
    """Context for OpenClaw agent interactions."""
    user_input: str
    robot_agent: RobotAgent
    session_id: str
    command_history: list[dict]


class OpenClawROSBridge:
    """Bridge between OpenClaw and Agent ROS Bridge.
    
    Enables OpenClaw agents to control ROS robots safely.
    """
    
    def __init__(self, robot_agent: RobotAgent = None):
        self.robot_agent = robot_agent or RobotAgent(
            device_id="openclaw_bot",
            require_confirmation=True,
        )
        self.topic_tool = ROSTopicEchoTool()
        self.service_tool = ROSServiceCallTool()
        self.command_history = []
        
        # Natural language command patterns
        self.patterns = {
            "navigate": [
                "go to {location}",
                "navigate to {location}",
                "move to {location}",
                "drive to {location}",
                "head to {location}",
                "proceed to {location}",
            ],
            "inspect": [
                "check {topic}",
                "read {topic}",
                "get {topic} data",
                "show me {topic}",
                "what's on {topic}",
            ],
            "call_service": [
                "call {service}",
                "execute {service}",
                "run {service}",
                "trigger {service}",
            ],
            "status": [
                "status",
                "what's the status",
                "robot status",
                "check status",
                "how is the robot",
            ],
            "emergency": [
                "stop",
                "emergency stop",
                "halt",
                "freeze",
                "abort",
            ],
        }
    
    async def process_natural_language(self, user_input: str) -> dict[str, Any]:
        """Process natural language input and execute ROS commands.
        
        Args:
            user_input: Natural language command from user
            
        Returns:
            Result dictionary with execution details
        """
        user_input_lower = user_input.lower().strip()
        
        # Pattern matching for navigation
        for pattern in self.patterns["navigate"]:
            if self._matches_pattern(user_input_lower, pattern):
                location = self._extract_location(user_input_lower)
                return await self._execute_navigation(location, user_input)
        
        # Pattern matching for inspection
        for pattern in self.patterns["inspect"]:
            if self._matches_pattern(user_input_lower, pattern):
                topic = self._extract_topic(user_input_lower)
                return await self._execute_topic_inspection(topic, user_input)
        
        # Pattern matching for service calls
        for pattern in self.patterns["call_service"]:
            if self._matches_pattern(user_input_lower, pattern):
                service = self._extract_service(user_input_lower)
                return await self._execute_service_call(service, user_input)
        
        # Pattern matching for status
        for pattern in self.patterns["status"]:
            if self._matches_pattern(user_input_lower, pattern):
                return await self._execute_status_check(user_input)
        
        # Pattern matching for emergency
        for pattern in self.patterns["emergency"]:
            if self._matches_pattern(user_input_lower, pattern):
                return await self._execute_emergency_stop(user_input)
        
        # Unknown command
        return {
            "success": False,
            "action": "unknown",
            "input": user_input,
            "error": "Command not recognized. Try: navigate, inspect, call service, status, or emergency stop",
            "suggestions": [
                "Navigate to the kitchen",
                "Check /cmd_vel topic",
                "Call /clear_costmap service",
                "What's the status?",
                "Emergency stop",
            ],
        }
    
    def _matches_pattern(self, input_str: str, pattern: str) -> bool:
        """Check if input matches a pattern."""
        # Simple pattern matching - can be enhanced with NLP
        pattern_base = pattern.replace("{", "").replace("}", "").replace("location", "").replace("topic", "").replace("service", "").strip()
        return pattern_base in input_str or input_str.startswith(pattern_base.split()[0])
    
    def _extract_location(self, input_str: str) -> str:
        """Extract location from natural language."""
        # "go to the kitchen" -> "kitchen"
        # "navigate to position A" -> "position A"
        markers = [" to ", " towards ", " into ", " at "]
        for marker in markers:
            if marker in input_str:
                return input_str.split(marker)[-1].strip()
        return "unknown"
    
    def _extract_topic(self, input_str: str) -> str:
        """Extract topic name from natural language."""
        # Look for topic names starting with /
        words = input_str.split()
        for word in words:
            if word.startswith("/"):
                return word
        # Common topic names without /
        common_topics = ["cmd_vel", "odom", "scan", "imu", "joint_states"]
        for topic in common_topics:
            if topic in input_str:
                return f"/{topic}"
        return "/unknown"
    
    def _extract_service(self, input_str: str) -> str:
        """Extract service name from natural language."""
        words = input_str.split()
        for word in words:
            if word.startswith("/"):
                return word
        return "/unknown"
    
    async def _execute_navigation(self, location: str, original_input: str) -> dict:
        """Execute navigation command."""
        print(f"🤖 Interpreting: '{original_input}' -> Navigate to {location}")
        print(f"⚠️  Safety: Human confirmation required")
        
        # Execute through RobotAgent with safety
        result = self.robot_agent.execute(f"Navigate to {location}")
        
        self.command_history.append({
            "input": original_input,
            "action": "navigate",
            "target": location,
            "result": result.success,
        })
        
        return {
            "success": result.success,
            "action": "navigate",
            "target": location,
            "message": result.message,
            "duration": result.duration_seconds,
            "safety": {
                "human_approvals": result.human_approvals,
                "ai_confidence": result.ai_confidence,
            },
        }
    
    async def _execute_topic_inspection(self, topic: str, original_input: str) -> dict:
        """Execute topic inspection."""
        print(f"🤖 Interpreting: '{original_input}' -> Inspect {topic}")
        
        result = self.topic_tool.execute(topic=topic, count=1)
        
        self.command_history.append({
            "input": original_input,
            "action": "inspect",
            "target": topic,
            "result": result.success,
        })
        
        return {
            "success": result.success,
            "action": "inspect",
            "target": topic,
            "data": result.data,
            "output": result.output if result.success else None,
            "error": result.error if not result.success else None,
        }
    
    async def _execute_service_call(self, service: str, original_input: str) -> dict:
        """Execute service call."""
        print(f"🤖 Interpreting: '{original_input}' -> Call {service}")
        print(f"⚠️  Safety: Human confirmation required")
        
        result = self.service_tool.execute(service=service)
        
        self.command_history.append({
            "input": original_input,
            "action": "call_service",
            "target": service,
            "result": result.success,
        })
        
        return {
            "success": result.success,
            "action": "call_service",
            "target": service,
            "output": result.output if result.success else None,
            "error": result.error if not result.success else None,
        }
    
    async def _execute_status_check(self, original_input: str) -> dict:
        """Execute status check."""
        status = self.robot_agent.get_state()
        safety = self.robot_agent.safety
        
        return {
            "success": True,
            "action": "status",
            "robot": {
                "device_id": status.get("device_id"),
                "device_type": status.get("device_type"),
                "connected": status.get("connected"),
            },
            "safety": {
                "autonomous_mode": safety.autonomous_mode,
                "human_in_the_loop": safety.human_in_the_loop,
                "shadow_mode": safety.shadow_mode_enabled,
                "validation_status": safety.safety_validation_status,
                "shadow_hours": safety.shadow_mode_hours_collected,
                "required_hours": safety.required_shadow_hours,
            },
        }
    
    async def _execute_emergency_stop(self, original_input: str) -> dict:
        """Execute emergency stop."""
        print(f"🚨 EMERGENCY STOP triggered by: '{original_input}'")
        
        # Immediate stop - highest priority
        result = self.robot_agent.execute("STOP")
        
        return {
            "success": result.success,
            "action": "emergency_stop",
            "priority": "CRITICAL",
            "message": "Emergency stop executed",
        }
    
    def get_command_history(self) -> list[dict]:
        """Get history of commands."""
        return self.command_history.copy()
    
    def generate_natural_response(self, result: dict) -> str:
        """Generate natural language response from result."""
        action = result.get("action", "unknown")
        success = result.get("success", False)
        
        if not success:
            error = result.get("error", "Unknown error")
            return f"❌ I couldn't complete that action. {error}"
        
        if action == "navigate":
            target = result.get("target", "unknown")
            duration = result.get("duration", 0)
            return f"✅ I've navigated to {target}. It took {duration:.1f} seconds."
        
        elif action == "inspect":
            target = result.get("target", "unknown")
            return f"✅ I've inspected {target}. Here's what I found:\n{result.get('output', 'No data')}"
        
        elif action == "call_service":
            target = result.get("target", "unknown")
            return f"✅ I've called the {target} service. Result:\n{result.get('output', 'Success')}"
        
        elif action == "status":
            robot = result.get("robot", {})
            safety = result.get("safety", {})
            return f"""
🤖 Robot Status:
- Device: {robot.get('device_id', 'unknown')}
- Safety Mode: {'Autonomous' if safety.get('autonomous_mode') else 'Human Supervised'}
- Shadow Hours: {safety.get('shadow_hours', 0):.1f}/{safety.get('required_hours', 200)}
- Validation: {safety.get('validation_status', 'unknown')}
            """.strip()
        
        elif action == "emergency_stop":
            return "🚨 Emergency stop executed. The robot has been halted."
        
        return f"✅ Action {action} completed successfully."


# Example usage
async def main():
    """Example OpenClaw integration."""
    bridge = OpenClawROSBridge()
    
    # Example natural language commands
    commands = [
        "Go to the kitchen",
        "Navigate to position A",
        "Check /cmd_vel topic",
        "What's the robot's status?",
        "Call /clear_costmap service",
        "Show me /odom data",
        "Emergency stop",
    ]
    
    print("=" * 60)
    print("🦾 OpenClaw + Agent ROS Bridge Integration Demo")
    print("=" * 60)
    
    for command in commands:
        print(f"\n📝 User: \"{command}\"")
        result = await bridge.process_natural_language(command)
        response = bridge.generate_natural_response(result)
        print(f"🤖 OpenClaw: {response}")
    
    print("\n" + "=" * 60)
    print("Command History:")
    for i, cmd in enumerate(bridge.get_command_history(), 1):
        print(f"  {i}. {cmd['input']} -> {cmd['action']} ({'✅' if cmd['result'] else '❌'})")


if __name__ == "__main__":
    asyncio.run(main())
