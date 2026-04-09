"""Natural Language Instruction Examples for Agent ROS Bridge.

Demonstrates how AI agents can generate natural language instructions
that Agent ROS Bridge interprets and executes safely.
"""

import asyncio
from dataclasses import dataclass
from typing import Any

from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.tools import ROSTopicEchoTool


@dataclass
class NaturalLanguageCommand:
    """Natural language command with metadata."""
    natural_language: str
    interpreted_action: str
    safety_level: str  # low, medium, high, critical
    requires_confirmation: bool
    generated_by: str  # AI model name


class NaturalLanguageGenerator:
    """Generates natural language instructions for ROS operations.
    
    This class demonstrates how AI agents can generate human-readable
    instructions that Agent ROS Bridge can interpret and execute.
    """
    
    def __init__(self, robot_agent: RobotAgent):
        self.robot_agent = robot_agent
        self.command_history = []
    
    def generate_navigation_command(
        self,
        destination: str,
        urgency: str = "normal",
        context: dict = None
    ) -> NaturalLanguageCommand:
        """Generate natural language navigation command.
        
        Args:
            destination: Where to navigate
            urgency: normal, urgent, or emergency
            context: Additional context for instruction generation
            
        Returns:
            Natural language command with metadata
        """
        context = context or {}
        obstacles = context.get('obstacles', [])
        
        # Generate natural language based on context
        if urgency == "emergency":
            natural_lang = f"Emergency: Navigate immediately to {destination} avoiding all obstacles."
            safety = "critical"
        elif obstacles:
            obstacle_str = ", ".join(obstacles)
            natural_lang = f"Please navigate to {destination}, carefully avoiding {obstacle_str}."
            safety = "high"
        else:
            natural_lang = f"Go to {destination} using the safest path."
            safety = "medium"
        
        return NaturalLanguageCommand(
            natural_language=natural_lang,
            interpreted_action=f"navigate_to({destination})",
            safety_level=safety,
            requires_confirmation=True,
            generated_by="NLG-v1"
        )
    
    def generate_inspection_command(
        self,
        target: str,
        detail_level: str = "summary"
    ) -> NaturalLanguageCommand:
        """Generate natural language inspection command.
        
        Args:
            target: What to inspect (topic, sensor, etc.)
            detail_level: summary, detailed, or diagnostic
            
        Returns:
            Natural language command with metadata
        """
        if detail_level == "diagnostic":
            natural_lang = f"Perform comprehensive diagnostic analysis of {target} including all subsystems."
            safety = "medium"
        elif detail_level == "detailed":
            natural_lang = f"Check {target} and provide detailed readings."
            safety = "low"
        else:
            natural_lang = f"What's the current status of {target}?"
            safety = "low"
        
        return NaturalLanguageCommand(
            natural_language=natural_lang,
            interpreted_action=f"inspect({target}, level={detail_level})",
            safety_level=safety,
            requires_confirmation=False,
            generated_by="NLG-v1"
        )
    
    def generate_manipulation_command(
        self,
        action: str,
        object_name: str,
        location: str = None
    ) -> NaturalLanguageCommand:
        """Generate natural language manipulation command.
        
        Args:
            action: pick, place, grasp, release
            object_name: Object to manipulate
            location: Where to place/pick from
            
        Returns:
            Natural language command with metadata
        """
        if location:
            natural_lang = f"Carefully {action} the {object_name} at {location}."
        else:
            natural_lang = f"{action.capitalize()} the {object_name} safely."
        
        return NaturalLanguageCommand(
            natural_language=natural_lang,
            interpreted_action=f"manipulate(action={action}, object={object_name})",
            safety_level="high",
            requires_confirmation=True,
            generated_by="NLG-v1"
        )
    
    def generate_safety_command(
        self,
        situation: str
    ) -> NaturalLanguageCommand:
        """Generate emergency/safety command.
        
        Args:
            situation: Description of safety concern
            
        Returns:
            Natural language command with metadata
        """
        natural_lang = f"🚨 EMERGENCY: {situation} - STOP ALL MOTION IMMEDIATELY!"
        
        return NaturalLanguageCommand(
            natural_language=natural_lang,
            interpreted_action="emergency_stop()",
            safety_level="critical",
            requires_confirmation=False,  # Emergency bypasses confirmation
            generated_by="NLG-v1"
        )


class NaturalLanguageInterpreter:
    """Interprets natural language into executable commands.
    
    This demonstrates how Agent ROS Bridge processes natural language
    and converts it to safe, executable operations.
    """
    
    def __init__(self, robot_agent: RobotAgent):
        self.robot_agent = robot_agent
        self.topic_tool = ROSTopicEchoTool()
        
        # Natural language patterns
        self.patterns = {
            "navigate": [
                "go to", "navigate to", "move to", "drive to",
                "head to", "proceed to", "travel to", "advance to"
            ],
            "inspect": [
                "check", "inspect", "read", "get", "show me",
                "what's on", "what is", "monitor", "view"
            ],
            "stop": [
                "stop", "halt", "freeze", "abort", "emergency stop"
            ],
            "manipulate": [
                "pick up", "grab", "grasp", "place", "put down",
                "release", "drop", "move arm", "position arm"
            ],
        }
    
    def interpret(self, natural_language: str) -> dict[str, Any]:
        """Interpret natural language command.
        
        Args:
            natural_language: Human-readable command
            
        Returns:
            Interpreted command with safety metadata
        """
        text = natural_language.lower().strip()
        
        # Pattern matching
        for action, patterns in self.patterns.items():
            for pattern in patterns:
                if pattern in text:
                    return self._handle_action(action, text, natural_language)
        
        # Unknown command
        return {
            "success": False,
            "error": f"Command not understood: '{natural_language}'",
            "suggestions": [
                "Try: 'Go to the kitchen'",
                "Try: 'Check /cmd_vel topic'",
                "Try: 'Pick up the cup'",
                "Try: 'Emergency stop'",
            ]
        }
    
    def _handle_action(self, action: str, text: str, original: str) -> dict:
        """Handle matched action."""
        if action == "navigate":
            return self._interpret_navigation(text, original)
        elif action == "inspect":
            return self._interpret_inspection(text, original)
        elif action == "stop":
            return self._interpret_stop(text, original)
        elif action == "manipulate":
            return self._interpret_manipulation(text, original)
        else:
            return {"success": False, "error": f"Unknown action: {action}"}
    
    def _interpret_navigation(self, text: str, original: str) -> dict:
        """Interpret navigation command."""
        # Extract destination
        for pattern in [" to ", " towards ", " into "]:
            if pattern in text:
                destination = text.split(pattern)[-1].strip()
                break
        else:
            destination = "unknown"
        
        # Check for safety keywords
        if "carefully" in text or "safely" in text:
            speed = "slow"
        elif "quickly" in text or "fast" in text:
            speed = "fast"
        else:
            speed = "normal"
        
        return {
            "success": True,
            "action": "navigate",
            "destination": destination,
            "speed": speed,
            "original": original,
            "interpreted": f"Navigate to {destination} at {speed} speed",
            "safety_level": "medium",
            "requires_confirmation": True,
        }
    
    def _interpret_inspection(self, text: str, original: str) -> dict:
        """Interpret inspection command."""
        # Look for topic names
        words = text.split()
        topic = None
        
        for word in words:
            if word.startswith("/"):
                topic = word
                break
        
        if not topic:
            # Check for common topics without /
            common = ["cmd_vel", "odom", "scan", "imu", "battery", "status"]
            for t in common:
                if t in text:
                    topic = f"/{t}"
                    break
        
        topic = topic or "/status"
        
        return {
            "success": True,
            "action": "inspect",
            "target": topic,
            "original": original,
            "interpreted": f"Inspect {topic}",
            "safety_level": "low",
            "requires_confirmation": False,
        }
    
    def _interpret_stop(self, text: str, original: str) -> dict:
        """Interpret stop command."""
        is_emergency = "emergency" in text or "immediately" in text
        
        return {
            "success": True,
            "action": "stop",
            "type": "emergency" if is_emergency else "normal",
            "original": original,
            "interpreted": "Emergency stop" if is_emergency else "Stop",
            "safety_level": "critical" if is_emergency else "high",
            "requires_confirmation": not is_emergency,  # Emergency bypasses
        }
    
    def _interpret_manipulation(self, text: str, original: str) -> dict:
        """Interpret manipulation command."""
        # Extract action
        actions = {
            "pick up": "pick",
            "grab": "grasp",
            "grasp": "grasp",
            "place": "place",
            "put down": "place",
            "release": "release",
            "drop": "release",
        }
        
        action = "unknown"
        for pattern, act in actions.items():
            if pattern in text:
                action = act
                break
        
        return {
            "success": True,
            "action": "manipulate",
            "manipulation_action": action,
            "original": original,
            "interpreted": f"Manipulate: {action}",
            "safety_level": "high",
            "requires_confirmation": True,
        }


# Example usage
async def main():
    """Demo natural language instruction generation and interpretation."""
    
    # Initialize
    robot = RobotAgent(device_id="nl_demo_bot")
    nlg = NaturalLanguageGenerator(robot)
    nli = NaturalLanguageInterpreter(robot)
    
    print("=" * 70)
    print("🗣️  Natural Language Instructions for Agent ROS Bridge")
    print("=" * 70)
    
    # Example 1: Navigation Commands
    print("\n📍 NAVIGATION COMMANDS")
    print("-" * 70)
    
    nav_commands = [
        nlg.generate_navigation_command("kitchen", urgency="normal"),
        nlg.generate_navigation_command("charging station", urgency="urgent"),
        nlg.generate_navigation_command(
            "office",
            context={"obstacles": ["chair", "table"]}
        ),
    ]
    
    for cmd in nav_commands:
        print(f"\n📝 Natural Language: \"{cmd.natural_language}\"")
        print(f"   Interpreted: {cmd.interpreted_action}")
        print(f"   Safety Level: {cmd.safety_level}")
        print(f"   Requires Confirmation: {cmd.requires_confirmation}")
    
    # Example 2: Inspection Commands
    print("\n\n🔍 INSPECTION COMMANDS")
    print("-" * 70)
    
    inspect_commands = [
        nlg.generate_inspection_command("/cmd_vel", detail_level="summary"),
        nlg.generate_inspection_command("/odom", detail_level="detailed"),
        nlg.generate_inspection_command("system", detail_level="diagnostic"),
    ]
    
    for cmd in inspect_commands:
        print(f"\n📝 Natural Language: \"{cmd.natural_language}\"")
        print(f"   Interpreted: {cmd.interpreted_action}")
        print(f"   Safety Level: {cmd.safety_level}")
    
    # Example 3: Manipulation Commands
    print("\n\n🦾 MANIPULATION COMMANDS")
    print("-" * 70)
    
    manip_commands = [
        nlg.generate_manipulation_command("pick up", "cup", "table"),
        nlg.generate_manipulation_command("place", "box", "shelf"),
    ]
    
    for cmd in manip_commands:
        print(f"\n📝 Natural Language: \"{cmd.natural_language}\"")
        print(f"   Interpreted: {cmd.interpreted_action}")
        print(f"   Safety Level: {cmd.safety_level}")
        print(f"   Requires Confirmation: {cmd.requires_confirmation}")
    
    # Example 4: Interpretation Examples
    print("\n\n🧠 INTERPRETATION EXAMPLES")
    print("-" * 70)
    
    user_inputs = [
        "Go to the kitchen carefully",
        "Check /cmd_vel topic",
        "Pick up the cup on the table",
        "Emergency stop!",
        "What's the battery status?",
    ]
    
    for user_input in user_inputs:
        result = nli.interpret(user_input)
        print(f"\n🗣️  User: \"{user_input}\"")
        if result.get("success"):
            print(f"   🤖 Interpreted: {result.get('interpreted')}")
            print(f"   ⚠️  Safety: {result.get('safety_level')}")
            print(f"   ✅ Confirmation: {'Required' if result.get('requires_confirmation') else 'Not required'}")
        else:
            print(f"   ❌ Error: {result.get('error')}")
    
    # Example 5: Adaptive Safety
    print("\n\n🛡️  ADAPTIVE SAFETY EXAMPLES")
    print("-" * 70)
    
    safety_scenarios = [
        ("Go to kitchen", "normal", False),
        ("Go to kitchen NOW", "urgent", True),
        ("Emergency: Go to exit", "emergency", True),
    ]
    
    for nl, urgency, danger in safety_scenarios:
        context = {"danger_present": danger}
        cmd = nlg.generate_navigation_command("location", urgency=urgency, context=context)
        
        print(f"\n📝 \"{nl}\" (urgency={urgency})")
        print(f"   Generated: \"{cmd.natural_language}\"")
        print(f"   Safety Level: {cmd.safety_level}")
        print(f"   Confirmation: {'Required' if cmd.requires_confirmation else 'Bypassed'}")
    
    print("\n" + "=" * 70)
    print("✅ Natural language instruction demo complete!")
    print("=" * 70)
    print("\nKey Features Demonstrated:")
    print("  • Context-aware instruction generation")
    print("  • Adaptive safety based on urgency")
    print("  • Natural language interpretation")
    print("  • Human-in-the-loop enforcement")
    print("  • Emergency bypass for critical situations")


if __name__ == "__main__":
    asyncio.run(main())
