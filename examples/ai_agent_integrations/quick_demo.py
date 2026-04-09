#!/usr/bin/env python3
"""
Quick Demo - Show AI Agent Capabilities in OpenClaw

This script demonstrates the AI agent integration capabilities
without requiring interactive input. Perfect for first-time experience.

Usage:
    python3 quick_demo.py
"""

import asyncio
from agent_ros_bridge.agentic import RobotAgent
from agent_ros_bridge.tools import ROSTopicEchoTool, ROSServiceCallTool
from openclaw_integration import OpenClawROSBridge
from natural_language_examples import NaturalLanguageGenerator, NaturalLanguageInterpreter


class QuickDemo:
    """Quick demonstration of OpenClaw AI capabilities."""
    
    def __init__(self):
        print("=" * 70)
        print("🚀 OpenClaw AI Agent Capabilities Demo")
        print("=" * 70)
        print()
        
        # Initialize components
        self.robot = RobotAgent(
            device_id="demo_bot",
            device_type="mobile_robot",
            require_confirmation=True,
        )
        self.bridge = OpenClawROSBridge(self.robot)
        self.nlg = NaturalLanguageGenerator(self.robot)
        self.nli = NaturalLanguageInterpreter(self.robot)
    
    async def run(self):
        """Run all demo scenarios."""
        
        # Demo 1: Natural Language Generation
        await self._demo_nl_generation()
        
        # Demo 2: Natural Language Interpretation
        await self._demo_nl_interpretation()
        
        # Demo 3: Adaptive Safety
        await self._demo_adaptive_safety()
        
        # Demo 4: Tool Execution
        await self._demo_tools()
        
        # Demo 5: Complete Workflow
        await self._demo_workflow()
        
        # Summary
        self._show_summary()
    
    async def _demo_nl_generation(self):
        """Demo: Natural Language Generation."""
        print("📍 DEMO 1: Natural Language Generation")
        print("-" * 70)
        print("AI agents generate context-aware instructions based on situation.")
        print()
        
        scenarios = [
            {
                "destination": "kitchen",
                "urgency": "normal",
                "context": {},
                "description": "Standard navigation"
            },
            {
                "destination": "charging station",
                "urgency": "urgent",
                "context": {"battery_low": True},
                "description": "Low battery situation"
            },
            {
                "destination": "office",
                "urgency": "normal",
                "context": {"obstacles": ["chair", "table"], "humans_nearby": True},
                "description": "With obstacles and humans"
            },
        ]
        
        for scenario in scenarios:
            cmd = self.nlg.generate_navigation_command(
                destination=scenario["destination"],
                urgency=scenario["urgency"],
                context=scenario["context"]
            )
            
            print(f"Context: {scenario['description']}")
            print(f"  → Generated: \"{cmd.natural_language}\"")
            print(f"  → Safety: {cmd.safety_level}")
            print()
    
    async def _demo_nl_interpretation(self):
        """Demo: Natural Language Interpretation."""
        print("🧠 DEMO 2: Natural Language Interpretation")
        print("-" * 70)
        print("OpenClaw interprets natural language into structured commands.")
        print()
        
        commands = [
            "Go to the kitchen",
            "Navigate to the charging station carefully",
            "Check /cmd_vel topic",
            "What's the robot status?",
            "Pick up the cup on the table",
            "Emergency stop!",
        ]
        
        for cmd_text in commands:
            result = self.nli.interpret(cmd_text)
            
            print(f"🗣️  \"{cmd_text}\"")
            if result.get("success"):
                print(f"   🎯 Action: {result.get('interpreted')}")
                print(f"   ⚠️  Safety: {result.get('safety_level')}")
                print(f"   ✋ Confirmation: {'Required' if result.get('requires_confirmation') else 'Not required'}")
            else:
                print(f"   ❌ Error: {result.get('error')}")
            print()
    
    async def _demo_adaptive_safety(self):
        """Demo: Adaptive Safety Levels."""
        print("🛡️  DEMO 3: Adaptive Safety")
        print("-" * 70)
        print("Safety level adjusts based on command urgency and context.")
        print()
        
        safety_tests = [
            ("Check battery status", "low"),
            ("Go to the kitchen", "medium"),
            ("Go to the kitchen carefully", "medium"),
            ("Go to the kitchen NOW", "high"),
            ("Emergency: Go to exit immediately", "critical"),
        ]
        
        for command, expected_safety in safety_tests:
            result = await self.bridge.process_natural_language(command)
            actual_safety = result.get("safety_level", "unknown")
            
            status = "✅" if actual_safety == expected_safety else "⚠️"
            print(f"{status} \"{command}\"")
            print(f"   Expected: {expected_safety} | Actual: {actual_safety}")
            print(f"   Confirmation: {'Required' if result.get('requires_confirmation') else 'Bypassed'}")
            print()
    
    async def _demo_tools(self):
        """Demo: Tool Execution."""
        print("🔧 DEMO 4: ROS Tool Execution")
        print("-" * 70)
        print("AI agents can use ROS tools through Agent ROS Bridge.")
        print()
        
        # Create tools
        topic_tool = ROSTopicEchoTool()
        service_tool = ROSServiceCallTool()
        
        print("Available Tools:")
        print(f"  📡 {topic_tool.name}: {topic_tool.description}")
        print(f"  📞 {service_tool.name}: {service_tool.description}")
        print()
        
        # Simulate tool execution (will fail gracefully without ROS2)
        print("Simulating tool execution:")
        result = topic_tool.execute(topic="/cmd_vel", count=1)
        print(f"  Topic /cmd_vel: {'✅ Success' if result.success else '❌ Failed (ROS2 not available)'}")
        if not result.success:
            print(f"    Error: {result.error}")
        
        result = service_tool.execute(service="/clear_costmap")
        print(f"  Service /clear_costmap: {'✅ Success' if result.success else '❌ Failed (ROS2 not available)'}")
        if not result.success:
            print(f"    Error: {result.error}")
        print()
    
    async def _demo_workflow(self):
        """Demo: Complete Workflow."""
        print("🎮 DEMO 5: Complete Workflow")
        print("-" * 70)
        print("Full workflow: Natural Language → Interpretation → Safety → Execution")
        print()
        
        workflow_steps = [
            "Step 1: User types natural language command",
            "Step 2: OpenClaw interprets the command",
            "Step 3: Safety level is assessed",
            "Step 4: Human confirmation requested (if needed)",
            "Step 5: Command executed through Agent ROS Bridge",
            "Step 6: Result logged to shadow mode",
            "Step 7: Natural language response generated",
        ]
        
        for step in workflow_steps:
            print(f"  ✅ {step}")
        
        print()
        print("Example workflow:")
        print("  🗣️  User: \"Go to the kitchen carefully\"")
        print("  🤖 OpenClaw: Interpreting...")
        print("  🎯 Interpreted: Navigate to kitchen (slow speed)")
        print("  ⚠️  Safety Level: High (avoid obstacles)")
        print("  ✋ Confirmation: Required")
        print("  👤 Human: Approves")
        print("  ✅ Executed: Robot navigates safely")
        print("  📝 Logged: Shadow mode recorded decision")
        print()
    
    def _show_summary(self):
        """Show summary of capabilities."""
        print("=" * 70)
        print("📊 Summary of OpenClaw AI Agent Capabilities")
        print("=" * 70)
        print()
        
        capabilities = [
            ("Natural Language Generation", "✅", "AI generates context-aware instructions"),
            ("Natural Language Interpretation", "✅", "Understands human commands"),
            ("Adaptive Safety", "✅", "Adjusts based on urgency/context"),
            ("Human-in-the-Loop", "✅", "Requires confirmation for risky actions"),
            ("Shadow Mode Logging", "✅", "Records all decisions for learning"),
            ("ROS Tool Integration", "✅", "Uses rostopic, rosservice, etc."),
            ("Multi-Framework Support", "✅", "Works with LangChain, OpenClaw, Claude"),
            ("Emergency Handling", "✅", "Immediate stop capability"),
        ]
        
        for name, status, description in capabilities:
            print(f"{status} {name}")
            print(f"   {description}")
            print()
        
        print("=" * 70)
        print("🎉 Demo Complete!")
        print("=" * 70)
        print()
        print("Next steps:")
        print("  1. Run interactive demo:")
        print("     python3 openclaw_integration_demo.py")
        print()
        print("  2. Try your own commands:")
        print("     - Navigate to [location]")
        print("     - Check [topic]")
        print("     - What's the status?")
        print()
        print("  3. Read the guide:")
        print("     cat ../../OPENCLAW_DEMO_GUIDE.md")
        print()


async def main():
    """Main entry point."""
    demo = QuickDemo()
    await demo.run()


if __name__ == "__main__":
    asyncio.run(main())
