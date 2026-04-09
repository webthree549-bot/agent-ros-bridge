# Experience AI Agent Integration in OpenClaw

**Date:** April 8, 2026  
**Goal:** Practical guide to experiencing Agent ROS Bridge AI capabilities in OpenClaw

---

## 🚀 Quick Start (5 Minutes)

### Step 1: Start OpenClaw with Agent ROS Bridge

```bash
# In your terminal
cd /Users/webthree/.openclaw/workspace

# Ensure Agent ROS Bridge is installed
pip install -e .

# Test the installation
python3 -c "from agent_ros_bridge import RobotAgent; print('✅ Ready')"
```

### Step 2: Launch OpenClaw AI Agent Demo

```bash
# Run the interactive demo
python3 examples/ai_agent_integrations/openclaw_integration_demo.py
```

---

## 🎮 Interactive Demo Script

**File:** `examples/ai_agent_integrations/openclaw_integration_demo.py`

This script creates an interactive OpenClaw session where you can:
- Type natural language commands
- See AI interpretation
- View safety validation
- Execute ROS operations

```python
#!/usr/bin/env python3
"""
OpenClaw Interactive Demo for Agent ROS Bridge

Experience AI agent capabilities directly in OpenClaw.
Type natural language commands and see them executed safely.
"""

import asyncio
import sys
from agent_ros_bridge.agentic import RobotAgent
from examples.ai_agent_integrations.openclaw_integration import OpenClawROSBridge


class OpenClawExperience:
    """Interactive OpenClaw experience for Agent ROS Bridge."""
    
    def __init__(self):
        print("🚀 Starting OpenClaw AI Agent Experience...")
        print("=" * 60)
        
        # Initialize RobotAgent with safety
        self.robot = RobotAgent(
            device_id="openclaw_demo_bot",
            device_type="mobile_robot",
            require_confirmation=True,
        )
        
        # Initialize OpenClaw bridge
        self.bridge = OpenClawROSBridge(self.robot)
        
        print("✅ RobotAgent initialized with safety enabled")
        print(f"   Device: {self.robot.device_id}")
        print(f"   Safety Status: {self.robot.safety.safety_validation_status}")
        print("=" * 60)
    
    async def run_interactive(self):
        """Run interactive session."""
        print("\n📝 Available Commands:")
        print("  Navigation: 'Go to kitchen', 'Navigate to dock 5'")
        print("  Inspection: 'Check /cmd_vel', 'What's the status?'")
        print("  Services:   'Call /clear_costmap'")
        print("  Emergency:  'STOP', 'Emergency stop'")
        print("  Other:      'Help', 'History', 'Quit'")
        print("-" * 60)
        
        while True:
            try:
                # Get user input
                user_input = input("\n🗣️  You: ").strip()
                
                if not user_input:
                    continue
                
                # Handle special commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("\n👋 Goodbye!")
                    break
                
                if user_input.lower() in ['help', 'h']:
                    self._show_help()
                    continue
                
                if user_input.lower() in ['history', 'hist']:
                    self._show_history()
                    continue
                
                # Process through OpenClaw bridge
                print(f"\n🤖 OpenClaw is processing: '{user_input}'")
                print("   🔍 Interpreting natural language...")
                
                result = await self.bridge.process_natural_language(user_input)
                
                # Display interpretation
                print(f"   🎯 Interpreted: {result.get('action', 'unknown')}")
                print(f"   ⚠️  Safety Level: {result.get('safety_level', 'unknown')}")
                
                if result.get('requires_confirmation'):
                    print("   ✋ Human confirmation required")
                    confirm = input("   Approve? (y/n): ").lower().strip()
                    if confirm != 'y':
                        print("   ❌ Cancelled by user")
                        continue
                
                # Generate and display response
                response = self.bridge.generate_natural_response(result)
                print(f"\n📢 Result: {response}")
                
            except KeyboardInterrupt:
                print("\n\n👋 Goodbye!")
                break
            except Exception as e:
                print(f"\n❌ Error: {e}")
    
    def _show_help(self):
        """Show help information."""
        print("\n📚 OpenClaw Agent ROS Bridge Help")
        print("=" * 60)
        print("\nNavigation Commands:")
        print("  'Go to [location]'           - Navigate to a location")
        print("  'Navigate to [location]'     - Navigate with care")
        print("  'Move to [position]'         - Quick navigation")
        print("\nInspection Commands:")
        print("  'Check [topic]'              - Read ROS topic")
        print("  'What's the status?'         - Get robot status")
        print("  'Show me [topic]'            - Display topic data")
        print("\nService Commands:")
        print("  'Call [service]'             - Call ROS service")
        print("  'Execute [service]'          - Execute service")
        print("\nEmergency Commands:")
        print("  'STOP'                       - Emergency stop")
        print("  'Emergency stop'             - Immediate halt")
        print("  'Halt'                       - Stop all motion")
        print("\nOther Commands:")
        print("  'Help'                       - Show this help")
        print("  'History'                    - Show command history")
        print("  'Quit'                       - Exit demo")
        print("=" * 60)
    
    def _show_history(self):
        """Show command history."""
        history = self.bridge.get_command_history()
        
        if not history:
            print("\n📭 No commands in history yet")
            return
        
        print("\n📜 Command History")
        print("=" * 60)
        for i, cmd in enumerate(history, 1):
            status = "✅" if cmd['result'] else "❌"
            print(f"  {i}. {status} {cmd['input']}")
            print(f"     Action: {cmd['action']} → {cmd.get('target', 'N/A')}")
        print("=" * 60)


async def main():
    """Main entry point."""
    demo = OpenClawExperience()
    await demo.run_interactive()


if __name__ == "__main__":
    asyncio.run(main())
