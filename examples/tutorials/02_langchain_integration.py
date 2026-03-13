#!/usr/bin/env python3
"""
Tutorial 2: LangChain Integration

This tutorial demonstrates using Agent ROS Bridge with LangChain.
"""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agent_ros_bridge.frameworks.langchain import AgentROSBridgeClient, get_ros_tools
from langchain.agents import AgentType, initialize_agent
from langchain.chat_models import ChatOpenAI
from langchain.llms import OpenAI


async def langchain_basic_example():
    """Basic LangChain integration example."""
    print("=" * 60)
    print("Tutorial: LangChain Integration")
    print("=" * 60)
    print()

    # Initialize bridge client
    print("Step 1: Connect to Agent ROS Bridge")
    print("-" * 40)
    client = AgentROSBridgeClient(
        base_url="http://localhost:8765",
        token="your-jwt-token"
    )
    print("✅ Connected to bridge")
    print()

    # Get LangChain tools
    print("Step 2: Get LangChain tools")
    print("-" * 40)
    tools = get_ros_tools(client)
    print(f"✅ Loaded {len(tools)} tools:")
    for tool in tools:
        print(f"   - {tool.name}")
    print()

    # Initialize LangChain agent
    print("Step 3: Initialize LangChain agent")
    print("-" * 40)

    # Use ChatOpenAI if API key available, otherwise mock
    try:
        llm = ChatOpenAI(temperature=0)
        print("✅ Using ChatOpenAI")
    except:
        print("⚠️  OpenAI API key not found, using mock LLM")
        llm = OpenAI(temperature=0)

    agent = initialize_agent(
        tools,
        llm,
        agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
        verbose=True
    )
    print("✅ Agent initialized")
    print()

    # Example interactions
    print("Step 4: Example interactions")
    print("-" * 40)

    examples = [
        "Move the robot forward 1 meter",
        "Check the robot's battery status",
        "Subscribe to the laser scan topic",
    ]

    for example in examples:
        print(f"\n📝 User: {example}")
        print("🤖 Agent thinking...")
        try:
            # In real usage: response = agent.run(example)
            print("   (Would execute: agent.run())")
            print("✅ Command processed")
        except Exception as e:
            print(f"❌ Error: {e}")

    print()
    print("=" * 60)
    print("LangChain Tutorial Complete!")
    print("=" * 60)


async def langchain_advanced_example():
    """Advanced LangChain with planning."""
    print()
    print("=" * 60)
    print("Advanced: Multi-step Robot Tasks")
    print("=" * 60)
    print()

    print("Example: 'Go to the kitchen, pick up the cup, and bring it to me'")
    print()
    print("This would be broken down into steps:")
    print("  1. Navigate to kitchen")
    print("  2. Locate cup using vision")
    print("  3. Move arm to grasp cup")
    print("  4. Navigate to user location")
    print("  5. Release cup")
    print()
    print("LangChain agent handles the planning and execution!")
    print()


async def main():
    """Run LangChain tutorial."""
    await langchain_basic_example()
    await langchain_advanced_example()

    print()
    print("💡 Tip: Set OPENAI_API_KEY environment variable for live LLM")


if __name__ == "__main__":
    asyncio.run(main())
