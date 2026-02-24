"""Example: Using Agent ROS Bridge with LangChain.

This example shows how to control a ROS robot using LangChain agents.
"""

import asyncio
import os
from agent_ros_bridge import Bridge

# Set JWT secret (required)
os.environ["JWT_SECRET"] = "your-secret-key-here"


async def main():
    """Main example."""
    
    # Create bridge with AI features
    bridge = Bridge(config={
        "memory_backend": "sqlite",
        "memory_path": "/tmp/bridge_memory.db"
    })
    
    # Get LangChain tool
    tool = bridge.get_langchain_tool([
        "navigate",
        "move_arm", 
        "get_status"
    ])
    
    print(f"✅ LangChain tool created: {tool.name}")
    print(f"   Description: {tool.description}")
    print(f"   Actions: {tool.get_available_actions()}")
    
    # Example with LangChain agent (requires langchain to be installed)
    try:
        from langchain.agents import initialize_agent, Tool
        from langchain.llms import OpenAI
        
        # Setup LLM (requires OPENAI_API_KEY)
        llm = OpenAI(temperature=0)
        
        # Create agent with ROS tool
        agent = initialize_agent(
            [tool],
            llm,
            agent="zero-shot-react-description",
            verbose=True
        )
        
        # Run agent
        result = await agent.arun("Navigate the robot to position (5, 3) and check status")
        print(f"\n🤖 Agent result: {result}")
        
    except ImportError as e:
        print(f"\n⚠️  LangChain not installed: {e}")
        print("   Install with: pip install langchain openai")
    
    # Cleanup
    await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
