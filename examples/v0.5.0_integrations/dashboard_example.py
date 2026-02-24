"""Example: Using Agent ROS Bridge Dashboard.

This example shows how to start the web dashboard.
"""

import asyncio
import os
import webbrowser
from agent_ros_bridge import Bridge

os.environ["JWT_SECRET"] = "your-secret-key-here"


async def main():
    """Main example."""
    
    bridge = Bridge()
    
    # Start the bridge
    await bridge.start()
    
    # Get dashboard server
    dashboard = bridge.get_dashboard(port=8080)
    
    # Start dashboard
    await dashboard.start()
    
    print("✅ Dashboard Started")
    print("   URL: http://localhost:8080")
    print("   Features:")
    print("   - Real-time telemetry")
    print("   - Connection status")
    print("   - Emergency stop button")
    print("   - Action history")
    
    # Open browser (optional)
    # webbrowser.open("http://localhost:8080")
    
    print("\n🚀 Press Ctrl+C to stop")
    
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping...")
    
    await dashboard.stop()
    await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
