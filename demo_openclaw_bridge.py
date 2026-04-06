#!/usr/bin/env python3
"""Demo script for OpenClaw Bridge with Agent ROS Bridge.

Run this to demonstrate real-time bidirectional communication.
"""

import asyncio
import json
import sys
from datetime import datetime

# Demo configuration
BRIDGE_PORT = 8766
AGENT_ROS_PORT = 8765


class DemoClient:
    """Simulated client for demo purposes."""
    
    def __init__(self, client_type: str, name: str):
        self.client_type = client_type
        self.name = name
        self.messages_received = 0
        
    async def connect_and_demo(self):
        """Run demo sequence."""
        try:
            import websockets
        except ImportError:
            print("❌ websockets not installed. Run: pip install websockets")
            return
            
        uri = f"ws://localhost:{BRIDGE_PORT}"
        
        print(f"\n{'='*60}")
        print(f"🎭 DEMO: {self.name} ({self.client_type})")
        print(f"{'='*60}")
        print(f"Connecting to {uri}...")
        
        try:
            async with websockets.connect(uri) as websocket:
                # Identify
                await websocket.send(json.dumps({
                    "client_type": self.client_type
                }))
                print(f"✅ Connected as {self.client_type}")
                
                if self.client_type == "openclaw":
                    await self._demo_openclaw(websocket)
                else:
                    await self._demo_web_ui(websocket)
                    
        except ConnectionRefusedError:
            print(f"❌ Could not connect. Is the bridge running?")
            print(f"   Start it with: ./start_openclaw_bridge.sh")
        except Exception as e:
            print(f"❌ Error: {e}")
            
    async def _demo_openclaw(self, websocket):
        """Demo OpenClaw agent behavior."""
        import websockets
        
        print("\n📤 Sending commands to robot...\n")
        
        # Command 1: Navigate
        print("1️⃣  Command: Navigate to kitchen")
        await websocket.send(json.dumps({
            "action": "command",
            "command": "navigate to kitchen",
            "parameters": {"speed": "normal"}
        }))
        
        # Wait for response
        try:
            response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            data = json.loads(response)
            print(f"   ⬅️  Response: {data.get('type', 'unknown')}")
            if data.get('type') == 'command_result':
                print(f"   ✅ Command executed successfully!")
        except asyncio.TimeoutError:
            print("   ⏱️  Waiting for command result...")
            
        # Subscribe to telemetry
        print("\n2️⃣  Subscribing to telemetry stream...")
        await websocket.send(json.dumps({
            "action": "subscribe"
        }))
        
        # Listen for telemetry updates
        print("\n📡 Listening for telemetry (5 seconds)...\n")
        start_time = asyncio.get_event_loop().time()
        
        while asyncio.get_event_loop().time() - start_time < 5:
            try:
                message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                data = json.loads(message)
                self.messages_received += 1
                
                if data.get('type') == 'telemetry_update':
                    telem = data.get('data', {})
                    battery = telem.get('battery', 'N/A')
                    pos = telem.get('position', {})
                    print(f"   📊 Telemetry #{self.messages_received}: "
                          f"Battery={battery}%, Pos=({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})")
                          
                elif data.get('type') == 'command_result':
                    print(f"   ✅ Command result received!")
                    
            except asyncio.TimeoutError:
                print(f"   ⏱️  No update this second...")
                
        print(f"\n📈 Total messages received: {self.messages_received}")
        
    async def _demo_web_ui(self, websocket):
        """Demo Web UI behavior."""
        import websockets
        
        print("\n🖥️  Web UI initialized")
        print("   Waiting for state broadcast...\n")
        
        # Wait for initial state
        try:
            message = await asyncio.wait_for(websocket.recv(), timeout=3.0)
            data = json.loads(message)
            
            if data.get('type') == 'state':
                clients = data.get('connected_clients', {})
                print(f"   📊 Connected clients: {clients}")
                print(f"   🤖 Robot telemetry: {data.get('telemetry', {})}")
                
        except asyncio.TimeoutError:
            print("   ⏱️  No state received (robot may not be connected)")
            
        # Send a command
        print("\n📤 Sending robot command from Web UI...")
        await websocket.send(json.dumps({
            "action": "execute",
            "command": "get_status"
        }))
        
        # Wait for result
        try:
            message = await asyncio.wait_for(websocket.recv(), timeout=3.0)
            data = json.loads(message)
            print(f"   ⬅️  Response type: {data.get('type')}")
        except asyncio.TimeoutError:
            print("   ⏱️  No response (Agent ROS Bridge may not be running)")


async def demo_simultaneous():
    """Demo both clients simultaneously."""
    print("\n" + "="*60)
    print("🚀 OPENCLAW BRIDGE DEMO")
    print("="*60)
    print("\nThis demo shows:")
    print("  ✅ Real-time bidirectional communication")
    print("  ✅ OpenClaw + Web UI synchronization")
    print("  ✅ Command forwarding and telemetry streaming")
    print("  ✅ Multi-client support")
    
    # Create clients
    openclaw = DemoClient("openclaw", "OpenClaw Agent")
    web_ui = DemoClient("web_ui", "Web Dashboard")
    
    # Run both demos
    await asyncio.gather(
        openclaw.connect_and_demo(),
        web_ui.connect_and_demo(),
        return_exceptions=True
    )
    
    print("\n" + "="*60)
    print("✅ DEMO COMPLETE")
    print("="*60)


def main():
    """Main entry point."""
    print("\n" + "="*60)
    print("🔌 OPENCLAW BRIDGE DEMO")
    print("="*60)
    
    # Check if bridge is running
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    result = sock.connect_ex(('localhost', BRIDGE_PORT))
    sock.close()
    
    if result != 0:
        print(f"\n❌ OpenClaw Bridge is NOT running on port {BRIDGE_PORT}")
        print(f"\n   Start it first with:")
        print(f"   ./start_openclaw_bridge.sh")
        print(f"\n   Or manually:")
        print(f"   python3 skills/agent-ros-bridge/scripts/openclaw_bridge.py")
        sys.exit(1)
    
    print(f"\n✅ OpenClaw Bridge detected on port {BRIDGE_PORT}")
    
    # Run demo
    try:
        asyncio.run(demo_simultaneous())
    except KeyboardInterrupt:
        print("\n\n🛑 Demo interrupted by user")


if __name__ == "__main__":
    main()
