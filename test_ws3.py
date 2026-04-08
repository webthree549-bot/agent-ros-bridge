#!/usr/bin/env python3
import asyncio
import json
import websockets

async def test():
    uri = "ws://127.0.0.1:8765"
    
    try:
        async with websockets.connect(uri) as ws:
            print("✅ Connected to Agent ROS Bridge Gateway")
            print(f"   URI: {uri}\n")
            
            # Test 1: Get status
            print("--- Test 1: Get Status ---")
            cmd = {
                "header": {
                    "message_id": "test-1",
                    "timestamp": "2024-01-01T00:00:00",
                    "source": "openclaw",
                    "target": "ros2_jazzy"
                },
                "command": {
                    "action": "get_status",
                    "parameters": {},
                    "timeout_ms": 5000,
                    "priority": 5
                }
            }
            await ws.send(json.dumps(cmd))
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5)
                data = json.loads(response)
                print(f"✅ Response: {json.dumps(data, indent=2)[:500]}\n")
            except asyncio.TimeoutError:
                print("⏱️ No response (might be OK for get_status)\n")
            
            # Test 2: List robots
            print("--- Test 2: List Robots ---")
            cmd["header"]["message_id"] = "test-2"
            cmd["command"]["action"] = "list_robots"
            await ws.send(json.dumps(cmd))
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=5)
                data = json.loads(response)
                print(f"✅ Response: {json.dumps(data, indent=2)[:500]}\n")
            except asyncio.TimeoutError:
                print("⏱️ No response\n")
                
    except Exception as e:
        print(f"❌ Error: {e}")

asyncio.run(test())
