#!/usr/bin/env python3
import asyncio
import websockets

async def test():
    try:
        # Use sockopt to bypass proxy
        async with websockets.connect(
            "ws://127.0.0.1:8765",
            ping_interval=None,
        ) as ws:
            print("✅ Connected to gateway!")
            await ws.send('{"header":{"message_id":"1","timestamp":"2024-01-01T00:00:00","source":"openclaw","target":"ros2"},"command":{"action":"get_status","parameters":{},"timeout_ms":5000,"priority":5}}')
            response = await asyncio.wait_for(ws.recv(), timeout=5)
            print(f"📥 Response: {response[:200]}...")
    except Exception as e:
        print(f"❌ Error: {e}")

asyncio.run(test())
