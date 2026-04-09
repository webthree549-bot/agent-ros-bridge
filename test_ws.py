#!/usr/bin/env python3
import asyncio
import websockets

async def test():
    # Disable proxy by using direct connection
    import os
    for k in list(os.environ.keys()):
        if 'proxy' in k.lower():
            del os.environ[k]
    
    try:
        async with websockets.connect("ws://localhost:8765") as ws:
            print("Connected!")
            await ws.send('{"header":{"message_id":"1"},"command":{"action":"get_status"}}')
            response = await asyncio.wait_for(ws.recv(), timeout=5)
            print(f"Response: {response}")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(test())
