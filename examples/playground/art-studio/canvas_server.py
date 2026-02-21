"""
Canvas Server - WebSocket server for real-time canvas updates
"""
import asyncio
import json
import websockets
from aiohttp import web
import os

# Store connected clients
clients = set()
canvas_state = []

async def register(websocket):
    clients.add(websocket)
    print(f"Client connected. Total: {len(clients)}")

async def unregister(websocket):
    clients.remove(websocket)
    print(f"Client disconnected. Total: {len(clients)}")

async def broadcast(message):
    if clients:
        await asyncio.gather(
            *[client.send(json.dumps(message)) for client in clients],
            return_exceptions=True
        )

async def websocket_handler(websocket, path):
    await register(websocket)
    try:
        async for message in websocket:
            data = json.loads(message)
            # Broadcast to all clients
            await broadcast(data)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        await unregister(websocket)

async def http_handler(request):
    """Serve the canvas HTML"""
    html_path = '/app/canvas.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    
    # Debug info
    files = os.listdir('/app')
    return web.Response(
        text=f"Canvas HTML not found at {html_path}. Files in /app: {files}", 
        status=404
    )

async def main():
    # WebSocket server
    ws_server = await websockets.serve(
        websocket_handler, 
        "0.0.0.0", 
        8766,
        ping_interval=20,
        ping_timeout=10
    )
    
    # HTTP server for canvas
    app = web.Application()
    app.router.add_get('/', http_handler)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8081)
    await site.start()
    
    print("🎨 Canvas Server started")
    print("  WebSocket: ws://localhost:8766")
    print("  HTTP: http://localhost:8081")
    
    await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())