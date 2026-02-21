"""
Garden Server - Web server for The Talking Garden UI
"""
import asyncio
from aiohttp import web
import os

async def index_handler(request):
    """Serve the garden HTML"""
    html_path = '/app/garden.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    
    files = os.listdir('/app')
    return web.Response(
        text=f"Garden HTML not found. Files: {files}", 
        status=404
    )

async def main():
    app = web.Application()
    app.router.add_get('/', index_handler)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    
    print("🌱 Garden Server started at http://localhost:8080")
    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())