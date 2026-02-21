#!/usr/bin/env python3
"""Mars HTTP Server"""
import asyncio
from aiohttp import web
import os

async def index(request):
    html_path = '/ros2_ws/dashboard.html'
    if os.path.exists(html_path):
        with open(html_path) as f:
            return web.Response(text=f.read(), content_type='text/html')
    return web.Response(text="dashboard.html not found", status=404)

async def main():
    app = web.Application()
    app.router.add_get('/', index)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()
    print('🚀 Mars HTTP Server: http://localhost:8080')
    await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())