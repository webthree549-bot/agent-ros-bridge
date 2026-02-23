"""Real-time dashboard for Agent ROS Bridge.

HTTP server with WebSocket for live updates.

Usage:
    from agent_ros_bridge.dashboard import DashboardServer
    
    dashboard = DashboardServer(bridge, port=8080)
    await dashboard.start()
    
    # Open http://localhost:8080
"""

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional
from datetime import datetime
from pathlib import Path

try:
    import aiohttp
    from aiohttp import web
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

logger = logging.getLogger(__name__)


class DashboardServer:
    """Real-time dashboard server.
    
    Features:
    - Live robot telemetry
    - Agent activity stream
    - Action history
    - Topic visualization
    - Emergency stop button
    """
    
    def __init__(self, bridge, port: int = 8080, host: str = "0.0.0.0"):
        """Initialize dashboard server.
        
        Args:
            bridge: ROSBridge instance
            port: HTTP port
            host: Bind host
        """
        if not AIOHTTP_AVAILABLE:
            raise ImportError(
                "Dashboard requires aiohttp: pip install aiohttp"
            )
        
        self.bridge = bridge
        self.port = port
        self.host = host
        self.app = web.Application()
        self.runner = None
        self._websockets: List[web.WebSocketResponse] = []
        self._running = False
        
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup HTTP routes."""
        self.app.router.add_get('/', self.handle_index)
        self.app.router.add_get('/api/status', self.handle_status)
        self.app.router.add_get('/api/actions', self.handle_actions)
        self.app.router.add_get('/api/topics', self.handle_topics)
        self.app.router.add_get('/api/agents', self.handle_agents)
        self.app.router.add_post('/api/emergency_stop', self.handle_emergency_stop)
        self.app.router.add_get('/ws', self.handle_websocket)
        self.app.router.add_static('/static/', path=Path(__file__).parent / 'static', name='static')
    
    async def start(self):
        """Start dashboard server."""
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        
        site = web.TCPSite(self.runner, self.host, self.port)
        await site.start()
        
        self._running = True
        asyncio.create_task(self._broadcast_loop())
        
        logger.info(f"Dashboard started at http://{self.host}:{self.port}")
    
    async def stop(self):
        """Stop dashboard server."""
        self._running = False
        
        # Close all WebSockets
        for ws in self._websockets:
            await ws.close()
        self._websockets.clear()
        
        if self.runner:
            await self.runner.cleanup()
        
        logger.info("Dashboard stopped")
    
    async def handle_index(self, request):
        """Serve main dashboard HTML."""
        html = self._get_dashboard_html()
        return web.Response(text=html, content_type='text/html')
    
    async def handle_status(self, request):
        """API: Get bridge status."""
        status = {
            "bridge": {
                "ros_version": self.bridge.ros_version,
                "uptime": getattr(self.bridge, '_start_time', 0),
            },
            "actions": {
                "registered": len(self.bridge.get_registered_actions()),
                "list": self.bridge.get_registered_actions()
            },
            "agents": {
                "connected": len(self.bridge._sessions),
                "sessions": [
                    {
                        "id": s.session_id,
                        "agent_id": s.agent_id,
                        "connected_at": s.connected_at.isoformat() if hasattr(s.connected_at, 'isoformat') else str(s.connected_at)
                    }
                    for s in self.bridge._sessions.values()
                ]
            },
            "transports": [
                {"type": type(t).__name__}
                for t in self.bridge.transport_manager.get_transports()
            ],
            "connectors": [
                {"type": type(c).__name__}
                for c in self.bridge.connector_manager.get_connectors()
            ]
        }
        return web.json_response(status)
    
    async def handle_actions(self, request):
        """API: Get action history."""
        # This would be stored in a ring buffer
        actions = []  # TODO: Implement action history
        return web.json_response({"actions": actions})
    
    async def handle_topics(self, request):
        """API: Get topic data."""
        topics = self.bridge.get_available_topics()
        topic_data = {}
        for topic in topics:
            data = await self.bridge.get_topic_data(topic)
            topic_data[topic] = data
        return web.json_response({"topics": topic_data})
    
    async def handle_agents(self, request):
        """API: Get connected agents."""
        agents = [
            {
                "session_id": s.session_id,
                "agent_id": s.agent_id,
                "permissions": list(s.permissions)
            }
            for s in self.bridge._sessions.values()
        ]
        return web.json_response({"agents": agents})
    
    async def handle_emergency_stop(self, request):
        """API: Trigger emergency stop."""
        # Trigger emergency stop
        if hasattr(self.bridge, '_safety_manager'):
            self.bridge._safety_manager.emergency_stop()
        
        # Broadcast to all clients
        await self._broadcast({
            "type": "emergency_stop",
            "timestamp": datetime.utcnow().isoformat()
        })
        
        return web.json_response({"status": "emergency_stop_triggered"})
    
    async def handle_websocket(self, request):
        """WebSocket endpoint for live updates."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self._websockets.append(ws)
        logger.info(f"WebSocket client connected. Total: {len(self._websockets)}")
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    await self._handle_ws_message(ws, data)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {ws.exception()}")
        finally:
            self._websockets.remove(ws)
            logger.info(f"WebSocket client disconnected. Total: {len(self._websockets)}")
        
        return ws
    
    async def _handle_ws_message(self, ws: web.WebSocketResponse, data: Dict):
        """Handle incoming WebSocket message."""
        msg_type = data.get('type')
        
        if msg_type == 'subscribe':
            # Client subscribing to updates
            channel = data.get('channel')
            await ws.send_json({"type": "subscribed", "channel": channel})
        
        elif msg_type == 'ping':
            await ws.send_json({"type": "pong"})
    
    async def _broadcast_loop(self):
        """Broadcast updates to all WebSocket clients."""
        while self._running:
            try:
                if self._websockets:
                    update = await self._get_status_update()
                    await self._broadcast(update)
                
                await asyncio.sleep(1)  # Update every second
            except Exception as e:
                logger.error(f"Broadcast error: {e}")
                await asyncio.sleep(1)
    
    async def _get_status_update(self) -> Dict:
        """Get current status for broadcasting."""
        return {
            "type": "status_update",
            "timestamp": datetime.utcnow().isoformat(),
            "data": {
                "connected_agents": len(self.bridge._sessions),
                "registered_actions": len(self.bridge.get_registered_actions()),
                "available_topics": len(self.bridge.get_available_topics())
            }
        }
    
    async def _broadcast(self, message: Dict):
        """Broadcast message to all WebSocket clients."""
        disconnected = []
        
        for ws in self._websockets:
            try:
                await ws.send_json(message)
            except Exception:
                disconnected.append(ws)
        
        # Remove disconnected clients
        for ws in disconnected:
            if ws in self._websockets:
                self._websockets.remove(ws)
    
    def _get_dashboard_html(self) -> str:
        """Get dashboard HTML."""
        return '''<!DOCTYPE html>
<html>
<head>
    <title>Agent ROS Bridge - Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #0a0a0f;
            color: #fff;
            padding: 2rem;
        }
        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 2rem;
            padding-bottom: 1rem;
            border-bottom: 1px solid rgba(255,255,255,0.1);
        }
        h1 { font-size: 1.5rem; background: linear-gradient(90deg, #00d4ff, #7b2cbf); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
        .emergency-btn {
            background: #ff4444;
            color: white;
            border: none;
            padding: 0.75rem 1.5rem;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            text-transform: uppercase;
        }
        .emergency-btn:hover { background: #ff2222; }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 1.5rem;
        }
        .card {
            background: rgba(255,255,255,0.03);
            border: 1px solid rgba(255,255,255,0.08);
            border-radius: 12px;
            padding: 1.5rem;
        }
        .card h3 { color: #00d4ff; margin-bottom: 1rem; font-size: 0.875rem; text-transform: uppercase; }
        .stat { display: flex; justify-content: space-between; margin-bottom: 0.5rem; }
        .stat-value { font-family: monospace; color: #00d4ff; }
        .status { display: inline-flex; align-items: center; gap: 0.5rem; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; background: #00ff88; }
        .status-dot.warning { background: #ffaa00; }
        .status-dot.error { background: #ff4444; }
        .log { background: rgba(0,0,0,0.3); border-radius: 6px; padding: 1rem; font-family: monospace; font-size: 0.75rem; max-height: 200px; overflow-y: auto; }
        .log-entry { margin-bottom: 0.25rem; opacity: 0.8; }
        .connected { color: #00ff88; }
        .disconnected { color: #ff4444; }
    </style>
</head>
<body>
    <header>
        <h1>🤖 Agent ROS Bridge Dashboard</h1>
        <button class="emergency-btn" onclick="emergencyStop()">🛑 Emergency Stop</button>
    </header>
    
    <div class="grid">
        <div class="card">
            <h3>Bridge Status</h3>
            <div class="stat"><span>ROS Version</span><span class="stat-value" id="ros-version">-</span></div>
            <div class="stat"><span>Status</span><span class="status"><span class="status-dot" id="status-dot"></span><span id="status-text">Connecting...</span></span></div>
            <div class="stat"><span>Uptime</span><span class="stat-value" id="uptime">-</span></div>
        </div>
        
        <div class="card">
            <h3>Connected Agents</h3>
            <div class="stat"><span>Active Sessions</span><span class="stat-value" id="agent-count">0</span></div>
            <div id="agent-list"></div>
        </div>
        
        <div class="card">
            <h3>Actions</h3>
            <div class="stat"><span>Registered</span><span class="stat-value" id="action-count">0</span></div>
            <div class="log" id="action-log"></div>
        </div>
        
        <div class="card">
            <h3>Topics</h3>
            <div class="stat"><span>Available</span><span class="stat-value" id="topic-count">0</span></div>
            <div id="topic-list"></div>
        </div>
    </div>
    
    <script>
        const ws = new WebSocket(`ws://${window.location.host}/ws`);
        
        ws.onopen = () => {
            document.getElementById('status-text').textContent = 'Connected';
            document.getElementById('status-dot').classList.add('connected');
            loadStatus();
        };
        
        ws.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            if (msg.type === 'status_update') {
                updateStatus(msg.data);
            }
        };
        
        ws.onclose = () => {
            document.getElementById('status-text').textContent = 'Disconnected';
            document.getElementById('status-dot').classList.remove('connected');
            document.getElementById('status-dot').classList.add('error');
        };
        
        async function loadStatus() {
            const res = await fetch('/api/status');
            const data = await res.json();
            updateStatus(data);
        }
        
        function updateStatus(data) {
            document.getElementById('ros-version').textContent = 'ROS' + data.bridge.ros_version;
            document.getElementById('action-count').textContent = data.actions.registered;
            document.getElementById('agent-count').textContent = data.agents.connected;
            document.getElementById('topic-count').textContent = data.topics?.length || 0;
        }
        
        function emergencyStop() {
            if (confirm('Are you sure you want to trigger EMERGENCY STOP?')) {
                fetch('/api/emergency_stop', { method: 'POST' });
            }
        }
        
        setInterval(loadStatus, 5000);
    </script>
</body>
</html>'''


__all__ = ["DashboardServer"]
