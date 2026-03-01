"""Dashboard Server - Real-time web UI for monitoring."""

import logging
from datetime import datetime, timezone
from typing import Any, Dict

try:
    from aiohttp import web

    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

logger = logging.getLogger(__name__)


class DashboardServer:
    """Real-time dashboard for monitoring the bridge.

    Provides:
    - Live telemetry
    - Connection status
    - Action history
    - Emergency stop button

    Example:
        from agent_ros_bridge.gateway_v2.core import Bridge
        from agent_ros_bridge.integrations.dashboard_server import DashboardServer

        bridge = Bridge()
        dashboard = DashboardServer(bridge, port=8080)
        await dashboard.start()
    """

    def __init__(self, bridge, port: int = 8080):
        self.bridge = bridge
        self.port = port
        self.app = None
        self.runner = None

        if not AIOHTTP_AVAILABLE:
            logger.warning("aiohttp not available, dashboard disabled")
        else:
            self._setup_routes()

        logger.info(f"DashboardServer initialized (port: {port})")

    def _setup_routes(self):
        """Setup HTTP routes."""
        if not AIOHTTP_AVAILABLE:
            return

        self.app = web.Application()
        self.app.router.add_get("/", self._handle_index)
        self.app.router.add_get("/api/status", self._handle_status)
        self.app.router.add_get("/api/metrics", self._handle_metrics)
        self.app.router.add_post("/api/emergency_stop", self._handle_emergency_stop)

    async def start(self):
        """Start dashboard server."""
        if not AIOHTTP_AVAILABLE:
            logger.warning("Dashboard requires aiohttp")
            return

        self.runner = web.AppRunner(self.app)
        await self.runner.setup()

        site = web.TCPSite(self.runner, "localhost", self.port)
        await site.start()

        logger.info(f"Dashboard running at http://localhost:{self.port}")

    async def stop(self):
        """Stop dashboard server."""
        if self.runner:
            await self.runner.cleanup()
            logger.info("Dashboard stopped")

    async def _handle_index(self, request):
        """Serve main dashboard page."""
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Agent ROS Bridge Dashboard</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                h1 { color: #333; }
                .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
                .ok { background: #d4edda; }
                .error { background: #f8d7da; }
                button { padding: 10px 20px; font-size: 16px; cursor: pointer; }
                .emergency { background: #dc3545; color: white; border: none; }
                .emergency:hover { background: #c82333; }
            </style>
        </head>
        <body>
            <h1>Agent ROS Bridge Dashboard</h1>
            <div id="status">Loading...</div>
            <button class="emergency" onclick="emergencyStop()">EMERGENCY STOP</button>
            <h2>Metrics</h2>
            <pre id="metrics">Loading...</pre>
            
            <script>
                async function updateStatus() {
                    try {
                        const resp = await fetch('/api/status');
                        const data = await resp.json();
                        document.getElementById('status').innerHTML = 
                            '<div class="status ok">Bridge: ' + data.status + '</div>';
                    } catch (e) {
                        document.getElementById('status').innerHTML = 
                            '<div class="status error">Connection Error</div>';
                    }
                }
                
                async function updateMetrics() {
                    try {
                        const resp = await fetch('/api/metrics');
                        const data = await resp.json();
                        document.getElementById('metrics').textContent = JSON.stringify(data, null, 2);
                    } catch (e) {
                        document.getElementById('metrics').textContent = 'Error loading metrics';
                    }
                }
                
                async function emergencyStop() {
                    if (confirm('Are you sure you want to trigger emergency stop?')) {
                        await fetch('/api/emergency_stop', { method: 'POST' });
                        alert('Emergency stop triggered!');
                    }
                }
                
                setInterval(updateStatus, 1000);
                setInterval(updateMetrics, 5000);
                updateStatus();
                updateMetrics();
            </script>
        </body>
        </html>
        """
        return web.Response(text=html, content_type="text/html")

    async def _handle_status(self, request):
        """API: Get bridge status."""
        b = self.bridge
        status = {
            "status": "running" if (b and b.running) else "stopped",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "transports": list(b.transport_manager.transports.keys()) if b else [],
            "fleets": list(b.fleets.keys()) if b else [],
        }
        return web.json_response(status)

    async def _handle_metrics(self, request):
        """API: Get runtime metrics."""
        b = self.bridge
        metrics: Dict[str, Any] = {}
        if b:
            metrics = {
                "running": b.running,
                "transports": {
                    name: {"running": t.running}
                    for name, t in b.transport_manager.transports.items()
                },
                "fleets": {name: {"robot_count": len(f.robots)} for name, f in b.fleets.items()},
                "memory_available": b.memory is not None,
                "safety_available": b.safety is not None,
            }
        return web.json_response(metrics)

    async def _handle_emergency_stop(self, request):
        """API: Trigger emergency stop."""
        if self.bridge and hasattr(self.bridge, "emergency_stop"):
            self.bridge.emergency_stop()
            return web.json_response({"status": "emergency_stop_triggered"})

        return web.json_response({"error": "Emergency stop not available"}, status=500)
