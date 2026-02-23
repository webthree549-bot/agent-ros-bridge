#!/bin/bash
# run_actions_demo.sh - Run actions demo without Docker

cd "$(dirname "$0")"

echo "⚡ Actions Demo (Native - No Docker)"
echo

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

# Create venv if needed
if [ ! -d ".venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv .venv
fi

source .venv/bin/activate

# Install deps
echo "📦 Installing dependencies..."
pip install -q agent-ros-bridge websockets grpcio grpcio-tools

# Run demo
echo "🚀 Starting Actions Demo..."
echo "🌐 Dashboard: http://localhost:8773"
echo "📡 WebSocket: ws://localhost:8765"
echo

python3 << 'PYTHON_EOF'
import threading
import time
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import asyncio

# HTTP Server
STATIC_DIR = Path(__file__).parent

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"""<!DOCTYPE html>
<html><head><title>Actions Demo</title>
<style>
body{font-family:sans-serif;background:#1a1a2e;color:#fff;padding:40px;max-width:800px;margin:0 auto}
h1{background:linear-gradient(90deg,#00d4ff,#7b2cbf);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.card{background:rgba(255,255,255,0.05);border-radius:16px;padding:24px;margin:20px 0}
.btn{background:linear-gradient(90deg,#00d4ff,#7b2cbf);border:none;border-radius:8px;color:#fff;padding:12px 24px;cursor:pointer}
input{background:rgba(255,255,255,0.1);border:1px solid rgba(255,255,255,0.2);border-radius:6px;color:#fff;padding:10px;margin:5px;width:100px}
#log{background:rgba(0,0,0,0.3);border-radius:8px;padding:16px;font-family:monospace;min-height:200px;max-height:300px;overflow-y:auto}
</style></head>
<body>
<h1>⚡ Actions Demo</h1>
<div class="card">
<h2>🧭 Navigation</h2>
<input type="number" id="x" placeholder="X" value="5"> 
<input type="number" id="y" placeholder="Y" value="3">
<button class="btn" onclick="navigate()">Navigate</button>
</div>
<div class="card">
<h2>📝 Log</h2>
<div id="log">Ready...</div>
</div>
<script>
const log = document.getElementById('log');
function addLog(msg) { log.innerHTML += '<div>' + new Date().toLocaleTimeString() + ' - ' + msg + '</div>'; log.scrollTop = log.scrollHeight; }
async function navigate() {
    const x = document.getElementById('x').value;
    const y = document.getElementById('y').value;
    addLog('Navigating to (' + x + ', ' + y + ')...');
    try {
        const ws = new WebSocket('ws://localhost:8765');
        ws.onopen = () => {
            ws.send(JSON.stringify({command:{action:'actions.navigate',parameters:{x:parseFloat(x),y:parseFloat(y)}}}));
        };
        ws.onmessage = (e) => {
            const data = JSON.parse(e.data);
            addLog('Response: ' + JSON.stringify(data));
            ws.close();
        };
    } catch(e) {
        addLog('Error: ' + e.message);
    }
}
addLog('Dashboard loaded. Enter coordinates and click Navigate.');
</script>
</body></html>""")
        else:
            self.send_error(404)

print("Starting HTTP server on port 8773...")
http_thread = threading.Thread(
    target=lambda: HTTPServer(("0.0.0.0", 8773), DashboardHandler).serve_forever(),
    daemon=True
)
http_thread.start()
time.sleep(1)
print(f"HTTP server running: http://localhost:8773")

# Start bridge
print("Starting ROS Bridge...")
from agent_ros_bridge import ROSBridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport
from agent_ros_bridge.gateway_v2.transports.grpc import GRPCServer
from agent_ros_bridge.gateway_v2.connectors.ros2 import ROS2Connector

async def main():
    bridge = ROSBridge(ros_version=2)
    bridge.transport_manager.register(WebSocketTransport({"host": "0.0.0.0", "port": 8765, "auth": {"enabled": False}}))
    bridge.transport_manager.register(GRPCServer({"host": "0.0.0.0", "port": 50051, "auth": {"enabled": False}}))
    bridge.connector_manager.register(ROS2Connector({"domain_id": 0}))
    
    @bridge.action("actions.navigate")
    async def nav(x, y, theta=0.0):
        print(f"Navigate to ({x}, {y})")
        return {"status": "success", "position": {"x": x, "y": y}}
    
    print("📡 WebSocket: ws://localhost:8765")
    print("🔗 gRPC: localhost:50051")
    print("Press Ctrl+C to stop")
    await bridge.start()

asyncio.run(main())
PYTHON_EOF
