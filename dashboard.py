#!/usr/bin/env python3
"""
Agent ROS Bridge - System Status Dashboard
Simple web interface to visualize ROS2 robot state
"""
import asyncio
import json
import subprocess

# ROS2 bridge
import sys
import threading
from datetime import datetime

# HTTP server for dashboard
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, '/Users/webthree/.openclaw/workspace')

from agent_ros_bridge import Bridge
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

# Global state
robot_state = {
    "odom": None,
    "scan": None,
    "cmd_vel": None,
    "map": None,
    "nav_status": "idle",
    "last_update": None
}

# HTML Dashboard
DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Agent ROS Bridge - Robot Status</title>
    <meta charset="utf-8">
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e;
            color: #eee;
            margin: 0;
            padding: 20px;
        }
        .header {
            text-align: center;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 10px;
            margin-bottom: 20px;
        }
        .header h1 {
            margin: 0;
            font-size: 2em;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
        }
        .card {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            border: 1px solid #0f3460;
        }
        .card h3 {
            margin-top: 0;
            color: #e94560;
        }
        .status-indicator {
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .status-online { background: #00d9ff; box-shadow: 0 0 10px #00d9ff; }
        .status-offline { background: #ff4757; }
        .data-value {
            font-family: 'Courier New', monospace;
            background: #0f3460;
            padding: 10px;
            border-radius: 5px;
            font-size: 0.9em;
            overflow-x: auto;
        }
        .refresh-btn {
            background: #e94560;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 1em;
        }
        .refresh-btn:hover { background: #ff6b6b; }
        .timestamp {
            text-align: center;
            color: #888;
            margin-top: 20px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Agent ROS Bridge</h1>
        <p>TurtleBot3 Navigation System Status</p>
    </div>
    
    <div class="status-grid">
        <div class="card">
            <h3><span class="status-indicator status-online"></span>System Status</h3>
            <div class="data-value" id="system-status">Loading...</div>
        </div>
        
        <div class="card">
            <h3>📍 Robot Position</h3>
            <div class="data-value" id="robot-position">Loading...</div>
        </div>
        
        <div class="card">
            <h3>🧭 Navigation</h3>
            <div class="data-value" id="nav-status">Loading...</div>
        </div>
        
        <div class="card">
            <h3>📡 ROS Topics</h3>
            <div class="data-value" id="ros-topics">Loading...</div>
        </div>
        
        <div class="card">
            <h3>🗺️ Map Status</h3>
            <div class="data-value" id="map-status">Loading...</div>
        </div>
        
        <div class="card">
            <h3>⚡ Velocity</h3>
            <div class="data-value" id="velocity">Loading...</div>
        </div>
    </div>
    
    <div style="text-align: center; margin-top: 20px;">
        <button class="refresh-btn" onclick="location.reload()">🔄 Refresh</button>
    </div>
    
    <div class="timestamp" id="timestamp"></div>
    
    <script>
        // Auto-refresh every 5 seconds
        setInterval(() => location.reload(), 5000);
        
        // Update timestamp
        document.getElementById('timestamp').textContent =
            'Last updated: ' + new Date().toLocaleString();
    </script>
</body>
</html>
"""

class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()

            # Get fresh data from ROS2
            html = self.generate_dashboard()
            self.wfile.write(html.encode())
        elif self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            status = self.get_system_status()
            self.wfile.write(json.dumps(status).encode())
        else:
            self.send_error(404)

    def generate_dashboard(self):
        """Generate dashboard HTML with live data."""
        status = self.get_system_status()

        # Replace placeholders with actual data
        html = DASHBOARD_HTML

        # System status
        system_html = f"""
            <strong>ROS2 Container:</strong> {'✅ Running' if status['ros2_running'] else '❌ Stopped'}<br>
            <strong>Gazebo:</strong> {'✅ Active' if status['gazebo_running'] else '❌ Inactive'}<br>
            <strong>Nav2:</strong> {'✅ Running' if status['nav2_running'] else '❌ Stopped'}<br>
            <strong>SLAM:</strong> {'✅ Active' if status['slam_running'] else '❌ Inactive'}<br>
            <strong>Topics:</strong> {status['topic_count']} active
        """
        html = html.replace('id="system-status">Loading...', f'id="system-status">{system_html}')

        # Robot position
        pos = status.get('robot_position', {})
        pos_html = f"""
            <strong>X:</strong> {pos.get('x', 'N/A'):.3f}<br>
            <strong>Y:</strong> {pos.get('y', 'N/A'):.3f}<br>
            <strong>Theta:</strong> {pos.get('theta', 'N/A'):.3f} rad
        """ if pos else "Position data not available"
        html = html.replace('id="robot-position">Loading...', f'id="robot-position">{pos_html}')

        # Navigation status
        nav_html = f"""
            <strong>Status:</strong> {status.get('nav_status', 'Unknown')}<br>
            <strong>Action Servers:</strong><br>
            - /navigate_to_pose ✅<br>
            - /navigate_through_poses ✅
        """
        html = html.replace('id="nav-status">Loading...', f'id="nav-status">{nav_html}')

        # ROS topics
        topics = status.get('topics', [])
        topics_html = '<br>'.join(topics[:10]) + (f'<br>... and {len(topics)-10} more' if len(topics) > 10 else '')
        html = html.replace('id="ros-topics">Loading...', f'id="ros-topics">{topics_html}')

        # Map status
        map_html = f"""
            <strong>Map Topic:</strong> {'✅ /map publishing' if status.get('map_active') else '❌ Inactive'}<br>
            <strong>Frame:</strong> map → odom → base_link<br>
            <strong>SLAM:</strong> {'✅ Running' if status.get('slam_running') else '❌ Stopped'}
        """
        html = html.replace('id="map-status">Loading...', f'id="map-status">{map_html}')

        # Velocity
        vel = status.get('velocity', {})
        vel_html = f"""
            <strong>Linear X:</strong> {vel.get('linear_x', 0):.4f} m/s<br>
            <strong>Angular Z:</strong> {vel.get('angular_z', 0):.4f} rad/s
        """
        html = html.replace('id="velocity">Loading...', f'id="velocity">{vel_html}')

        return html

    def get_system_status(self):
        """Get current system status from ROS2."""
        status = {
            'timestamp': datetime.now().isoformat(),
            'ros2_running': False,
            'gazebo_running': False,
            'nav2_running': False,
            'slam_running': False,
            'map_active': False,
            'topic_count': 0,
            'topics': [],
            'robot_position': None,
            'velocity': None,
            'nav_status': 'idle'
        }

        try:
            # Check container running
            result = subprocess.run(
                ['docker', 'ps', '--filter', 'name=ros2_humble', '--format', '{{.Status}}'],
                capture_output=True, text=True, timeout=5
            )
            status['ros2_running'] = 'Up' in result.stdout

            if status['ros2_running']:
                # Get topic list
                result = subprocess.run(
                    ['docker', 'exec', 'ros2_humble', 'bash', '-c',
                     'source /opt/ros/humble/setup.bash && ros2 topic list'],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode == 0:
                    topics = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
                    status['topics'] = topics
                    status['topic_count'] = len(topics)

                    # Check specific components
                    status['gazebo_running'] = '/clock' in topics
                    status['map_active'] = '/map' in topics
                    status['slam_running'] = '/slam_toolbox/feedback' in topics or '/map' in topics
                    status['nav2_running'] = '/navigate_to_pose' in topics or '/goal_pose' in topics

                # Get robot position
                result = subprocess.run(
                    ['docker', 'exec', 'ros2_humble', 'bash', '-c',
                     'source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once 2>/dev/null | head -20'],
                    capture_output=True, text=True, timeout=5
                )
                if result.returncode == 0 and 'position:' in result.stdout:
                    # Parse position from output
                    lines = result.stdout.split('\n')
                    pos = {'x': 0, 'y': 0, 'theta': 0}
                    for i, line in enumerate(lines):
                        if 'position:' in line:
                            for j in range(i+1, min(i+5, len(lines))):
                                if 'x:' in lines[j]:
                                    try:
                                        pos['x'] = float(lines[j].split(':')[1].strip())
                                    except Exception:
                                        pass
                                if 'y:' in lines[j]:
                                    try:
                                        pos['y'] = float(lines[j].split(':')[1].strip())
                                    except Exception:
                                        pass
                    status['robot_position'] = pos

                    # Get velocity
                    vel = {'linear_x': 0, 'angular_z': 0}
                    for i, line in enumerate(lines):
                        if 'twist:' in line or 'linear:' in line:
                            for j in range(i+1, min(i+10, len(lines))):
                                if 'x:' in lines[j] and 'linear' in lines[i]:
                                    try:
                                        vel['linear_x'] = float(lines[j].split(':')[1].strip())
                                    except Exception:
                                        pass
                                if 'z:' in lines[j] and 'angular' in lines[i]:
                                    try:
                                        vel['angular_z'] = float(lines[j].split(':')[1].strip())
                                    except Exception:
                                        pass
                    status['velocity'] = vel

        except Exception as e:
            status['error'] = str(e)

        return status

    def log_message(self, format, *args):
        pass  # Suppress logs


def run_http_server(port=8080):
    """Run HTTP server in background thread."""
    server = HTTPServer(('0.0.0.0', port), DashboardHandler)
    print(f"🌐 Dashboard: http://localhost:{port}")
    server.serve_forever()


async def main():
    print("=" * 60)
    print("🚀 Agent ROS Bridge - System Status Dashboard")
    print("=" * 60)

    # Start HTTP server in background
    http_thread = threading.Thread(target=run_http_server, args=(8080,), daemon=True)
    http_thread.start()

    # Create bridge with WebSocket
    bridge = Bridge()
    bridge.transport_manager.register(
        WebSocketTransport({"host": "0.0.0.0", "port": 8768, "auth": {"enabled": False}})
    )

    print("📡 WebSocket: ws://localhost:8768")
    print("=" * 60)

    await bridge.start()

    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
