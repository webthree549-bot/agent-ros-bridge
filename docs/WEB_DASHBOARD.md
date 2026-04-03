# Web Dashboard Setup Guide

Complete guide for setting up and using the Agent ROS Bridge Web Dashboard.

## Overview

The web dashboard provides a modern, intuitive interface for:
- **Robot Control** - Manual control, natural language commands
- **Real-time Monitoring** - Telemetry, LiDAR, sensor data
- **Shadow Mode Analytics** - AI-human agreement metrics
- **Safety Monitoring** - Validation gates, deployment status
- **Fleet Management** - Multi-robot coordination

## Architecture

```
┌─────────────────┐     HTTP      ┌─────────────────┐
│   Browser       │ ◄────────────► │  Web Dashboard  │
│   (User)        │                │  (nginx:8081)   │
└─────────────────┘                └────────┬────────┘
                                            │
                                            │ WebSocket
                                            ▼
                                    ┌─────────────────┐
                                    │  Agent ROS      │
                                    │  Bridge         │
                                    │  (port 8765)    │
                                    └────────┬────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │  Robot Fleet    │
                                    │  (ROS/Gazebo)   │
                                    └─────────────────┘
```

## Quick Start

### Prerequisites

- Docker and Docker Compose (recommended)
- Or Python 3.11+ for host-based setup
- Agent ROS Bridge running (port 8765)

### Option 1: Docker Compose (Recommended)

```bash
# 1. Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# 2. Start bridge + web dashboard
docker-compose --profile web up -d

# 3. Access dashboard
open http://localhost:8081
# Or: xdg-open http://localhost:8081 (Linux)
```

### Option 2: Host-Based Development

```bash
# 1. Install Agent ROS Bridge
pip install agent-ros-bridge

# 2. Start the bridge (in terminal 1)
export JWT_SECRET=$(openssl rand -base64 32)
agent-ros-bridge --websocket-port 8765 --host 0.0.0.0

# 3. Serve dashboard (in terminal 2)
cd agent_ros_bridge/web
python3 -m http.server 8081

# 4. Access dashboard
open http://localhost:8081
```

### Option 3: ROS2 Simulation Stack

```bash
# Start ROS2 container with Gazebo
./scripts/docker/docker-manager.sh start

# Dashboard is already running on port 8080 (3D visualization)
# Or start the new dashboard on port 8081:
docker-compose --profile web up -d web-dashboard
```

## Configuration

### Environment Variables

Create `.env` file:

```bash
# Bridge Configuration
JWT_SECRET=your-secret-key-here
LOG_LEVEL=INFO

# Web Dashboard (optional)
DASHBOARD_PORT=8081
BRIDGE_URL=ws://localhost:8765
```

### Docker Compose Profiles

| Profile | Services | Use Case |
|---------|----------|----------|
| `default` | bridge | Production deployment |
| `web` | bridge + web-dashboard | Full stack with UI |
| `postgres` | + postgres | Production with database |
| `redis` | + redis | With caching |
| `monitoring` | + prometheus + grafana | With monitoring |

Example:
```bash
# Full production stack
docker-compose --profile web --profile postgres --profile monitoring up -d
```

## Dashboard Sections

### 1. Dashboard (Home)

**Stats Overview:**
- Active robots count
- Tasks completed
- Average battery level
- Connection latency

**Robot List:** Shows all connected robots with status and battery

**Activity Log:** Real-time system events

### 2. Robots

**Robot Management:**
- Table view of all robots
- Discovery button (auto-find robots)
- Add robot manually
- Control individual robots

**Columns:**
- ID, Name, Type
- Status (online/offline/busy)
- Battery percentage
- Current location

### 3. Control

**Manual Control:**
- D-pad (arrow keys work too!)
  - ↑ Forward
  - ↓ Backward
  - ← Rotate left
  - → Rotate right
  - Space Stop

**Quick Commands:**
- Go to (1, 0)
- Return to dock
- Explore mode

**Natural Language:**
Chat interface for commands like:
- "Move forward 2 meters"
- "Turn left 90 degrees"
- "Navigate to the kitchen"
- "What is your status?"

### 4. Telemetry

**Live Sensor Data:**
- Position (x, y)
- Orientation (degrees)
- Linear velocity (m/s)
- Angular velocity (rad/s)
- Battery (%)
- Obstacle distance (m)

**LiDAR Visualization:**
- Real-time 360° scan
- Obstacle detection
- Robot position

### 5. Fleet

**Multi-Robot Coordination:**
- Fleet map visualization
- Broadcast commands
- Return all to dock
- Start patrol

### 6. Shadow Mode

**AI-Human Agreement Metrics:**
- Agreement rate (%)
- Total decisions
- Pending decisions
- Completed decisions

**Recent Decisions Table:**
- Timestamp
- Robot ID
- AI intent
- Human action
- Agreement status
- Agreement score

**Purpose:** Track how often AI and human operators agree, validate AI safety before full deployment.

### 7. Safety

**Safety Status:**
- Safe mode indicator
- Human-in-the-loop status
- Shadow mode status

**Validation Gates:**
- Gate 1: Unit Tests (2,021 passed)
- Gate 2: Simulation (95.93% success)
- Gate 3: Shadow Mode (pending)
- Gate 4: Gradual Rollout (pending)

**Shadow Statistics:**
- Hours collected vs required (0/200)
- Agreement rate progress

### 8. Logs

**System Logs:**
- Filter by level (debug, info, warn, error)
- Export to file
- Real-time updates

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| ↑ | Move forward |
| ↓ | Move backward |
| ← | Rotate left |
| → | Rotate right |
| Space | Stop |
| Escape | Close modals |

## Troubleshooting

### Connection Issues

**"Cannot connect to bridge"**
```bash
# Check if bridge is running
curl http://localhost:8765/health

# Check WebSocket
ws://localhost:8765  # Should not return HTML
```

**"WebSocket connection refused"**
- Verify bridge port: `netstat -an | grep 8765`
- Check firewall settings
- Try `ws://127.0.0.1:8765` instead of `localhost`

### Dashboard Not Loading

**Port 8081 in use**
```bash
# Find process using port 8081
lsof -i :8081

# Kill it or use different port
python3 -m http.server 8082  # Use port 8082
```

**404 errors for assets**
- Verify `agent_ros_bridge/web/` directory exists
- Check nginx volume mount in docker-compose.yml

### Robot Not Responding

1. Check robot status shows "online"
2. Verify battery > 0%
3. Check logs for error messages
4. Try emergency stop + reconnect

## Development

### File Structure

```
agent_ros_bridge/web/
├── index.html      # Main UI
├── styles.css      # Dark theme
├── app.js          # Application logic
├── server.py       # Python dev server
└── README.md       # This file
```

### Adding New Features

1. Edit `index.html` for UI changes
2. Edit `styles.css` for styling
3. Edit `app.js` for functionality
4. Test with `python3 -m http.server`
5. Submit PR

### Testing

```bash
# Run dashboard locally
cd agent_ros_bridge/web
python3 server.py

# Test WebSocket connection
# Open browser console:
# ws = new WebSocket('ws://localhost:8765')
# ws.send(JSON.stringify({type: 'ping'}))
```

## API Reference

### WebSocket Messages

**Connect:**
```javascript
ws = new WebSocket('ws://localhost:8765?token=<JWT>')
```

**Send Command:**
```json
{
  "type": "command",
  "robot_id": "bot1",
  "command": {
    "action": "move",
    "parameters": {"direction": "forward", "distance": 1.0}
  }
}
```

**Natural Language:**
```json
{
  "type": "natural_language",
  "text": "Move forward 2 meters"
}
```

**Get Robots:**
```json
{"type": "get_robots"}
```

### REST API (Shadow Metrics)

**Get Metrics:**
```bash
curl http://localhost:8765/api/metrics
```

**Get Decisions:**
```bash
curl "http://localhost:8765/api/decisions?limit=10"
```

## Security

- Always use JWT tokens in production
- Enable HTTPS/WSS for remote access
- Restrict dashboard access to authorized users
- Monitor shadow mode for safety violations

## Production Deployment

### With Nginx Reverse Proxy

```nginx
server {
    listen 443 ssl;
    server_name robots.example.com;

    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;

    location / {
        proxy_pass http://localhost:8081;
    }

    location /ws {
        proxy_pass http://localhost:8765;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

### With Kubernetes

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent-ros-bridge
spec:
  replicas: 1
  selector:
    matchLabels:
      app: agent-ros-bridge
  template:
    metadata:
      labels:
        app: agent-ros-bridge
    spec:
      containers:
      - name: bridge
        image: agentrosbridge/agent-ros-bridge:latest
        ports:
        - containerPort: 8765
      - name: web
        image: nginx:alpine
        volumeMounts:
        - name: web-files
          mountPath: /usr/share/nginx/html
      volumes:
      - name: web-files
        configMap:
          name: dashboard-files
```

## Support

- **Issues:** https://github.com/webthree549-bot/agent-ros-bridge/issues
- **Documentation:** https://agent-ros-bridge.readthedocs.io
- **Discord:** https://discord.gg/agent-ros-bridge

## License

MIT License - See [LICENSE](LICENSE) file.