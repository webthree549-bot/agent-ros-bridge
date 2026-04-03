# Agent ROS Bridge - Web Dashboard

A modern, real-time web interface for controlling and monitoring ROS robots through Agent ROS Bridge.

![Dashboard Preview](https://via.placeholder.com/800x400/1e293b/3b82f6?text=Agent+ROS+Bridge+Dashboard)

## Features

### 🎮 Robot Control
- **D-Pad Interface** - Arrow key compatible manual control
- **Natural Language** - Type commands like "Move forward 2 meters"
- **Quick Commands** - One-click common actions
- **Emergency Stop** - Instant halt for all robots

### 📊 Real-time Monitoring
- **Live Telemetry** - Position, velocity, battery, sensors
- **LiDAR Visualization** - Real-time point cloud display
- **Fleet Overview** - Multi-robot status at a glance
- **Connection Status** - WebSocket health monitoring

### 🛡️ Safety First
- **Validation Gates** - Track deployment readiness
- **Shadow Mode Stats** - Monitor AI-human agreement
- **Human-in-the-Loop** - Approval workflow visibility
- **Safety Indicators** - Clear safe/unsafe status

### 📡 Multi-Robot Support
- **Robot Discovery** - Auto-find robots on network
- **Fleet Commands** - Broadcast to multiple robots
- **Individual Control** - Per-robot detailed control
- **Status Tracking** - Battery, location, task state

## Quick Start

### Option 1: Direct File Open
Simply open `index.html` in your browser:
```bash
open index.html
```

### Option 2: Local Server (Recommended)
```bash
cd web
python3 -m http.server 8080
# Open http://localhost:8080
```

### Option 3: Node.js
```bash
cd web
npx serve .
```

## Connecting to Bridge

1. Start the Agent ROS Bridge:
```bash
export JWT_SECRET=$(openssl rand -base64 32)
agent-ros-bridge --websocket-port 8765
```

2. Open the web dashboard
3. Click **Connect** button
4. Enter WebSocket URL (default: `ws://localhost:8765`)
5. Add JWT token if authentication is enabled

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| ↑ | Move forward |
| ↓ | Move backward |
| ← | Rotate left |
| → | Rotate right |
| Space | Stop |
| Escape | Close modals |

## Natural Language Commands

The dashboard supports natural language commands:

```
"Move forward 2 meters"
"Turn left 90 degrees"
"Navigate to the kitchen"
"Return to base"
"What is your status?"
"Check surroundings"
"Stop"
```

## Architecture

```
┌─────────────┐     WebSocket      ┌─────────────────┐     ROS2     ┌─────────┐
│   Browser   │ ◄────────────────► │  Agent ROS      │ ◄───────────►│  Robot  │
│  Dashboard  │     JSON/WS        │  Bridge         │   Topics     │         │
└─────────────┘                    └─────────────────┘              └─────────┘
```

## WebSocket Protocol

### Commands
```json
{
  "type": "command",
  "robot_id": "turtlebot_01",
  "command": {
    "action": "move",
    "parameters": {"distance": 1.0, "speed": 0.5}
  }
}
```

### Natural Language
```json
{
  "type": "natural_language",
  "robot_id": "turtlebot_01",
  "text": "Move forward 2 meters"
}
```

### Telemetry Request
```json
{
  "type": "get_telemetry",
  "robot_id": "turtlebot_01"
}
```

## Development

### File Structure
```
web/
├── index.html      # Main HTML structure
├── styles.css      # Dark theme styling
├── app.js          # Application logic
└── README.md       # This file
```

### Customization

#### Colors
Edit CSS variables in `styles.css`:
```css
:root {
  --primary: #3b82f6;    /* Blue accent */
  --success: #10b981;    /* Green for safe */
  --danger: #ef4444;     /* Red for emergency */
  --bg-dark: #0f172a;    /* Dark background */
}
```

#### Adding New Commands
Edit `app.js` quick commands:
```javascript
quickCommand(text) {
    // Add your command handling
}
```

## Browser Support

- Chrome/Edge 90+
- Firefox 88+
- Safari 14+
- Mobile browsers (responsive design)

## Security

- JWT token authentication support
- WSS (WebSocket Secure) compatible
- No data stored locally
- All communication through WebSocket

## Troubleshooting

### Connection Refused
- Verify bridge is running: `agent-ros-bridge --websocket-port 8765`
- Check firewall settings
- Try `ws://127.0.0.1:8765` instead of `localhost`

### Authentication Failed
- Verify JWT token is correct
- Check `JWT_SECRET` environment variable matches

### Commands Not Working
- Ensure robot is selected from dropdown
- Check robot status is "online"
- Look for errors in browser console

## License

MIT - Same as Agent ROS Bridge