# API Reference

Complete API documentation for Agent ROS Bridge.

**Version**: 0.6.5  
**Base URL**: `ws://localhost:8765` (WebSocket)  
**REST API**: `http://localhost:8765/api`

---

## WebSocket API

### Connection

```javascript
const ws = new WebSocket('ws://localhost:8765?token=JWT_TOKEN');

ws.onopen = () => console.log('Connected');
ws.onmessage = (event) => console.log(JSON.parse(event.data));
ws.onerror = (error) => console.error('Error:', error);
ws.onclose = () => console.log('Disconnected');
```

### Authentication

Include JWT token in connection URL:
```
ws://localhost:8765?token=eyJhbGciOiJSUzI1NiIs...
```

Or send authentication message:
```json
{
  "type": "auth",
  "token": "eyJhbGciOiJSUzI1NiIs..."
}
```

### Message Format

All messages are JSON with a `type` field:

```json
{
  "type": "command",
  "...": "..."
}
```

---

## Commands

### List Robots

Request:
```json
{
  "type": "get_robots"
}
```

Response:
```json
{
  "type": "robot_list",
  "robots": [
    {
      "id": "bot_001",
      "name": "Warehouse Bot 1",
      "type": "TurtleBot3",
      "status": "online",
      "battery": 85,
      "location": "Zone A"
    }
  ]
}
```

### Execute Command

Request:
```json
{
  "type": "command",
  "robot_id": "bot_001",
  "command": {
    "action": "move",
    "parameters": {
      "direction": "forward",
      "distance": 2.0,
      "speed": 0.5
    }
  }
}
```

Response:
```json
{
  "type": "command_result",
  "success": true,
  "robot_id": "bot_001",
  "action": "move",
  "duration": 4.2,
  "timestamp": "2024-04-02T10:30:00Z"
}
```

### Natural Language Command

Request:
```json
{
  "type": "natural_language",
  "robot_id": "bot_001",
  "text": "Move forward 2 meters slowly"
}
```

Response:
```json
{
  "type": "command_result",
  "success": true,
  "response": "Moving forward 2 meters at 0.3 m/s",
  "parsed_intent": "move",
  "confidence": 0.94,
  "action": "natural_language"
}
```

### Get Telemetry

Request:
```json
{
  "type": "get_telemetry",
  "robot_id": "bot_001"
}
```

Response:
```json
{
  "type": "telemetry",
  "robot_id": "bot_001",
  "data": {
    "position": {
      "x": 1.5,
      "y": 2.3,
      "z": 0.0
    },
    "orientation": 0.785,
    "linear_velocity": 0.5,
    "angular_velocity": 0.1,
    "battery": 85,
    "obstacle_distance": 1.2,
    "timestamp": "2024-04-02T10:30:00Z"
  }
}
```

### Emergency Stop

Request:
```json
{
  "type": "emergency_stop"
}
```

Response:
```json
{
  "type": "emergency_stop_result",
  "success": true,
  "affected_robots": ["bot_001", "bot_002"],
  "timestamp": "2024-04-02T10:30:00Z"
}
```

### Subscribe to Telemetry

Request:
```json
{
  "type": "subscribe_telemetry",
  "robot_id": "bot_001",
  "interval": 1.0
}
```

Continuous responses (every 1 second):
```json
{
  "type": "telemetry",
  "robot_id": "bot_001",
  "data": { ... }
}
```

Unsubscribe:
```json
{
  "type": "unsubscribe_telemetry",
  "robot_id": "bot_001"
}
```

---

## Actions Reference

### Movement Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `move` | `direction`, `distance`, `speed` | Linear movement |
| `rotate` | `angle`, `speed` | Rotation in place |
| `navigate` | `target`, `tolerance` | Navigate to location |
| `stop` | None | Emergency stop |

### Manipulation Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `pick` | `object`, `height` | Pick up object |
| `place` | `location`, `height` | Place object |
| `grasp` | `force` | Grasp with force |
| `release` | None | Release gripper |

### System Actions

| Action | Parameters | Description |
|--------|-----------|-------------|
| `status` | None | Get robot status |
| `home` | None | Return to home |
| `dock` | None | Dock at charging station |
| `explore` | `duration`, `area` | Autonomous exploration |

---

## REST API

### Shadow Mode Metrics

#### GET /api/metrics

Returns current shadow mode statistics.

Response:
```json
{
  "agreement_rate": 0.96,
  "total_decisions": 144000,
  "pending_count": 0,
  "completed_decisions": 144000,
  "hours_collected": 200.0,
  "required_hours": 200.0
}
```

#### GET /api/decisions

Query parameters:
- `limit`: Number of decisions (default: 10, max: 1000)
- `offset`: Pagination offset
- `robot_id`: Filter by robot
- `since`: ISO timestamp filter

Response:
```json
{
  "decisions": [
    {
      "timestamp": "2024-04-02T10:30:00Z",
      "robot_id": "bot_001",
      "ai_proposal": {
        "intent_type": "move_forward",
        "confidence": 0.94,
        "parameters": {"distance": 2.0}
      },
      "human_action": {
        "command": "move_forward",
        "parameters": {"distance": 2.0}
      },
      "agreement": true,
      "agreement_score": 0.95
    }
  ],
  "total": 144000
}
```

### Robot Management

#### GET /api/robots

List all connected robots.

Response:
```json
{
  "robots": [
    {
      "id": "bot_001",
      "name": "Warehouse Bot 1",
      "type": "TurtleBot3",
      "status": "online",
      "battery": 85,
      "location": "Zone A",
      "last_seen": "2024-04-02T10:30:00Z"
    }
  ]
}
```

#### GET /api/robots/{id}

Get specific robot details.

Response:
```json
{
  "id": "bot_001",
  "name": "Warehouse Bot 1",
  "type": "TurtleBot3",
  "status": "online",
  "battery": 85,
  "location": "Zone A",
  "capabilities": ["move", "navigate", "sensors"],
  "safety_status": "supervised"
}
```

### Fleet Operations

#### POST /api/fleet/broadcast

Send command to multiple robots.

Request:
```json
{
  "command": {
    "action": "return_home"
  },
  "selector": {
    "battery_below": 20
  }
}
```

Response:
```json
{
  "success": true,
  "affected_robots": ["bot_001", "bot_003"],
  "results": [
    {"robot_id": "bot_001", "success": true},
    {"robot_id": "bot_003", "success": true}
  ]
}
```

### Health Check

#### GET /health

Returns service health status.

Response:
```json
{
  "status": "healthy",
  "version": "0.6.5",
  "timestamp": "2024-04-02T10:30:00Z",
  "robots_connected": 5,
  "uptime_seconds": 86400
}
```

---

## Error Handling

### Error Response Format

```json
{
  "type": "error",
  "error": {
    "code": "ROBOT_NOT_FOUND",
    "message": "Robot 'bot_999' not found",
    "details": {
      "robot_id": "bot_999"
    }
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `AUTH_INVALID` | 401 | Invalid authentication token |
| `AUTH_EXPIRED` | 401 | Token expired |
| `ROBOT_NOT_FOUND` | 404 | Robot ID not found |
| `ROBOT_OFFLINE` | 503 | Robot is offline |
| `COMMAND_INVALID` | 400 | Invalid command format |
| `SAFETY_VIOLATION` | 403 | Safety check failed |
| `RATE_LIMITED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |

---

## Code Examples

### Python

```python
import asyncio
import websockets
import json

async def control_robot():
    uri = "ws://localhost:8765?token=YOUR_TOKEN"
    
    async with websockets.connect(uri) as ws:
        # Send command
        await ws.send(json.dumps({
            "type": "command",
            "robot_id": "bot_001",
            "command": {
                "action": "move",
                "parameters": {"direction": "forward", "distance": 2.0}
            }
        }))
        
        # Get response
        response = await ws.recv()
        data = json.loads(response)
        print(f"Result: {data}")

asyncio.run(control_robot())
```

### JavaScript

```javascript
const ws = new WebSocket('ws://localhost:8765?token=YOUR_TOKEN');

ws.onopen = () => {
  // Send natural language command
  ws.send(JSON.stringify({
    type: 'natural_language',
    robot_id: 'bot_001',
    text: 'Navigate to the kitchen'
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Response:', data);
};
```

### cURL (REST API)

```bash
# Get metrics
curl http://localhost:8765/api/metrics

# List robots
curl http://localhost:8765/api/robots

# Get decisions
curl "http://localhost:8765/api/decisions?limit=10"
```

---

## SDKs

### Official SDKs

- **Python**: `pip install agent-ros-bridge`
- **JavaScript**: `npm install @agent-ros-bridge/sdk`

### Community SDKs

- **Go**: github.com/example/agent-ros-bridge-go
- **Rust**: github.com/example/agent-ros-bridge-rs

---

## Rate Limits

| Endpoint | Limit | Window |
|----------|-------|--------|
| WebSocket messages | 1000 | per minute |
| REST API | 100 | per minute |
| Telemetry subscriptions | 10 | per robot |

---

## Changelog

### v0.6.5 (Current)
- Added fleet broadcast API
- Added shadow mode metrics endpoint
- Improved telemetry streaming

### v0.6.0
- Added natural language processing
- Added gRPC support
- Breaking: Changed auth format

See [CHANGELOG.md](../CHANGELOG.md) for full history.