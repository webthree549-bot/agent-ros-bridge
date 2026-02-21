# How to Play with the Actions Demo

Your actions demo is running! Here's how to interact with it.

---

## Current Status

✅ Bridge is running on port 8773
❌ WebSocket transport failed (missing library - FIXED in Dockerfile)
❌ gRPC transport failed (missing library - FIXED in Dockerfile)

---

## Step 1: Rebuild with Dependencies

```bash
# Stop current demo
docker-compose down

# Rebuild with fixed dependencies
docker-compose up --build
```

---

## Step 2: Interact with the Bridge

### Option A: WebSocket (wscat)

```bash
# Install wscat
npm install -g wscat

# Connect to bridge
wscat -c "ws://localhost:8773"

# Send commands
{"command": {"action": "list_robots"}}
{"command": {"action": "ping"}}
```

### Option B: Python Client

```python
import asyncio
import websockets
import json

async def test():
    uri = "ws://localhost:8773"
    async with websockets.connect(uri) as ws:
        # Send command
        await ws.send(json.dumps({
            "command": {"action": "list_robots"}
        }))
        
        # Get response
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(test())
```

### Option C: cURL (HTTP bridge)

```bash
# If HTTP endpoint is enabled
curl -X POST http://localhost:8773/api/command \
  -H "Content-Type: application/json" \
  -d '{"action": "list_robots"}'
```

### Option D: Browser

Open `http://localhost:8773` in browser (if web dashboard is enabled)

---

## Available Commands

### System Commands

```json
{"command": {"action": "ping"}}
{"command": {"action": "list_robots"}}
{"command": {"action": "get_topics"}}
```

### Action Commands

```json
{"command": {"action": "actions.navigate", "parameters": {"x": 5.0, "y": 3.0}}}
{"command": {"action": "actions.cancel"}}
{"command": {"action": "actions.status"}}
```

---

## Quick Test Script

```bash
#!/bin/bash
# test_bridge.sh

echo "Testing Agent ROS Bridge..."

# Test 1: Check if port is open
nc -zv localhost 8773 && echo "✅ Port 8773 open" || echo "❌ Port closed"

# Test 2: WebSocket connection (if wscat installed)
if command -v wscat &> /dev/null; then
    echo '{"command": {"action": "ping"}}' | wscat -c "ws://localhost:8773" &
    sleep 1
    kill %1 2>/dev/null
fi

echo "Done!"
```

---

## Expected Output

When working correctly:

```
✅ WebSocket server started on ws://0.0.0.0:8773
✅ gRPC server started on grpc://0.0.0.0:50051
✅ ROS2 connector registered

Ready for connections!
```

---

## Troubleshooting

### Port already in use
```bash
lsof -i :8773
kill <PID>
```

### Connection refused
```bash
# Check if container is running
docker ps | grep actions-demo

# Check logs
docker-compose logs -f
```

### Missing JWT_SECRET
```bash
export JWT_SECRET=$(openssl rand -base64 32)
docker-compose up
```

---

## Next Steps

1. ✅ Rebuild with `docker-compose up --build`
2. ✅ Test with `wscat -c "ws://localhost:8773"`
3. ✅ Send commands
4. ✅ Watch responses

**Enjoy your robot bridge! 🤖**
