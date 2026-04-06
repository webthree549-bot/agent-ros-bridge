# OpenClaw Bridge Demo Guide

Complete guide to demonstrating real-time bidirectional robot control.

---

## 🎯 Demo Overview

**What You'll Show:**
1. ✅ Real-time telemetry streaming (2Hz)
2. ✅ Command forwarding with progress updates
3. ✅ Multi-client synchronization (OpenClaw + Web UI)
4. ✅ Emergency stop broadcasting
5. ✅ Web dashboard integration

**Architecture:**
```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐      ┌───────┐
│  OpenClaw   │◄────►│ OpenClaw     │◄────►│ Agent ROS   │◄────►│ Robot │
│  (You)      │      │ Bridge:8766  │      │ Bridge:8765 │      │       │
└─────────────┘      └──────────────┘      └─────────────┘      └───────┘
                            ▲
                            │
                     ┌──────┴──────┐
                     │   Web UI    │
                     │  (Browser)  │
                     └─────────────┘
```

---

## 📋 Prerequisites

### 1. Agent ROS Bridge (if you have ROS)
```bash
# Terminal 1 - Start Agent ROS Bridge
agent-ros-bridge --websocket-port 8765
```

If you don't have ROS installed, the bridge will run in **simulation mode**.

### 2. OpenClaw Bridge
```bash
# Terminal 2 - Start OpenClaw Bridge
./start_openclaw_bridge.sh

# Or manually:
python3 skills/agent-ros-bridge/scripts/openclaw_bridge.py --port 8766
```

**Verify it's running:**
```bash
lsof -i :8766  # Should show Python listening
```

---

## 🎬 Demo Script

### Phase 1: Setup (30 seconds)

**Terminal 1 - OpenClaw Bridge:**
```bash
$ ./start_openclaw_bridge.sh

2026-04-06 14:27:51,841 - openclaw_bridge - INFO - Starting OpenClaw Bridge on port 8766
2026-04-06 14:27:51,891 - websockets.server - INFO - server listening on 127.0.0.1:8766
2026-04-06 14:27:51,891 - openclaw_bridge - INFO - Bridge listening on ws://localhost:8766
```

**Key points to mention:**
- Bridge is listening on WebSocket port 8766
- Ready for OpenClaw and Web UI connections
- Telemetry will stream at 2Hz (every 500ms)

---

### Phase 2: Run Demo Script (1 minute)

**Terminal 2 - Demo Client:**
```bash
$ python3 demo_openclaw_bridge.py

============================================================
🔌 OPENCLAW BRIDGE DEMO
============================================================

✅ OpenClaw Bridge detected on port 8766

============================================================
🚀 OPENCLAW BRIDGE DEMO
============================================================

This demo shows:
  ✅ Real-time bidirectional communication
  ✅ OpenClaw + Web UI synchronization
  ✅ Command forwarding and telemetry streaming
  ✅ Multi-client support

============================================================
🎭 DEMO: OpenClaw Agent (openclaw)
============================================================
Connecting to ws://localhost:8766...
✅ Connected as openclaw

📤 Sending commands to robot...

1️⃣  Command: Navigate to kitchen
   ⬅️  Response: command_result
   ✅ Command executed successfully!

2️⃣  Subscribing to telemetry stream...

📡 Listening for telemetry (5 seconds)...

   📊 Telemetry #1: Battery=85%, Pos=(1.50, 2.00)
   📊 Telemetry #2: Battery=84%, Pos=(1.65, 2.10)
   📊 Telemetry #3: Battery=84%, Pos=(1.80, 2.20)
   📊 Telemetry #4: Battery=83%, Pos=(1.95, 2.30)
   📊 Telemetry #5: Battery=83%, Pos=(2.10, 2.40)

📈 Total messages received: 5

============================================================
🎭 DEMO: Web Dashboard (web_ui)
============================================================
Connecting to ws://localhost:8766...
✅ Connected as web_ui

🖥️  Web UI initialized
   Waiting for state broadcast...

   📊 Connected clients: {'openclaw': 1, 'web_ui': 1}
   🤖 Robot telemetry: {'battery': 83, 'position': {...}}

📤 Sending robot command from Web UI...
   ⬅️  Response type: execution_result

============================================================
✅ DEMO COMPLETE
============================================================
```

**Key points to mention:**
- Both OpenClaw and Web UI connected simultaneously
- Telemetry streaming every 500ms
- Position updates show robot movement
- Web UI sees the same state as OpenClaw

---

### Phase 3: Live Commands (2 minutes)

**Interactive demo - Natural language commands:**

```python
# In Python REPL or script
from skills.agent_ros_bridge.scripts.openclaw_bridge import OpenClawSkillInterface

async def demo():
    interface = OpenClawSkillInterface(port=8766)
    await interface.connect()
    
    # Command 1: Navigation
    print("🤖 User: 'Navigate to the kitchen'")
    result = await interface.send_command("navigate to kitchen")
    print(f"✅ Robot: Arrived at kitchen")
    print(f"   Execution time: {result.get('execution_time', 'N/A')}s")
    
    # Command 2: Status query
    print("\n🤖 User: 'What's your status?'")
    status = await interface.query_status()
    print(f"✅ Robot: Battery {status.get('battery', 'N/A')}%, Position {status.get('position', {})}")
    
    # Command 3: Movement
    print("\n🤖 User: 'Move forward 2 meters'")
    result = await interface.send_command("move forward", distance=2.0)
    print(f"✅ Robot: Moved 2 meters forward")
    
    await interface.disconnect()

import asyncio
asyncio.run(demo())
```

**Expected output:**
```
🤖 User: 'Navigate to the kitchen'
✅ Robot: Arrived at kitchen
   Execution time: 12.5s

🤖 User: 'What's your status?'
✅ Robot: Battery 78%, Position {'x': 3.2, 'y': 4.1, 'theta': 0.5}

🤖 User: 'Move forward 2 meters'
✅ Robot: Moved 2 meters forward
```

---

### Phase 4: Web UI Demo (1 minute)

**Show the Web Dashboard:**

1. Open browser to `http://localhost:8000` (if Agent ROS Bridge web UI is running)
2. Or connect directly via WebSocket:

```javascript
// Browser console
const ws = new WebSocket('ws://localhost:8766', ['openclaw-bridge']);

ws.onopen = () => {
  ws.send(JSON.stringify({ client_type: 'web_ui' }));
  console.log('✅ Connected to bridge');
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('📨 Received:', data.type, data);
};
```

**What to show:**
- Live robot position on map
- Battery level indicator
- Telemetry graphs
- Command buttons that sync with OpenClaw

---

### Phase 5: Emergency Stop (30 seconds)

**Demo safety feature:**

```python
# Trigger emergency stop
await interface.send_command("emergency stop")
```

**What happens:**
1. Emergency stop command sent
2. Broadcast to ALL connected clients
3. Web UI shows red alert
4. OpenClaw receives confirmation
5. Robot stops immediately

**Web UI shows:**
```
🚨 EMERGENCY STOP TRIGGERED
Source: openclaw
Timestamp: 2026-04-06T14:30:00Z
```

---

## 📊 Demo Checklist

Before demo:
- [ ] OpenClaw Bridge running on port 8766
- [ ] Agent ROS Bridge running (or simulation mode)
- [ ] Demo script tested: `python3 demo_openclaw_bridge.py`
- [ ] Web UI accessible (if applicable)

During demo:
- [ ] Show terminal with bridge logs
- [ ] Run demo script - show real-time telemetry
- [ ] Execute live commands
- [ ] Show Web UI synchronization
- [ ] Demonstrate emergency stop
- [ ] Explain architecture diagram

After demo:
- [ ] Q&A about integration
- [ ] Discuss use cases
- [ ] Show configuration options

---

## 🎥 One-Command Demo

For a quick demo, run:

```bash
# Start everything
./start_openclaw_bridge.sh &
sleep 2
python3 demo_openclaw_bridge.py
```

---

## 📈 Expected Metrics

| Metric | Target | What It Shows |
|--------|--------|---------------|
| Connection time | < 100ms | Fast handshake |
| Telemetry latency | < 500ms | 2Hz streaming |
| Command response | < 2s | Efficient forwarding |
| Concurrent clients | 10+ | Scalability |

---

## 🔧 Troubleshooting

**"Connection refused"**
```bash
# Check if bridge is running
lsof -i :8766

# Restart if needed
pkill -f openclaw_bridge
./start_openclaw_bridge.sh
```

**"No telemetry received"**
- Agent ROS Bridge may not be running (simulation mode works fine)
- Check bridge logs for errors
- Verify WebSocket connection

**"Commands timeout"**
- Agent ROS Bridge may not be connected to actual robot
- Demo will still show command flow working

---

## 🎓 Key Takeaways

1. **Real-time**: 2Hz telemetry streaming, instant command execution
2. **Bidirectional**: OpenClaw ↔ Robot ↔ Web UI
3. **Multi-client**: Multiple UIs stay synchronized
4. **Safety**: Emergency stop broadcasts to all clients
5. **Flexible**: Works with or without actual ROS/robots

---

## 🚀 Next Steps

After the demo:
1. Integrate with actual robot hardware
2. Deploy to production environment
3. Add custom tool definitions
4. Scale to multi-robot fleets

---

*Demo version: 0.7.0*  
*Last updated: 2026-04-06*
