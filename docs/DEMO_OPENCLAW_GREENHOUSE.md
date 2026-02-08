# OpenClaw + Greenhouse Demo

Complete demonstration of OpenClaw AI controlling a greenhouse via ROS2.

## What It Does

```
┌─────────────────────────────────────────────────────────────┐
│                      DEMO FLOW                               │
├─────────────────────────────────────────────────────────────┤
│  1. Start Docker container with ROS2 Jazzy                   │
│  2. Start TCP server (port 9999 exposed)                     │
│  3. OpenClaw agent connects from macOS                       │
│  4. Read sensors: Temperature, Humidity                      │
│  5. AI Decision:                                             │
│     • Temp > 28°C → Turn ON fan                             │
│     • Humidity < 40% → Open valve                           │
│  6. Send commands back to greenhouse                         │
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

### Option 1: Automatic Demo (Recommended)

```bash
# Run everything automatically
./scripts/demo_openclaw_greenhouse.sh
```

This will:
- ✅ Check Docker is running
- ✅ Start ROS2 container
- ✅ Build the project
- ✅ Start TCP server
- ✅ Run OpenClaw AI agent
- ✅ Show live sensor readings and AI decisions

### Option 2: Manual Steps

```bash
# Step 1: Start Docker container
./scripts/docker_start.sh --jazzy

# Step 2: Inside container, build project
cd /app
./scripts/build.sh

# Step 3: Start TCP server
./scripts/start_openclaw_server.sh
# Server listening on port 9999

# Step 4: On macOS, run OpenClaw agent
python3 /tmp/openclaw_greenhouse_agent.py
```

## Expected Output

```
================================================
  OpenClaw + Greenhouse Demo
================================================

▶ STEP 1: Checking Docker...
✓ Docker is running

▶ STEP 2: Starting ROS Container...
✓ Container ready

▶ STEP 3: Building project...
✓ Build complete

▶ STEP 4: Starting TCP Server...
✓ TCP server started

▶ STEP 5: Testing connection...
✓ TCP server accessible

▶ STEP 6: Running Demo...

==================================================
🤖 OpenClaw AI Agent - Greenhouse Control
==================================================

System Status: ROS=jazzy, Mock=true

--- Iteration 1 ---
🌡️  Temperature: 25.0°C
💧 Humidity: 50%
  → 🌀 Fan: OFF (comfortable)
  → 💦 Valve: CLOSED (humid enough)

--- Iteration 2 ---
🌡️  Temperature: 32.0°C
💧 Humidity: 35%
  → 🌀 Fan: ON (too hot)
  → 💦 Valve: OPEN (too dry)

...

✓ Demo complete!
```

## Demo Scenarios

The AI responds to different conditions:

| Scenario | Temp | Humidity | Fan | Valve |
|----------|------|----------|-----|-------|
| Hot Day | 32°C | 35% | ON | OPEN |
| Cool Morning | 22°C | 60% | OFF | CLOSED |
| Dry Afternoon | 29°C | 25% | ON | OPEN |
| Ideal | 24°C | 55% | OFF | CLOSED |

## How It Works

### 1. Docker Container (ROS2)

- Runs ROS2 Jazzy
- Exposes port 9999 for TCP connection
- Contains greenhouse simulation (mock mode)

### 2. TCP Server

- Listens on 0.0.0.0:9999
- Accepts JSON commands from OpenClaw
- Translates to ROS2 topics

### 3. OpenClaw Agent (macOS)

- Connects via TCP socket to localhost:9999
- Sends sensor read requests
- Receives data and makes AI decisions
- Sends actuator commands back

### Communication Protocol

```python
# OpenClaw → ROS Bridge (Request)
{"action": "read_sensor", "sensor": "env"}

# ROS Bridge → OpenClaw (Response)
{"status": "ok", "data": {"temperature": 25.0, "humidity": 50.0}}

# OpenClaw → ROS Bridge (Command)
{"action": "write_actuator", "actuator": "fan", "value": true}

# ROS Bridge → OpenClaw (Response)
{"status": "ok", "message": "fan set to True"}
```

## Troubleshooting

### Connection Refused

```bash
# Check if server is running
docker exec ros2-jazzy-bridge pgrep -f openclaw_tcp_server

# Check port mapping
docker port ros2-jazzy-bridge

# Restart server
docker exec ros2-jazzy-bridge pkill -f openclaw_tcp_server
./scripts/start_openclaw_server.sh
```

### Demo Won't Start

```bash
# Stop and clean up
./scripts/docker_start.sh --stop
./scripts/docker_start.sh --rm

# Run fresh
./scripts/demo_openclaw_greenhouse.sh
```

### Test TCP Connection

```bash
# From macOS, test connection
nc -vz localhost 9999

# Send manual command
echo '{"action": "ping"}' | nc localhost 9999
```

## Files Involved

| File | Purpose |
|------|---------|
| `demo_openclaw_greenhouse.sh` | Main demo script |
| `openclaw_tcp_server.py` | TCP server in Docker |
| `docker_start.sh` | Start Docker container with ports |
| `greenhouse_plugin.py` | Greenhouse simulation |

## Next Steps

After the demo works:

1. **Try Arm Manipulation Demo** - Similar setup for robot arm
2. **Create Custom Plugin** - Build your own robot controller
3. **Connect Real Hardware** - Set `MOCK_MODE=false` and use actual sensors

## Resources

- `docs/MACOS_SETUP_GUIDE.md` - Complete setup instructions
- `docs/DOCKER_PORT_SETUP.md` - Port configuration details
- `docs/ARCHITECTURE.md` - System architecture
