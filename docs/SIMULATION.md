# ROS2 Gazebo Simulation Environment

Full simulation environment for testing Agent ROS Bridge with a virtual TurtleBot3 robot in Gazebo.

## Quick Start

```bash
# Build the simulation image (one-time setup)
./simulation-environment.sh setup

# Start full simulation (Gazebo + Nav2 + Bridge)
./simulation-environment.sh start

# Check status
./simulation-environment.sh status

# Run navigation tests
./simulation-environment.sh test

# Stop everything
./simulation-environment.sh stop
```

## What's Included

| Component | Description | Port | Purpose |
|-----------|-------------|------|---------|
| **Gazebo** | Physics simulator with TurtleBot3 world | 11345 | Simulation |
| **Nav2** | Navigation stack (SLAM, path planning) | - | Autonomy |
| **Agent ROS Bridge** | WebSocket/gRPC bridge for AI agents | 8766 | **AI Control** |
| **Rosbridge** | WebSocket bridge for Foxglove | 9090 | **Visualization** |
| **RViz** | Native ROS visualization (optional) | - | Development |

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SIMULATION STACK                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────┐  │
│  │   Foxglove      │◄──►│   Rosbridge     │◄──►│   ROS2/Gazebo       │  │
│  │   Studio        │WS  │   Server        │ROS2│   + Nav2            │  │
│  │  (Browser)      │:9090│   (Port 9090)   │    │   TurtleBot3        │  │
│  └─────────────────┘    └─────────────────┘    └─────────────────────┘  │
│         ▲                                            ▲                   │
│         │                                            │                   │
│         │         WATCH THE SIMULATION               │ CONTROL THE ROBOT │
│         │                                            │                   │
│  ┌─────────────────┐                         ┌─────────────────────┐    │
│  │  AI Agent       │◄───────────────────────►│   Agent ROS Bridge  │    │
│  │  (OpenClaw/LLM) │      ws://localhost:8766│   (Port 8766)       │    │
│  └─────────────────┘                         └─────────────────────┘    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      SIMULATION STACK                            │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │   Gazebo     │  │    Nav2      │  │  Agent ROS Bridge    │  │
│  │  Simulator   │◄─┤  Navigation  │◄─┤  WebSocket:8766      │  │
│  │  TurtleBot3  │  │  SLAM/Plan   │  │  gRPC:50051          │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│         ▲                                            │          │
│         │         ROS2 Topics/Services/Actions       │          │
│         └────────────────────────────────────────────┘          │
│                                                                  │
│  External: ws://localhost:8766  ─────►  AI Agent/Client         │
└─────────────────────────────────────────────────────────────────┘
```

## Commands

### Setup
```bash
# Build simulation Docker image
./simulation-environment.sh setup
```

### Start/Stop
```bash
# Start full stack
./simulation-environment.sh start

# Start individual components
./simulation-environment.sh start-gazebo
./simulation-environment.sh start-nav2
./simulation-environment.sh start-bridge

# Stop everything
./simulation-environment.sh stop

# Restart
./simulation-environment.sh restart
```

### Development
```bash
# Check status of all containers
./simulation-environment.sh status

# View logs
./simulation-environment.sh logs gazebo
./simulation-environment.sh logs nav2
./simulation-environment.sh logs bridge

# Open shell in container
./simulation-environment.sh shell gazebo
./simulation-environment.sh shell nav2
./simulation-environment.sh shell bridge
```

### Testing
```bash
# Run navigation E2E tests
./simulation-environment.sh test

# Quick test with simulated robot (no Gazebo)
./simulation-environment.sh quick-test
```

## Docker Compose Alternative

### Start Everything (Recommended)
```bash
# Build and start full stack (Gazebo + Nav2 + Agent Bridge + Rosbridge)
docker-compose -f docker-compose.ros2.yml up -d

# Check all services are running
docker-compose -f docker-compose.ros2.yml ps
```

### Foxglove Visualization
```bash
# 1. Start the simulation
docker-compose -f docker-compose.ros2.yml up -d

# 2. Open Foxglove Studio in browser
open https://studio.foxglove.dev

# 3. Click "Open connection" → "Rosbridge (WebSocket)"
# 4. Enter: ws://localhost:9090
# 5. Enjoy 3D visualization!
```

### Start Individual Components
```bash
# Just Gazebo + Nav2 (no bridges)
docker-compose -f docker-compose.ros2.yml up -d gazebo nav2

# Add visualization (Foxglove)
docker-compose -f docker-compose.ros2.yml up -d rosbridge

# Add AI control (Agent Bridge)
docker-compose -f docker-compose.ros2.yml up -d bridge

# Native RViz (requires X11)
docker-compose -f docker-compose.ros2.yml --profile rviz up -d

# Interactive ROS2 CLI
docker-compose -f docker-compose.ros2.yml --profile cli up -d ros2cli
docker-compose -f docker-compose.ros2.yml exec ros2cli bash
```

### View Logs
```bash
docker-compose -f docker-compose.ros2.yml logs -f gazebo
docker-compose -f docker-compose.ros2.yml logs -f nav2
docker-compose -f docker-compose.ros2.yml logs -f bridge
docker-compose -f docker-compose.ros2.yml logs -f rosbridge
```

### Stop Everything
```bash
docker-compose -f docker-compose.ros2.yml down
```

## Visualization with Foxglove

Foxglove Studio provides a web-based 3D visualization of your simulation - no installation required!

### Quick Start

1. **Start the simulation with rosbridge:**
   ```bash
   docker-compose -f docker-compose.ros2.yml up -d
   ```

2. **Open Foxglove Studio:**
   - Go to https://studio.foxglove.dev
   - Or install the desktop app: `brew install --cask foxglove-studio`

3. **Connect to your simulation:**
   - Click "Open connection"
   - Select "Rosbridge (WebSocket)"
   - Enter: `ws://localhost:9090`
   - Click "Open"

4. **Add panels to visualize:**
   - **3D** - See the robot, laser scans, path planning
   - **Image** - Camera feeds (if available)
   - **Plot** - Graph sensor data over time
   - **Topic Graph** - See ROS node connections
   - **Log** - View ROS logs

### For Others to Watch (Same Network)

If others are on the same WiFi/network:

1. **Find your machine's IP:**
   ```bash
   ipconfig getifaddr en0  # macOS
   hostname -I             # Linux
   ```

2. **Share the URL:**
   ```
   ws://YOUR_IP:9090
   ```

3. **They open Foxglove Studio** and connect to your IP

### For Remote Access (Internet)

Use ngrok to expose rosbridge publicly:

```bash
# Install ngrok
brew install ngrok

# Start tunnel
ngrok tcp 9090

# Share the URL (e.g., tcp://abc123.ngrok.io:12345)
# Others connect to: wss://abc123.ngrok.io (WebSocket secure)
```

## Testing with the Bridge

Once running, connect to the Agent ROS Bridge for AI control:

```python
import asyncio
import websockets
import json

async def test_robot():
    async with websockets.connect('ws://localhost:8766') as ws:
        # Authenticate
        await ws.send(json.dumps({
            'type': 'auth',
            'token': 'your-jwt-token'
        }))
        
        # Discover robots
        await ws.send(json.dumps({
            'type': 'command',
            'action': 'discover'
        }))
        
        response = await ws.recv()
        print(f"Discovered: {response}")
        
        # Send navigation goal
        await ws.send(json.dumps({
            'type': 'command',
            'action': 'robot.execute',
            'robot_id': 'turtlebot3',
            'command': {
                'action': 'navigate_to_pose',
                'parameters': {
                    'x': 1.0,
                    'y': 1.0,
                    'theta': 0.0
                }
            }
        }))

asyncio.run(test_robot())
```

## Troubleshooting

### macOS - No GUI
macOS doesn't support native X11 forwarding. Options:
1. Use XQuartz: `brew install xquartz`, then `xhost +localhost`
2. Use VNC viewer to connect to container
3. Run headless (tests work without GUI)

### Container won't start
```bash
# Check Docker is running
docker info

# Clean up old containers
./simulation-environment.sh stop
docker system prune -f

# Rebuild image
./simulation-environment.sh setup
```

### Nav2 not ready
Nav2 takes ~30-60 seconds to fully initialize. Check status:
```bash
./simulation-environment.sh status
```

### Tests failing
Ensure simulation is fully started:
```bash
./simulation-environment.sh start
sleep 30  # Wait for initialization
./simulation-environment.sh test
```

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `JWT_SECRET` | Auto-generated | Secret for JWT tokens |
| `ROS_DOMAIN_ID` | 0 | ROS2 domain ID |
| `TURTLEBOT3_MODEL` | burger | Robot model (burger/waffle/waffle_pi) |
| `DISPLAY` | host.docker.internal:0 | X11 display for GUI |

## Files

- `simulation-environment.sh` - Main management script
- `docker-compose.ros2.yml` - Docker Compose configuration
- `docker/Dockerfile.simulation` - Simulation image definition
- `docker/ros_entrypoint.sh` - Container entrypoint
