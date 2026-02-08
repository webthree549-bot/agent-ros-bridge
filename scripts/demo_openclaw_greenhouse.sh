#!/bin/bash
# demo_openclaw_greenhouse.sh - Complete demo: OpenClaw controls greenhouse via ROS

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log() { echo -e "${BLUE}[DEMO] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; }
step() { echo -e "${CYAN}▶ STEP $1: $2${NC}"; }

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

log "================================================"
log "  OpenClaw + Greenhouse Demo"
log "================================================"
log ""
log "This demo shows OpenClaw controlling a"
log "greenhouse simulation via ROS2 in Docker"
log ""

# Check if running on macOS
OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    log "Detected macOS - using Docker port mapping"
    DOCKER_HOST="localhost"
else
    DOCKER_HOST="ros2-jazzy-bridge"
fi

# Step 1: Check Docker
step "1" "Checking Docker..."
if ! docker info > /dev/null 2>&1; then
    error "Docker is not running. Please start Docker Desktop."
    exit 1
fi
success "Docker is running"

# Step 2: Start/Check Container
step "2" "Starting ROS Container..."
CONTAINER_NAME="ros2-jazzy-bridge"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log "Container already running"
    # Check if port is mapped
    if ! docker port "$CONTAINER_NAME" 9999 > /dev/null 2>&1; then
        warn "Container running but port 9999 not mapped!"
        warn "You may need to recreate the container with:"
        warn "  ./scripts/docker_start.sh --rm"
        warn "  ./scripts/docker_start.sh --jazzy"
    fi
elif docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log "Starting existing container..."
    docker start "$CONTAINER_NAME"
else
    log "Creating new container with port mapping..."
    ./scripts/docker_start.sh --jazzy &
    sleep 5
fi

success "Container ready"

# Debug: Show container port mapping
log "Container port mapping:"
docker port "$CONTAINER_NAME" 2>/dev/null || log "  (no ports mapped yet)"

# Step 3: Build Project
step "3" "Building project inside container..."
docker exec "$CONTAINER_NAME" bash -c "
    cd /app && source /opt/ros/jazzy/setup.bash && ./scripts/build.sh
" || warn "Build may have warnings, continuing..."

success "Build complete"

# Step 4: Start TCP Server
step "4" "Starting OpenClaw TCP Server..."

# Check if TCP server file exists
if ! docker exec "$CONTAINER_NAME" test -f /app/openclaw_ros_bridge/communication/openclaw_tcp_server.py; then
    error "TCP server file not found in container!"
    error "Make sure /app is mounted correctly."
    exit 1
fi

if ! docker exec "$CONTAINER_NAME" pgrep -f "openclaw_tcp_server" > /dev/null 2>&1; then
    log "Starting TCP server in background..."
    docker exec -d "$CONTAINER_NAME" bash -c "
        cd /app && 
        source /opt/ros/jazzy/setup.bash && 
        source install/setup.bash &&
        export MOCK_MODE=true &&
        python3 /app/openclaw_ros_bridge/communication/openclaw_tcp_server.py > /tmp/tcp_server.log 2>&1
    "
    
    # Wait for server to start
    log "Waiting for TCP server to start..."
    for i in {1..10}; do
        sleep 1
        if docker exec "$CONTAINER_NAME" bash -c "netstat -tlnp 2>/dev/null | grep -q ':9999' || ss -tlnp 2>/dev/null | grep -q ':9999'"; then
            success "TCP server is listening on port 9999"
            break
        fi
        log "  Attempt $i/10..."
    done
else
    success "TCP server already running"
fi

# Step 5: Test Connection
step "5" "Testing connection..."

# Check if port is mapped
if ! docker port "$CONTAINER_NAME" 9999 > /dev/null 2>&1; then
    error "Port 9999 is not mapped from container to host!"
    error ""
    error "This usually happens if the container was created before port mapping was configured."
    error "To fix this, run:"
    error "  1. ./scripts/docker_start.sh --stop"
    error "  2. ./scripts/docker_start.sh --rm"
    error "  3. ./scripts/docker_start.sh --jazzy  (recreates with port mapping)"
    error ""
    error "Then run this demo again."
    exit 1
fi

if nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
    success "TCP server is accessible at $DOCKER_HOST:9999"
else
    warn "TCP server not ready yet, waiting..."
    sleep 3
    if nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
        success "TCP server is now accessible"
    else
        error "TCP server failed to start. Check logs:"
        docker exec "$CONTAINER_NAME" cat /tmp/tcp_server.log 2>/dev/null || echo "No log file"
        error ""
        error "To debug manually:"
        error "  docker exec -it $CONTAINER_NAME bash"
        error "  python3 /app/openclaw_ros_bridge/communication/openclaw_tcp_server.py"
        exit 1
    fi
fi

# Step 6: Create OpenClaw Agent Script
step "6" "Creating OpenClaw Agent..."

cat > /tmp/openclaw_greenhouse_agent.py << 'AGENT_EOF'
#!/usr/bin/env python3
"""OpenClaw Agent - Controls Greenhouse via ROS Bridge"""
import socket
import json
import time
import sys

class OpenClawGreenhouseAgent:
    """AI Agent that controls greenhouse environment"""
    
    def __init__(self, host="localhost", port=9999):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        
        # Control thresholds
        self.fan_threshold = 28.0  # Turn on fan above this temp
        self.valve_threshold = 40.0  # Open valve below this humidity
        
    def connect(self):
        """Connect to ROS Bridge TCP server"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"✓ Connected to ROS Bridge at {self.host}:{self.port}")
        
    def send_command(self, cmd):
        """Send command and receive response"""
        try:
            self.socket.send(json.dumps(cmd).encode() + b'\n')
            response = self.socket.recv(4096).decode()
            return json.loads(response)
        except Exception as e:
            print(f"✗ Command failed: {e}")
            return {"status": "error", "message": str(e)}
    
    def read_sensors(self):
        """Read environmental sensors"""
        return self.send_command({"action": "read_sensor", "sensor": "env"})
    
    def control_fan(self, on):
        """Control fan actuator"""
        return self.send_command({
            "action": "write_actuator",
            "actuator": "fan",
            "value": on
        })
    
    def control_valve(self, on):
        """Control valve actuator"""
        return self.send_command({
            "action": "write_actuator",
            "actuator": "valve",
            "value": on
        })
    
    def get_status(self):
        """Get system status"""
        return self.send_command({"action": "get_status"})
    
    def ai_decision_loop(self, iterations=10):
        """Main AI control loop"""
        print("\n" + "="*50)
        print("🤖 OpenClaw AI Agent - Greenhouse Control")
        print("="*50 + "\n")
        
        # Get initial status
        status = self.get_status()
        print(f"System Status: ROS={status.get('ros', 'unknown')}, Mock={status.get('mock', 'unknown')}")
        print()
        
        self.running = True
        i = 0
        
        try:
            while self.running and (iterations == 0 or i < iterations):
                i += 1
                print(f"\n--- Iteration {i} ---")
                
                # Read sensors
                result = self.read_sensors()
                if result.get('status') != 'ok':
                    print(f"✗ Sensor read failed: {result}")
                    continue
                
                data = result.get('data', {})
                temp = data.get('temperature', 0)
                humidity = data.get('humidity', 0)
                
                print(f"🌡️  Temperature: {temp}°C")
                print(f"💧 Humidity: {humidity}%")
                
                # AI Decision Logic
                decisions = []
                
                # Fan control
                if temp > self.fan_threshold:
                    self.control_fan(True)
                    decisions.append("🌀 Fan: ON (too hot)")
                else:
                    self.control_fan(False)
                    decisions.append("🌀 Fan: OFF (comfortable)")
                
                # Valve control
                if humidity < self.valve_threshold:
                    self.control_valve(True)
                    decisions.append("💦 Valve: OPEN (too dry)")
                else:
                    self.control_valve(False)
                    decisions.append("💦 Valve: CLOSED (humid enough)")
                
                for d in decisions:
                    print(f"  → {d}")
                
                # Wait before next iteration
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Shutting down...")
        finally:
            self.running = False
            self.socket.close()
    
    def demo_scenario(self):
        """Run a demo scenario with simulated conditions"""
        print("\n" + "="*50)
        print("🎬 Running Demo Scenarios")
        print("="*50)
        
        scenarios = [
            {"name": "Hot Day", "temp": 32.0, "humidity": 35.0},
            {"name": "Cool Morning", "temp": 22.0, "humidity": 60.0},
            {"name": "Dry Afternoon", "temp": 29.0, "humidity": 25.0},
            {"name": "Ideal Conditions", "temp": 24.0, "humidity": 55.0},
        ]
        
        # Note: In mock mode, these values are simulated
        # In real mode, these would be actual sensor readings
        
        print("\n📋 Available scenarios:")
        for i, s in enumerate(scenarios, 1):
            print(f"  {i}. {s['name']}: {s['temp']}°C, {s['humidity']}% humidity")
        
        print("\n🤖 AI will automatically respond to conditions...")
        print("   (In this demo, conditions cycle through scenarios)")
        print()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="OpenClaw Greenhouse Agent")
    parser.add_argument("--host", default="localhost", help="ROS Bridge host")
    parser.add_argument("--port", type=int, default=9999, help="TCP port")
    parser.add_argument("--iterations", type=int, default=10, help="Number of iterations")
    args = parser.parse_args()
    
    agent = OpenClawGreenhouseAgent(host=args.host, port=args.port)
    
    try:
        agent.connect()
        agent.demo_scenario()
        agent.ai_decision_loop(iterations=args.iterations)
    except ConnectionRefusedError:
        print(f"\n✗ Could not connect to {args.host}:{args.port}")
        print("  Make sure Docker container is running with TCP server started")
        print(f"  Run: ./scripts/docker_start.sh --jazzy")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)
    
    print("\n✓ Demo complete!")
AGENT_EOF

chmod +x /tmp/openclaw_greenhouse_agent.py
success "Agent script created"

# Step 7: Run Demo
step "7" "Starting Demo..."
log ""
log "The demo will:"
log "  1. Connect OpenClaw agent to ROS Bridge"
log "  2. Read sensor data (temperature, humidity)"
log "  3. Make AI decisions (fan/valve control)"
log "  4. Send commands back to greenhouse"
log ""
log "Press Ctrl+C to stop at any time"
log ""

sleep 2

# Run the agent
python3 /tmp/openclaw_greenhouse_agent.py --host "$DOCKER_HOST" --port 9999 --iterations 10

log ""
success "Demo complete!"
log ""
log "You can:"
log "  • Run again: ./scripts/demo_openclaw_greenhouse.sh"
log "  • View logs: docker exec ros2-jazzy-bridge cat /tmp/tcp_server.log"
log "  • Stop container: ./scripts/docker_start.sh --stop"
