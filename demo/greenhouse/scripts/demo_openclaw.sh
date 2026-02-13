#!/bin/bash
# demo/greenhouse/scripts/demo_openclaw.sh - Complete OpenClaw + Greenhouse Demo
# This is a DEMO application built on top of the generic ROS Bridge

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log() { echo -e "${BLUE}[GREENHOUSE-DEMO] $1${NC}"; }
success() { echo -e "${GREEN}[SUCCESS] $1${NC}"; }
warn() { echo -e "${YELLOW}[WARN] $1${NC}"; }
error() { echo -e "${RED}[ERROR] $1${NC}"; }
step() { echo -e "${CYAN}▶ STEP $1: $2${NC}"; }

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$PROJECT_ROOT"

log "================================================"
log "  OpenClaw + Greenhouse Demo (Application)"
log "================================================"
log ""
log "This demo shows how to build an application"
log "on top of the generic OpenClaw ROS Bridge."
log ""

OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    DOCKER_HOST="localhost"
else
    DOCKER_HOST="ros2-jazzy-bridge"
fi

CONTAINER_NAME="ros2-jazzy-bridge"

# Step 1: Check Docker
step "1" "Checking Docker..."
if ! docker info > /dev/null 2>&1; then
    error "Docker is not running. Please start Docker Desktop."
    exit 1
fi
success "Docker is running"

# Step 2: Start/Check Container
step "2" "Starting ROS Container..."

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log "Container already running"
elif docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log "Starting existing container..."
    docker start "$CONTAINER_NAME"
else
    log "Creating new container..."
    "$PROJECT_ROOT/scripts/docker_start.sh" --jazzy &
    sleep 5
fi

success "Container ready"

# Step 3: Start Greenhouse TCP Server
step "3" "Starting Greenhouse Demo Server..."

if ! docker exec "$CONTAINER_NAME" pgrep -f "greenhouse_server" > /dev/null 2>&1; then
    log "Starting greenhouse server in background..."
    docker exec -d "$CONTAINER_NAME" bash -c "
        cd /app && 
        source /opt/ros/jazzy/setup.bash && 
        export MOCK_MODE=true &&
        python3 demo/greenhouse/greenhouse_server.py > /tmp/greenhouse_server.log 2>&1
    "
    
    log "Waiting for server to start..."
    for i in {1..10}; do
        sleep 1
        if docker exec "$CONTAINER_NAME" bash -c "netstat -tlnp 2>/dev/null | grep -q ':9999' || ss -tlnp 2>/dev/null | grep -q ':9999'"; then
            success "Greenhouse server is listening on port 9999"
            break
        fi
        log "  Attempt $i/10..."
    done
else
    success "Greenhouse server already running"
fi

# Step 4: Test Connection
step "4" "Testing connection..."

if nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
    success "Server is accessible at $DOCKER_HOST:9999"
else
    warn "Server not ready yet, waiting..."
    sleep 3
    if nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
        success "Server is now accessible"
    else
        error "Server failed to start. Check logs:"
        docker exec "$CONTAINER_NAME" cat /tmp/greenhouse_server.log 2>/dev/null || echo "No log file"
        exit 1
    fi
fi

# Step 5: Show Available Commands
step "5" "Checking available commands..."

cat > /tmp/list_handlers.py << 'EOF'
import socket, json, sys
HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 9999

def send_cmd(cmd):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send(json.dumps(cmd).encode() + b'\n')
    r = json.loads(s.recv(4096).decode())
    s.close()
    return r

# Get status
r = send_cmd({"action": "get_status"})
print(f"ROS: {r.get('ros', 'unknown')}")
print(f"Mock: {r.get('mock', 'unknown')}")
print(f"\nAvailable handlers:")
for h in r.get('registered_handlers', []):
    print(f"  • {h}")
EOF

python3 /tmp/list_handlers.py "$DOCKER_HOST"

# Step 6: Run Demo Agent
step "6" "Running OpenClaw Agent..."
log ""
log "The demo agent will:"
log "  1. Connect to the greenhouse server"
log "  2. Read sensor data (temperature, humidity)"
log "  3. Make AI decisions (fan/valve control)"
log "  4. Send commands back to actuators"
log ""
log "Press Ctrl+C to stop at any time"
log ""

sleep 2

# Run the agent
cat > /tmp/greenhouse_agent.py << 'AGENT_EOF'
#!/usr/bin/env python3
"""OpenClaw Agent - Demo Greenhouse Controller"""
import socket
import json
import time
import sys

class GreenhouseAgent:
    """AI Agent that controls greenhouse environment"""
    
    def __init__(self, host="localhost", port=9999):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.fan_threshold = 28.0
        self.valve_threshold = 40.0
        
    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"✓ Connected to Greenhouse Server at {self.host}:{self.port}")
        
    def send_command(self, cmd):
        try:
            self.socket.send(json.dumps(cmd).encode() + b'\n')
            response = self.socket.recv(4096).decode()
            return json.loads(response)
        except Exception as e:
            print(f"✗ Command failed: {e}")
            return {"status": "error", "message": str(e)}
    
    def read_sensors(self):
        return self.send_command({"action": "read_sensor", "sensor": "env"})
    
    def control_fan(self, on):
        return self.send_command({
            "action": "write_actuator",
            "actuator": "fan",
            "value": on
        })
    
    def control_valve(self, on):
        return self.send_command({
            "action": "write_actuator",
            "actuator": "valve",
            "value": on
        })
    
    def get_status(self):
        return self.send_command({"action": "get_greenhouse_status"})
    
    def ai_decision_loop(self, iterations=10):
        print("\n" + "="*50)
        print("🤖 OpenClaw AI Agent - Greenhouse Control")
        print("="*50 + "\n")
        
        status = self.get_status()
        print(f"Plugin: {status.get('plugin', 'unknown')} v{status.get('version', 'unknown')}")
        print()
        
        self.running = True
        i = 0
        
        try:
            while self.running and (iterations == 0 or i < iterations):
                i += 1
                print(f"\n--- Iteration {i} ---")
                
                result = self.read_sensors()
                if result.get('status') != 'ok':
                    print(f"✗ Sensor read failed: {result}")
                    continue
                
                data = result.get('data', {})
                temp = data.get('temperature', 0)
                humidity = data.get('humidity', 0)
                
                print(f"🌡️  Temperature: {temp}°C")
                print(f"💧 Humidity: {humidity}%")
                
                decisions = []
                
                if temp > self.fan_threshold:
                    self.control_fan(True)
                    decisions.append("🌀 Fan: ON (too hot)")
                else:
                    self.control_fan(False)
                    decisions.append("🌀 Fan: OFF (comfortable)")
                
                if humidity < self.valve_threshold:
                    self.control_valve(True)
                    decisions.append("💦 Valve: OPEN (too dry)")
                else:
                    self.control_valve(False)
                    decisions.append("💦 Valve: CLOSED (humid enough)")
                
                for d in decisions:
                    print(f"  → {d}")
                
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\n\n🛑 Shutting down...")
        finally:
            self.running = False
            self.socket.close()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Greenhouse Demo Agent")
    parser.add_argument("--host", default="localhost", help="Server host")
    parser.add_argument("--port", type=int, default=9999, help="TCP port")
    parser.add_argument("--iterations", type=int, default=10, help="Number of iterations")
    args = parser.parse_args()
    
    agent = GreenhouseAgent(host=args.host, port=args.port)
    
    try:
        agent.connect()
        agent.ai_decision_loop(iterations=args.iterations)
    except ConnectionRefusedError:
        print(f"\n✗ Could not connect to {args.host}:{args.port}")
        print("  Make sure the greenhouse server is running")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)
    
    print("\n✓ Demo complete!")
AGENT_EOF

chmod +x /tmp/greenhouse_agent.py
python3 /tmp/greenhouse_agent.py --host "$DOCKER_HOST" --port 9999 --iterations 10

log ""
success "Demo complete!"
log ""
log "You can:"
log "  • Run again: ./demo/greenhouse/scripts/demo_openclaw.sh"
log "  • View logs: docker exec $CONTAINER_NAME cat /tmp/greenhouse_server.log"
log "  • Control manually: ./demo/greenhouse/scripts/gh_control.sh status"
log "  • Stop container: ./scripts/docker_start.sh --stop"
