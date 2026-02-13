#!/bin/bash
# openclaw_interactive_greenhouse.sh - Control greenhouse through OpenClaw AI

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

log() { echo -e "${BLUE}[OPENCLAW] $1${NC}"; }
success() { echo -e "${GREEN}[✓] $1${NC}"; }
warn() { echo -e "${YELLOW}[!] $1${NC}"; }
error() { echo -e "${RED}[✗] $1${NC}"; }
ai() { echo -e "${MAGENTA}[🤖 AI] $1${NC}"; }
info() { echo -e "${CYAN}[ℹ] $1${NC}"; }

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

# Check if running on macOS
OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    DOCKER_HOST="localhost"
else
    DOCKER_HOST="ros2-jazzy-bridge"
fi

CONTAINER_NAME="ros2-jazzy-bridge"

log "================================================"
log "  🤖 OpenClaw Interactive Greenhouse Control"
log "================================================"
log ""
log "This demo lets YOU control the greenhouse"
log "through OpenClaw AI assistance"
log ""

# Check/Start Docker
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log "Starting Docker container..."
    ./scripts/docker_start.sh --jazzy &
    sleep 5
fi

# Ensure build
if ! docker exec "$CONTAINER_NAME" test -d /app/install; then
    log "Building project..."
    docker exec "$CONTAINER_NAME" bash -c "cd /app && source /opt/ros/jazzy/setup.bash && ./scripts/build.sh" > /dev/null 2>&1
fi

# Start TCP Server
if ! docker exec "$CONTAINER_NAME" pgrep -f "openclaw_tcp_server" > /dev/null 2>&1; then
    log "Starting TCP server..."
    docker exec -d "$CONTAINER_NAME" bash -c "
        cd /app && 
        source /opt/ros/jazzy/setup.bash && 
        source install/setup.bash &&
        export MOCK_MODE=true &&
        python3 /app/openclaw_ros_bridge/communication/openclaw_tcp_server.py > /tmp/tcp_server.log 2>&1
    "
    sleep 3
fi

# Wait for connection
for i in {1..5}; do
    if nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
        success "Connected to greenhouse system"
        break
    fi
    sleep 1
done

if ! nc -z "$DOCKER_HOST" 9999 2>/dev/null; then
    error "Could not connect to greenhouse system"
    exit 1
fi

cat << 'EOF'

╔══════════════════════════════════════════════════════════╗
║           🤖 OpenClaw Greenhouse Assistant               ║
╠══════════════════════════════════════════════════════════╣
║  I'm your AI assistant for greenhouse control.           ║
║  Tell me what you'd like to do!                          ║
╠══════════════════════════════════════════════════════════╣
║  COMMANDS:                                               ║
║    • "check status"  - View current conditions          ║
║    • "read sensors"  - Get temperature & humidity       ║
║    • "turn on fan"   - Activate cooling fan             ║
║    • "turn off fan"  - Deactivate fan                   ║
║    • "open valve"    - Open water valve                 ║
║    • "close valve"   - Close water valve                ║
║    • "auto mode"     - Let AI control everything        ║
║    • "demo"          - Run automated demo sequence      ║
║    • "help"          - Show all commands                ║
║    • "quit"          - Exit                             ║
╚══════════════════════════════════════════════════════════╝

EOF

# Create helper script for commands
cat > /tmp/greenhouse_cmd.py << 'PYEOF'
#!/usr/bin/env python3
import socket
import json
import sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 9999

def send_cmd(cmd):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        sock.send(json.dumps(cmd).encode() + b'\n')
        response = json.loads(sock.recv(4096).decode())
        sock.close()
        return response
    except Exception as e:
        return {"status": "error", "message": str(e)}

if len(sys.argv) < 3:
    print("Usage: greenhouse_cmd.py <host> <action> [args...]")
    sys.exit(1)

action = sys.argv[2]

if action == "status":
    print(json.dumps(send_cmd({"action": "get_status"})))
elif action == "read":
    print(json.dumps(send_cmd({"action": "read_sensor", "sensor": "env"})))
elif action == "fan_on":
    print(json.dumps(send_cmd({"action": "write_actuator", "actuator": "fan", "value": True})))
elif action == "fan_off":
    print(json.dumps(send_cmd({"action": "write_actuator", "actuator": "fan", "value": False})))
elif action == "valve_open":
    print(json.dumps(send_cmd({"action": "write_actuator", "actuator": "valve", "value": True})))
elif action == "valve_close":
    print(json.dumps(send_cmd({"action": "write_actuator", "actuator": "valve", "value": False})))
else:
    print(json.dumps({"status": "error", "message": "Unknown action"}))
PYEOF

chmod +x /tmp/greenhouse_cmd.py

# Function to execute commands
greenhouse_cmd() {
    python3 /tmp/greenhouse_cmd.py "$DOCKER_HOST" "$@" 2>/dev/null
}

# Function to show status
show_status() {
    local result=$(greenhouse_cmd status)
    local ros=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('ros','unknown'))")
    local mock=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('mock','unknown'))")
    
    result=$(greenhouse_cmd read)
    local temp=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('temperature','N/A'))")
    local humidity=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('humidity','N/A'))")
    
    echo ""
    echo "┌─────────────────────────────────────┐"
    echo "│      🌱 GREENHOUSE STATUS           │"
    echo "├─────────────────────────────────────┤"
    printf "│  🌡️  Temperature:  %5s °C        │\n" "$temp"
    printf "│  💧 Humidity:      %5s %%          │\n" "$humidity"
    echo "├─────────────────────────────────────┤"
    printf "│  ROS: %-10s  Mock: %-5s     │\n" "$ros" "$mock"
    echo "└─────────────────────────────────────┘"
    echo ""
}

# Main interaction loop
while true; do
    echo ""
    read -p "🤖 What would you like me to do? > " user_input
    echo ""
    
    # Convert to lowercase for matching
    input_lower=$(echo "$user_input" | tr '[:upper:]' '[:lower:]')
    
    case "$input_lower" in
        "quit"|"exit"|"q")
            ai "Goodbye! The greenhouse will continue running."
            log "To stop the container: ./scripts/docker_start.sh --stop"
            break
            ;;
            
        "help"|"?"|"commands")
            cat << 'EOF'
Available commands:
  • check status      - View current temperature, humidity, and system status
  • read sensors      - Get latest sensor readings
  • turn on fan       - Activate the cooling fan
  • turn off fan      - Turn off the cooling fan
  • open valve        - Open the water valve for irrigation
  • close valve       - Close the water valve
  • auto mode         - Let AI automatically control the greenhouse
  • demo              - Run a 10-iteration automated demo
  • help              - Show this help message
  • quit              - Exit interactive mode

You can also ask me in natural language:
  • "How hot is it?"
  • "Is the fan on?"
  • "Water the plants"
  • "Cool down the greenhouse"
EOF
            ;;
            
        "check status"|"status"|"show status")
            ai "Checking greenhouse status..."
            show_status
            ;;
            
        "read sensors"|"read"|"get sensors"|"sensors"|"how hot is it"|"what's the temperature"|"check temp")
            ai "Reading environmental sensors..."
            result=$(greenhouse_cmd read)
            temp=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('temperature','N/A'))")
            humidity=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('humidity','N/A'))")
            
            echo ""
            echo "📊 Current Readings:"
            echo "   🌡️  Temperature: ${temp}°C"
            echo "   💧 Humidity: ${humidity}%"
            echo ""
            
            # AI commentary
            if (( $(echo "$temp > 28" | bc -l) )); then
                ai "It's getting warm in there! Consider turning on the fan."
            elif (( $(echo "$temp < 22" | bc -l) )); then
                ai "It's quite cool. The plants might appreciate some warmth."
            else
                ai "Temperature looks comfortable for the plants."
            fi
            
            if (( $(echo "$humidity < 40" | bc -l) )); then
                ai "The air is dry. Opening the valve might help."
            elif (( $(echo "$humidity > 70" | bc -l) )); then
                ai "It's quite humid. Good ventilation would help."
            fi
            ;;
            
        "turn on fan"|"fan on"|"start fan"|"cool down"|"activate fan")
            ai "Turning on the cooling fan..."
            result=$(greenhouse_cmd fan_on)
            if echo "$result" | grep -q '"status": "ok"'; then
                success "Fan is now ON 🌀"
                ai "The greenhouse should start cooling down soon."
            else
                error "Failed to turn on fan"
            fi
            ;;
            
        "turn off fan"|"fan off"|"stop fan")
            ai "Turning off the cooling fan..."
            result=$(greenhouse_cmd fan_off)
            if echo "$result" | grep -q '"status": "ok"'; then
                success "Fan is now OFF"
                ai "Fan deactivated. Temperature may rise if it's warm."
            else
                error "Failed to turn off fan"
            fi
            ;;
            
        "open valve"|"valve open"|"water plants"|"start irrigation"|"water the plants")
            ai "Opening the water valve for irrigation..."
            result=$(greenhouse_cmd valve_open)
            if echo "$result" | grep -q '"status": "ok"'; then
                success "Valve is now OPEN 💦"
                ai "Plants are getting watered! Remember to close the valve after a few minutes."
            else
                error "Failed to open valve"
            fi
            ;;
            
        "close valve"|"valve close"|"stop watering"|"close water")
            ai "Closing the water valve..."
            result=$(greenhouse_cmd valve_close)
            if echo "$result" | grep -q '"status": "ok"'; then
                success "Valve is now CLOSED"
                ai "Irrigation stopped. The plants should be well-hydrated."
            else
                error "Failed to close valve"
            fi
            ;;
            
        "auto"|"auto mode"|"ai mode"|"let ai control"|"autopilot")
            ai "Switching to AI autopilot mode..."
            ai "I'll monitor the greenhouse and make optimal decisions."
            echo ""
            
            for i in {1..5}; do
                echo ""
                echo "--- AI Control Cycle $i/5 ---"
                
                result=$(greenhouse_cmd read)
                temp=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('temperature','0'))")
                humidity=$(echo "$result" | python3 -c "import sys,json; print(json.load(sys.stdin).get('data',{}).get('humidity','0'))")
                
                echo "🌡️  Temperature: ${temp}°C | 💧 Humidity: ${humidity}%"
                
                # AI decision logic
                decisions=0
                
                if (( $(echo "$temp > 28" | bc -l) )); then
                    ai "Temperature high (${temp}°C) - activating fan"
                    greenhouse_cmd fan_on > /dev/null
                    echo "   🌀 Fan: ON"
                    decisions=1
                else
                    greenhouse_cmd fan_off > /dev/null
                    echo "   🌀 Fan: OFF"
                fi
                
                if (( $(echo "$humidity < 40" | bc -l) )); then
                    ai "Humidity low (${humidity}%) - opening valve"
                    greenhouse_cmd valve_open > /dev/null
                    echo "   💦 Valve: OPEN"
                    decisions=1
                else
                    greenhouse_cmd valve_close > /dev/null
                    echo "   💦 Valve: CLOSED"
                fi
                
                if [ $decisions -eq 0 ]; then
                    ai "Conditions optimal - no action needed"
                fi
                
                if [ $i -lt 5 ]; then
                    echo "Waiting 3 seconds before next check..."
                    sleep 3
                fi
            done
            
            echo ""
            ai "Autopilot session complete. You can resume manual control."
            ;;
            
        "demo"|"run demo"|"show demo")
            ai "Running automated demo sequence..."
            ./scripts/demo_openclaw_greenhouse.sh
            ;;
            
        "")
            # Empty input, just show prompt again
            ;;
            
        *)
            ai "I didn't understand '$user_input'"
            ai "Try 'help' for a list of commands, or ask me naturally like:"
            ai "  • 'How hot is it?'"
            ai "  • 'Turn on the fan'"
            ai "  • 'Water the plants'"
            ;;
    esac
done

log ""
log "Interactive session ended."
log "The greenhouse system is still running in Docker."
log "To reconnect: ./scripts/openclaw_interactive_greenhouse.sh"
