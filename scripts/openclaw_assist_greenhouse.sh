#!/bin/bash
# openclaw_assist_greenhouse.sh - OpenClaw AI assists with greenhouse control

set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

# Check OS
OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    DOCKER_HOST="localhost"
else
    DOCKER_HOST="ros2-jazzy-bridge"
fi

CONTAINER_NAME="ros2-jazzy-bridge"

echo "================================================"
echo "  🤖 OpenClaw Greenhouse Assistant"
echo "================================================"
echo ""
echo "I'm your AI assistant for greenhouse control."
echo "I'll help you monitor and manage the environment."
echo ""

# Ensure container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting Docker container..."
    ./scripts/docker_start.sh --jazzy &
    sleep 5
fi

# Ensure build
if ! docker exec "$CONTAINER_NAME" test -d /app/install; then
    echo "Building project..."
    docker exec "$CONTAINER_NAME" bash -c "cd /app && source /opt/ros/jazzy/setup.bash && ./scripts/build.sh" > /dev/null 2>&1
fi

# Start TCP server
if ! docker exec "$CONTAINER_NAME" pgrep -f "openclaw_tcp_server" > /dev/null 2>&1; then
    echo "Starting TCP server..."
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
        echo "✓ Connected to greenhouse system"
        break
    fi
    sleep 1
done

echo ""
echo "================================================"
echo "  🌱 GREENHOUSE CONTROL PANEL"
echo "================================================"
echo ""
echo "STATUS CHECK:"
echo "-------------"

# Create Python helper
cat > /tmp/gh_helper.py << 'EOF'
import socket, json, sys
HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 9999

def cmd(c):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send(json.dumps(c).encode() + b'\n')
    r = json.loads(s.recv(4096).decode())
    s.close()
    return r

action = sys.argv[2]
if action == "read":
    r = cmd({"action": "read_sensor", "sensor": "env"})
    d = r.get("data", {})
    print(f"TEMP:{d.get('temperature','N/A')}")
    print(f"HUMIDITY:{d.get('humidity','N/A')}")
elif action == "fan_on":
    cmd({"action": "write_actuator", "actuator": "fan", "value": True})
    print("Fan ON")
elif action == "fan_off":
    cmd({"action": "write_actuator", "actuator": "fan", "value": False})
    print("Fan OFF")
elif action == "valve_open":
    cmd({"action": "write_actuator", "actuator": "valve", "value": True})
    print("Valve OPEN")
elif action == "valve_close":
    cmd({"action": "write_actuator", "actuator": "valve", "value": False})
    print("Valve CLOSED")
EOF

# Get current status
python3 /tmp/gh_helper.py "$DOCKER_HOST" read | while read line; do
    key=${line%%:*}
    val=${line#*:}
    if [ "$key" = "TEMP" ]; then
        echo "🌡️  Temperature: ${val}°C"
    elif [ "$key" = "HUMIDITY" ]; then
        echo "💧 Humidity: ${val}%"
    fi
done

echo ""
echo "================================================"
echo "  💬 ASK OPENCLAW AI TO HELP YOU"
echo "================================================"
echo ""
echo "Tell me what you want to do:"
echo ""
echo "  • 'Check the temperature'"
echo "  • 'Turn on the fan'"
echo "  • 'Water the plants'"
echo "  • 'Show me the status'"
echo "  • 'Run autopilot for 5 cycles'"
echo ""
echo "Or type commands directly:"
echo "  • ./scripts/gh_control.sh status"
echo "  • ./scripts/gh_control.sh fan on"
echo "  • ./scripts/gh_control.sh valve open"
echo ""
echo "================================================"
