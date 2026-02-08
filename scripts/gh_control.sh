#!/bin/bash
# gh_control.sh - Simple greenhouse control commands
# Usage: ./gh_control.sh [status|fan on|fan off|valve open|valve close|auto]

set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$PROJECT_ROOT"

OS=$(uname -s)
if [[ "$OS" == "Darwin" ]]; then
    DOCKER_HOST="localhost"
else
    DOCKER_HOST="ros2-jazzy-bridge"
fi

CONTAINER_NAME="ros2-jazzy-bridge"

# Ensure container running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container not running. Start with:"
    echo "  ./scripts/docker_start.sh --jazzy"
    exit 1
fi

# Python helper
cat > /tmp/gh_cmd.py << 'EOF'
import socket, json, sys, time
HOST = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = 9999

def send_cmd(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send(json.dumps(cmd).encode() + b'\n')
        r = json.loads(s.recv(4096).decode())
        s.close()
        return r
    except Exception as e:
        return {"status": "error", "message": str(e)}

action = sys.argv[2] if len(sys.argv) > 2 else "status"

if action == "status":
    r = send_cmd({"action": "get_status"})
    print(f"ROS: {r.get('ros', 'unknown')}")
    print(f"Mock: {r.get('mock', 'unknown')}")
    
    r = send_cmd({"action": "read_sensor", "sensor": "env"})
    d = r.get("data", {})
    print(f"Temperature: {d.get('temperature', 'N/A')}°C")
    print(f"Humidity: {d.get('humidity', 'N/A')}%")

elif action == "fan_on" or action == "fan on":
    r = send_cmd({"action": "write_actuator", "actuator": "fan", "value": True})
    print("✓ Fan turned ON")

elif action == "fan_off" or action == "fan off":
    r = send_cmd({"action": "write_actuator", "actuator": "fan", "value": False})
    print("✓ Fan turned OFF")

elif action == "valve_open" or action == "valve open":
    r = send_cmd({"action": "write_actuator", "actuator": "valve", "value": True})
    print("✓ Valve OPENED")

elif action == "valve_close" or action == "valve close":
    r = send_cmd({"action": "write_actuator", "actuator": "valve", "value": False})
    print("✓ Valve CLOSED")

elif action == "auto":
    print("🤖 Running AI autopilot for 5 cycles...")
    for i in range(5):
        print(f"\n--- Cycle {i+1}/5 ---")
        r = send_cmd({"action": "read_sensor", "sensor": "env"})
        d = r.get("data", {})
        temp = d.get("temperature", 0)
        humid = d.get("humidity", 0)
        print(f"  Temp: {temp}°C | Humidity: {humid}%")
        
        if temp > 28:
            send_cmd({"action": "write_actuator", "actuator": "fan", "value": True})
            print("  🌀 Fan ON (too hot)")
        else:
            send_cmd({"action": "write_actuator", "actuator": "fan", "value": False})
            print("  🌀 Fan OFF")
        
        if humid < 40:
            send_cmd({"action": "write_actuator", "actuator": "valve", "value": True})
            print("  💦 Valve OPEN (too dry)")
        else:
            send_cmd({"action": "write_actuator", "actuator": "valve", "value": False})
            print("  💦 Valve CLOSED")
        
        if i < 4:
            time.sleep(2)
    print("\n✓ Autopilot complete")

else:
    print(f"Unknown action: {action}")
    print("Usage: ./gh_control.sh [status|fan on|fan off|valve open|valve close|auto]")
EOF

# Run command
python3 /tmp/gh_cmd.py "$DOCKER_HOST" "${1:-status}" "${2:-}"
