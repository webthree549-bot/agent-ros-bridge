#!/usr/bin/env python3
"""
OpenClaw Greenhouse Client - For AI agent to control greenhouse via TCP
Usage: python3 greenhouse_client.py <command> [args]
"""
import socket
import json
import sys

HOST = "localhost"  # Docker exposes port to macOS host
PORT = 9999

def send_command(cmd):
    """Send command to greenhouse TCP server"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((HOST, PORT))
        sock.send(json.dumps(cmd).encode() + b'\n')
        response = json.loads(sock.recv(4096).decode())
        sock.close()
        return response
    except Exception as e:
        return {"status": "error", "message": str(e)}

def get_status():
    """Get greenhouse system status"""
    return send_command({"action": "get_status"})

def read_sensors():
    """Read temperature and humidity"""
    return send_command({"action": "read_sensor", "sensor": "env"})

def fan_on():
    """Turn on cooling fan"""
    return send_command({"action": "write_actuator", "actuator": "fan", "value": True})

def fan_off():
    """Turn off cooling fan"""
    return send_command({"action": "write_actuator", "actuator": "fan", "value": False})

def valve_open():
    """Open water valve"""
    return send_command({"action": "write_actuator", "actuator": "valve", "value": True})

def valve_close():
    """Close water valve"""
    return send_command({"action": "write_actuator", "actuator": "valve", "value": False})

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 greenhouse_client.py <command>")
        print("Commands: status, read, fan_on, fan_off, valve_open, valve_close")
        sys.exit(1)
    
    cmd = sys.argv[1]
    
    if cmd == "status":
        result = get_status()
        print(json.dumps(result))
    elif cmd == "read":
        result = read_sensors()
        print(json.dumps(result))
    elif cmd == "fan_on":
        result = fan_on()
        print(json.dumps(result))
    elif cmd == "fan_off":
        result = fan_off()
        print(json.dumps(result))
    elif cmd == "valve_open":
        result = valve_open()
        print(json.dumps(result))
    elif cmd == "valve_close":
        result = valve_close()
        print(json.dumps(result))
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)

if __name__ == "__main__":
    main()
