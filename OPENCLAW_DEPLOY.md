# Deploy Robot Control to OpenClaw
## Make OpenClaw Control Robots Directly

### Goal
Enable OpenClaw AI to control robots from chat:
```
User: "Check robot status"
OpenClaw: 🤖 Robot Fleet Status:
          ROS: jazzy, Mode: Simulation
          Temperature: 25°C, Humidity: 50%

User: "Turn on the fan"
OpenClaw: 🌀 Fan turned ON
```

---

## Method 1: Tool-Based Integration (Recommended)

Add robot control as an OpenClaw tool.

### Step 1: Create OpenClaw Tool Wrapper

Create `/Users/webthree/.openclaw/tools/robot_control.py`:

```python
#!/usr/bin/env python3
"""OpenClaw Tool - Robot Control"""
import subprocess
import json
import sys

def robot_control(command: str) -> str:
    """
    Control robot fleet via OpenClaw.
    
    Args:
        command: Natural language command or specific action
                Examples: "status", "read sensors", "fan on", "check temperature"
    
    Returns:
        Robot response or status
    """
    # Map natural language to skill commands
    cmd_map = {
        "status": "status",
        "check status": "status",
        "robot status": "status",
        "read": "read",
        "read sensors": "read",
        "check temperature": "read",
        "temperature": "read",
        "fan on": "fan on",
        "turn on fan": "fan on",
        "start fan": "fan on",
        "fan off": "fan off",
        "turn off fan": "fan off",
        "stop fan": "fan off",
        "valve open": "valve open",
        "open valve": "valve open",
        "water on": "valve open",
        "valve close": "valve close",
        "close valve": "valve close",
        "water off": "valve close",
        "ping": "ping",
        "test": "ping",
        "help": "help"
    }
    
    # Normalize command
    cmd_normalized = command.lower().strip()
    skill_cmd = cmd_map.get(cmd_normalized, cmd_normalized)
    
    # Execute via skill
    try:
        result = subprocess.run(
            ["/Volumes/2nd-HD/openclaw-ros-bridge/openclaw-robot-skill", skill_cmd],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            return result.stdout.strip()
        else:
            return f"Error: {result.stderr.strip()}"
            
    except subprocess.TimeoutExpired:
        return "Timeout: Robot control command took too long"
    except Exception as e:
        return f"Error controlling robot: {str(e)}"

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print(robot_control(" ".join(sys.argv[1:])))
    else:
        print(robot_control("status"))
```

### Step 2: Register as OpenClaw Tool

Add to OpenClaw configuration (`~/.openclaw/config.json` or similar):

```json
{
  "tools": [
    {
      "name": "robot_control",
      "command": "python3 /Users/webthree/.openclaw/tools/robot_control.py",
      "description": "Control robot fleet - status, sensors, actuators"
    }
  ]
}
```

Or create a shell alias in OpenClaw's environment.

---

## Method 2: Direct Command Execution (Simplest)

OpenClaw can directly execute the robot skill:

```python
# In this OpenClaw session
import subprocess

def robot(cmd):
    """Execute robot command"""
    result = subprocess.run(
        ["/Volumes/2nd-HD/openclaw-ros-bridge/openclaw-robot-skill", cmd],
        capture_output=True,
        text=True
    )
    return result.stdout if result.returncode == 0 else result.stderr

# Now use it
print(robot("status"))
print(robot("read"))
print(robot("fan on"))
```

---

## Method 3: Create OpenClaw Skill Package

### Structure
```
~/.openclaw/skills/robot-control/
├── SKILL.md
├── robot_skill.py
└── config.yaml
```

### SKILL.md
```markdown
---
name: robot-control
description: Control ROS-based robot fleet
---

# Robot Control Skill

Commands:
- `robot status` - Check fleet status
- `robot read` - Read sensors
- `robot fan on/off` - Control fan
- `robot valve open/close` - Control valve
```

### robot_skill.py
```python
#!/usr/bin/env python3
import subprocess
import sys

def main():
    cmd = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "status"
    result = subprocess.run(
        ["/Volumes/2nd-HD/openclaw-ros-bridge/openclaw-robot-skill", cmd],
        capture_output=True,
        text=True
    )
    print(result.stdout if result.returncode == 0 else result.stderr)

if __name__ == "__main__":
    main()
```

---

## Method 4: Wrapper Script (Immediate Use)

Create `/Users/webthree/.openclaw/workspace/robot`:

```bash
#!/bin/bash
# OpenClaw Robot Wrapper

cd /Volumes/2nd-HD/openclaw-ros-bridge
./openclaw-robot-skill "$@"
```

Make executable:
```bash
chmod +x /Users/webthree/.openclaw/workspace/robot
```

Now from OpenClaw:
```python
import subprocess
result = subprocess.run(["/Users/webthree/.openclaw/workspace/robot", "status"], capture_output=True, text=True)
print(result.stdout)
```

---

## Method 5: Direct Integration in This Session

### Quick Test (Run Now)

Copy-paste this into your terminal:

```bash
# Create wrapper
cat > /tmp/robot << 'EOF'
#!/bin/bash
cd /Volumes/2nd-HD/openclaw-ros-bridge
./openclaw-robot-skill "$@"
EOF
chmod +x /tmp/robot

# Test
/tmp/robot status
```

### In This OpenClaw Session

I can now execute robot commands directly:

```python
# I'll execute this for you
import subprocess

def check_robots():
    result = subprocess.run(
        ["/tmp/robot", "status"],
        capture_output=True,
        text=True
    )
    return result.stdout

def read_sensors():
    result = subprocess.run(
        ["/tmp/robot", "read"],
        capture_output=True,
        text=True
    )
    return result.stdout

# Execute
print(check_robots())
```

---

## 🎯 Recommended: Quick Deploy

**One-liner to enable OpenClaw robot control:**

```bash
# 1. Create wrapper
cat > ~/.local/bin/robot << 'EOF'
#!/bin/bash
cd /Volumes/2nd-HD/openclaw-ros-bridge
./openclaw-robot-skill "$@"
EOF
chmod +x ~/.local/bin/robot

# 2. Ensure fleet is deployed
robot deploy 2>/dev/null || /Volumes/2nd-HD/openclaw-ros-bridge/openclaw-robot deploy

# 3. Test
robot status
```

**Then in OpenClaw chat:**
```
User: "robot status"
OpenClaw: [executes ~/.local/bin/robot status]
```

Or I can add robot control as a tool I can invoke directly.

---

## ✅ What's the Easiest?

**Option A: I execute directly**
- I run robot commands via subprocess
- You see results in chat
- No setup needed

**Option B: Create wrapper script**
- Create `/tmp/robot` wrapper
- You type commands in terminal
- I can also call it

**Option C: Full skill package**
- Create proper OpenClaw skill
- Integrates into tool system
- More work but cleaner

**Which do you prefer?**
