# Test Agent ROS Bridge in OpenClaw

This guide shows how to test the bridge as an OpenClaw skill.

---

## Quick Test

### Option 1: Install in OpenClaw Skills Directory

```bash
cd ~/dev/agent-ros-bridge

# Install to OpenClaw
mkdir -p ~/.openclaw/skills/agent-ros-bridge
cp SKILL.md ~/.openclaw/skills/agent-ros-bridge/
cp -r agent_ros_bridge ~/.openclaw/skills/agent-ros-bridge/
cp -r config ~/.openclaw/skills/agent-ros-bridge/

# Test import
python3 -c "
import sys
sys.path.insert(0, '~/.openclaw/skills/agent-ros-bridge')
from agent_ros_bridge import Bridge
print('✓ OpenClaw can import Bridge')
"
```

### Option 2: Install from Built Wheel

```bash
# Install the built package
pip install --user ~/agent-ros-bridge-build-artifacts/dist/*.whl

# Test it works
agent-ros-bridge --version
python3 -c "from agent_ros_bridge import Bridge; print('OK')"
```

---

## Full OpenClaw Integration Test

### Step 1: Install Package

```bash
# Use the built wheel
pip install ~/agent-ros-bridge-build-artifacts/dist/agent_ros_bridge-*.whl
```

### Step 2: Test Python Import

```bash
# Test OpenClaw can import it
python3 << 'EOF'
from agent_ros_bridge import Bridge
from agent_ros_bridge import Robot, RobotFleet

print("✓ All imports successful")
print(f"Bridge version: {Bridge.__module__}")
EOF
```

### Step 3: Test CLI

```bash
agent-ros-bridge --version
agent-ros-bridge --help
```

### Step 4: Start Bridge Service

```bash
# Start in demo mode
agent-ros-bridge --demo &
BRIDGE_PID=$!
sleep 3

# Test it's running
curl http://localhost:8765/health
echo "✓ Bridge running"

# Stop when done
kill $BRIDGE_PID
```

### Step 5: Test from OpenClaw Agent Context

Create a test script that simulates OpenClaw using the bridge:

```bash
cat > /tmp/test_openclaw_agent.py << 'EOF'
#!/usr/bin/env python3
"""Test OpenClaw agent using Agent ROS Bridge"""

import asyncio
import json

# Test 1: Import
print("Test 1: Import agent_ros_bridge...")
from agent_ros_bridge import Bridge
print("✓ Import successful")

# Test 2: Create Bridge
print("\nTest 2: Create Bridge instance...")
bridge = Bridge()
print("✓ Bridge created")

# Test 3: Check version
print("\nTest 3: Check version...")
import agent_ros_bridge
print(f"✓ Version: {agent_ros_bridge.__version__}")

print("\n✅ All OpenClaw integration tests passed!")
print("\nThe bridge is ready to be used by OpenClaw agents.")
EOF

python3 /tmp/test_openclaw_agent.py
```

---

## OpenClaw Skill Structure

When OpenClaw installs a skill, it expects:

```
~/.openclaw/skills/agent-ros-bridge/
├── SKILL.md              # Skill manifest
├── agent_ros_bridge/     # Python package
│   ├── __init__.py
│   └── ...
└── config/               # Configuration files
    └── ...
```

---

## Automated Test Script

Use the provided script:

```bash
cd ~/dev/agent-ros-bridge
./test-in-openclaw.sh manual    # Manual test
./test-in-openclaw.sh install   # Install to OpenClaw
./test-in-openclaw.sh demo      # Start demo
./test-in-openclaw.sh clean     # Clean up
```

---

## Verification Checklist

- [ ] Package installs without errors
- [ ] `from agent_ros_bridge import Bridge` works
- [ ] CLI `agent-ros-bridge --version` works
- [ ] Bridge starts with `agent-ros-bridge --demo`
- [ ] Health endpoint responds
- [ ] Can be imported in OpenClaw context

---

## Troubleshooting

### Import Error
```bash
# Reinstall
pip uninstall agent-ros-bridge -y
pip install ~/agent-ros-bridge-build-artifacts/dist/*.whl
```

### Port Already in Use
```bash
# Kill existing bridge
pkill -f "agent-ros-bridge" || true
# Or use different port
agent-ros-bridge --websocket-port 8766
```

### Permission Error
```bash
# Use --user flag
pip install --user ~/agent-ros-bridge-build-artifacts/dist/*.whl
```

---

## Success Criteria

✅ **OpenClaw Integration Working When:**

1. Package installs successfully
2. Can import `from agent_ros_bridge import Bridge`
3. CLI commands work
4. Bridge service starts
5. OpenClaw agent can connect and use it

---

**Ready for OpenClaw!** 🤖
