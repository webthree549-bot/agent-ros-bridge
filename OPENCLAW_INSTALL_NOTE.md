# OpenClaw Installation - Syntax Note

**Issue:** `openclaw skills add agent-ros-bridge` returns:
```
error: too many arguments for 'skills'. Expected 0 arguments but got 2.
```

## Correct Installation Methods

### Method 1: Direct Package Installation (Recommended)

```bash
# Install from source
pip install ~/dev/agent-ros-bridge

# Or from built wheel
pip install ~/agent-ros-bridge-build-artifacts/dist/*.whl
```

### Method 2: Manual OpenClaw Skill Installation

```bash
# Run the install script
cd ~/dev/agent-ros-bridge
./install-openclaw-skill.sh
```

Or manually:
```bash
# Create OpenClaw skill directory
mkdir -p ~/.openclaw/skills/agent-ros-bridge

# Copy files
cp ~/dev/agent-ros-bridge/SKILL.md ~/.openclaw/skills/agent-ros-bridge/
cp -r ~/dev/agent-ros-bridge/agent_ros_bridge ~/.openclaw/skills/agent-ros-bridge/
cp -r ~/dev/agent-ros-bridge/config ~/.openclaw/skills/agent-ros-bridge/
```

### Method 3: Development Installation

```bash
cd ~/dev/agent-ros-bridge
pip install -e .
```

This makes it available system-wide for OpenClaw to use.

---

## Possible OpenClaw CLI Syntax

The correct OpenClaw command might be:

```bash
# Option 1: Different subcommand
openclaw skill install agent-ros-bridge

# Option 2: No subcommand
openclaw install agent-ros-bridge

# Option 3: From marketplace
openclaw marketplace install agent-ros-bridge

# Option 4: Add to skills list
openclaw skills install  # Interactive?
```

**Note:** Check `openclaw --help` or `openclaw skills --help` for correct syntax.

---

## Verification

After installation, verify it works:

```bash
# Test import
python3 -c "from agent_ros_bridge import Bridge; print('OK')"

# Test CLI
agent-ros-bridge --version

# Start demo
agent-ros-bridge --demo
```

---

## Summary

If `openclaw skills add` doesn't work, use:
```bash
pip install ~/dev/agent-ros-bridge
```

This installs the package system-wide, making it available to OpenClaw.
