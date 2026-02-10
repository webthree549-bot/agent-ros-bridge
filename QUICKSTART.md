# 🤖 OpenClaw Robot Control - Quick Start

## One-Command Deployment

```bash
cd /Volumes/2nd-HD/openclaw-ros-bridge
./openclaw-robot deploy
```

## One-Command Control from OpenClaw

```bash
# Check status
./openclaw-robot-skill status

# Read sensors
./openclaw-robot-skill read

# Control actuators
./openclaw-robot-skill "fan on"
./openclaw-robot-skill "valve open"
```

## Interactive Mode

```bash
./openclaw-robot cmd
```

Then type commands interactively.

## Full Command Reference

| Command | Description |
|---------|-------------|
| `./openclaw-robot deploy` | Start robot fleet |
| `./openclaw-robot status` | Check status |
| `./openclaw-robot cmd` | Interactive control |
| `./openclaw-robot stop` | Shutdown |
| `./openclaw-robot-skill status` | Quick status check |
| `./openclaw-robot-skill read` | Read sensors |
| `./openclaw-robot-skill "fan on"` | Turn on fan |
| `./openclaw-robot-skill "fan off"` | Turn off fan |
| `./openclaw-robot-skill "valve open"` | Open valve |
| `./openclaw-robot-skill "valve close"` | Close valve |
