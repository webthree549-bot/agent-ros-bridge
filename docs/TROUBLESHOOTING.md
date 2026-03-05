# Troubleshooting Guide

Common issues and solutions for Agent ROS Bridge.

## Table of Contents

1. [Installation Issues](#installation-issues)
2. [Connection Problems](#connection-problems)
3. [Authentication Errors](#authentication-errors)
4. [Robot Control Issues](#robot-control-issues)
5. [Performance Problems](#performance-problems)
6. [Framework Integration](#framework-integration)

---

## Installation Issues

### Problem: `pip install` fails

**Symptoms:**
```
ERROR: Could not find a version that satisfies the requirement agent-ros-bridge
```

**Solutions:**
1. Ensure Python >= 3.10:
   ```bash
   python --version  # Should be 3.10+
   ```

2. Upgrade pip:
   ```bash
   pip install --upgrade pip
   ```

3. Install from source:
   ```bash
   git clone https://github.com/webthree549-bot/agent-ros-bridge.git
   cd agent-ros-bridge
   pip install -e .
   ```

### Problem: ROS dependencies not found

**Symptoms:**
```
ImportError: No module named 'rclpy'
```

**Solutions:**
1. Source ROS environment:
   ```bash
   source /opt/ros/humble/setup.bash  # ROS2
   # or
   source /opt/ros/noetic/setup.bash  # ROS1
   ```

2. Install ROS bridge dependencies:
   ```bash
   pip install agent-ros-bridge[ros]
   ```

---

## Connection Problems

### Problem: Bridge won't start

**Symptoms:**
```
Error: Address already in use
```

**Solutions:**
1. Check if port is in use:
   ```bash
   lsof -i :8765  # WebSocket port
   lsof -i :1883  # MQTT port
   lsof -i :50051 # gRPC port
   ```

2. Kill existing process:
   ```bash
   kill -9 <PID>
   ```

3. Use different ports:
   ```bash
   agent-ros-bridge start --ws-port 8766 --mqtt-port 1884
   ```

### Problem: Cannot connect to robot

**Symptoms:**
```
Connection refused to ROS master
```

**Solutions:**
1. Check ROS master URI:
   ```bash
   echo $ROS_MASTER_URI  # ROS1
   echo $ROS_DOMAIN_ID   # ROS2
   ```

2. Start ROS master:
   ```bash
   roscore  # ROS1
   # or ensure ROS2 daemon is running
   ros2 daemon start
   ```

3. Check network connectivity:
   ```bash
   ping <robot-ip>
   ```

---

## Authentication Errors

### Problem: JWT token invalid

**Symptoms:**
```
Error: Invalid JWT token
```

**Solutions:**
1. Generate new token:
   ```bash
   agent-ros-bridge config --generate-token
   ```

2. Check token expiration:
   ```bash
   # Tokens expire after 24 hours by default
   # Generate long-lived token for production
   ```

3. Verify JWT secret:
   ```bash
   export JWT_SECRET="your-secret-key"
   ```

### Problem: Permission denied

**Symptoms:**
```
Error: Insufficient permissions
```

**Solutions:**
1. Check user roles in config
2. Verify token has required claims
3. Review authorization policies

---

## Robot Control Issues

### Problem: Commands not executing

**Symptoms:**
```
Command sent but robot not moving
```

**Solutions:**
1. Check robot connection:
   ```bash
   agent-ros-bridge status
   ```

2. Verify topic names:
   ```bash
   ros2 topic list  # ROS2
   rostopic list    # ROS1
   ```

3. Check message types:
   ```bash
   ros2 topic info /cmd_vel
   ```

4. Test with direct ROS:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'
   ```

### Problem: Natural language not understood

**Symptoms:**
```
Could not interpret command
```

**Solutions:**
1. Use simpler language
2. Check supported commands list
3. Enable debug mode:
   ```bash
   agent-ros-bridge start --log-level DEBUG
   ```

4. Add custom vocabulary:
   ```python
   bridge.add_vocabulary("custom_term", "ros_command")
   ```

---

## Performance Problems

### Problem: High latency

**Symptoms:**
```
Commands take seconds to execute
```

**Solutions:**
1. Enable Redis caching:
   ```bash
   export REDIS_URL=redis://localhost:6379
   ```

2. Check network latency:
   ```bash
   ping <bridge-host>
   ```

3. Optimize database:
   ```bash
   # Vacuum SQLite database
   sqlite3 bridge.db 'VACUUM;'
   ```

4. Use connection pooling

### Problem: Memory usage high

**Symptoms:**
```
Memory usage growing over time
```

**Solutions:**
1. Limit context history:
   ```python
   bridge.config['max_history'] = 100
   ```

2. Enable garbage collection
3. Use Redis instead of SQLite for large deployments

---

## Framework Integration

### Problem: LangChain tools not working

**Symptoms:**
```
Tool execution failed
```

**Solutions:**
1. Verify bridge URL:
   ```python
   client = AgentROSBridgeClient("http://localhost:8765")
   ```

2. Check authentication:
   ```python
   client = AgentROSBridgeClient("http://localhost:8765", token="your-token")
   ```

3. Test connection:
   ```python
   client.get_status()
   ```

### Problem: MCP server not connecting

**Symptoms:**
```
Could not connect to MCP server
```

**Solutions:**
1. Check Claude Desktop config:
   ```json
   {
     "mcpServers": {
       "agent-ros": {
         "command": "python",
         "args": ["-m", "agent_ros_bridge.frameworks.mcp.server"]
       }
     }
   }
   ```

2. Verify Python path
3. Check server logs

---

## Getting Help

If issues persist:

1. **Check logs:**
   ```bash
   agent-ros-bridge logs
   # or
   journalctl -u agent-ros-bridge -f
   ```

2. **Run diagnostics:**
   ```bash
   agent-ros-bridge doctor
   ```

3. **Enable debug mode:**
   ```bash
   agent-ros-bridge start --log-level DEBUG
   ```

4. **Contact support:**
   - GitHub Issues: https://github.com/webthree549-bot/agent-ros-bridge/issues
   - Discord: [Invite link]
   - Email: support@agentrosbridge.io

---

## Quick Diagnostics

Run this command to check system health:

```bash
agent-ros-bridge doctor
```

Expected output:
```
✅ Python version: 3.11.0
✅ ROS environment: sourced
✅ Bridge service: running
✅ Database: connected
✅ Redis: connected (optional)
✅ JWT secret: configured
✅ Network: reachable
```
