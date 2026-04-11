# Troubleshooting Guide

**Version:** v0.6.7  
**Last Updated:** 2026-04-10

---

## Quick Diagnostics

Run the built-in diagnostic command:

```bash
agent-ros-bridge --diagnose
```

Or via Python:

```python
from agent_ros_bridge import diagnose

report = diagnose.full_check()
print(report.summary())
```

---

## Installation Issues

### "pip install agent-ros-bridge" fails

**Symptoms:**
```
ERROR: Could not find a version that satisfies the requirement agent-ros-bridge
```

**Solutions:**

1. **Check Python version** (requires 3.11+):
   ```bash
   python --version  # Should be 3.11, 3.12, 3.13, or 3.14
   ```

2. **Install from source**:
   ```bash
   git clone https://github.com/your-org/agent-ros-bridge.git
   cd agent-ros-bridge
   pip install -e ".[all]"
   ```

3. **Check for conflicting packages**:
   ```bash
   pip list | grep -i ros
   # Remove any conflicting packages
   pip uninstall ros-bridge-other
   ```

---

## Connection Issues

### "Connection refused" to WebSocket

**Symptoms:**
```
websockets.exceptions.InvalidStatusCode: server rejected WebSocket connection: HTTP 403
```

**Solutions:**

1. **Check bridge is running**:
   ```bash
   ps aux | grep agent-ros-bridge
   # If not running:
   agent-ros-bridge --websocket-port 8765
   ```

2. **Verify port is available**:
   ```bash
   lsof -i :8765  # On Linux/Mac
   netstat -an | findstr 8765  # On Windows
   ```

3. **Check JWT token**:
   ```python
   import jwt
   # Verify your token is valid and not expired
   decoded = jwt.decode(token, secret, algorithms=["HS256"])
   ```

4. **Firewall settings**:
   ```bash
   # Check firewall
   sudo ufw status  # Ubuntu
   sudo firewall-cmd --list-ports  # RHEL/CentOS
   ```

---

### "Cannot connect to ROS"

**Symptoms:**
```
ROSConnectionError: Failed to connect to ROS master at http://localhost:11311
```

**Solutions:**

1. **Start ROS master**:
   ```bash
   # ROS1
   roscore
   
   # ROS2
   ros2 daemon start
   ```

2. **Check ROS_MASTER_URI**:
   ```bash
   echo $ROS_MASTER_URI  # Should be http://localhost:11311
   export ROS_MASTER_URI=http://localhost:11311
   ```

3. **Verify ROS installation**:
   ```bash
   # ROS1
   rostopic list
   
   # ROS2
   ros2 topic list
   ```

4. **Network issues**:
   ```bash
   # Check if ROS master is reachable
   ping $(echo $ROS_MASTER_URI | sed 's|http://||' | cut -d: -f1)
   ```

---

## Robot Control Issues

### "Robot not responding to commands"

**Symptoms:**
- Commands return success but robot doesn't move
- No error messages

**Solutions:**

1. **Check robot is powered on**:
   ```python
   status = robot.get_status()
   print(f"Power: {status['power']}")
   print(f"E-stop: {status['emergency_stop']}")
   ```

2. **Verify ROS topics**:
   ```bash
   # Check cmd_vel is being published
   rostopic echo /cmd_vel  # ROS1
   ros2 topic echo /cmd_vel  # ROS2
   ```

3. **Check motor controllers**:
   ```bash
   # Check for motor controller errors
   rostopic echo /diagnostics  # ROS1
   ros2 topic echo /diagnostics  # ROS2
   ```

4. **Test manual control**:
   ```bash
   # Publish test velocity
   rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
   ```

---

### "Navigation failed"

**Symptoms:**
```
NavigationError: Failed to reach goal
```

**Solutions:**

1. **Check map is loaded**:
   ```bash
   # Check map topic
   rostopic info /map
   ros2 topic info /map
   ```

2. **Verify goal is reachable**:
   ```python
   from agent_ros_bridge import RobotAgent
   
   robot = RobotAgent()
   # Check if path exists
   path = robot.plan_path(goal_x, goal_y)
   if path is None:
       print("Goal is unreachable - check for obstacles")
   ```

3. **Check localization**:
   ```bash
   # Verify AMCL is working
   rostopic echo /amcl_pose  # ROS1
   ros2 topic echo /amcl_pose  # ROS2
   ```

4. **Obstacle detection**:
   ```bash
   # Check for obstacles
   rostopic echo /scan  # LiDAR
   rostopic echo /bumper  # Bumpers
   ```

---

## Performance Issues

### "High latency in responses"

**Symptoms:**
- Commands take seconds to execute
- Telemetry updates are delayed

**Solutions:**

1. **Check network latency**:
   ```bash
   ping <robot_ip>
   # Should be < 10ms for local network
   ```

2. **Reduce telemetry rate**:
   ```python
   from agent_ros_bridge import BridgeConfig
   
   config = BridgeConfig(
       telemetry_rate_hz=1,  # Reduce from default 10Hz
   )
   ```

3. **Check CPU usage**:
   ```bash
   top  # Linux
   htop  # Better view
   ```

4. **Profile the code**:
   ```python
   import cProfile
   cProfile.run('robot.execute("move forward")')
   ```

---

### "Memory usage growing over time"

**Symptoms:**
- Bridge memory usage increases continuously
- Eventually crashes with OOM

**Solutions:**

1. **Check for memory leaks**:
   ```python
   import tracemalloc
   tracemalloc.start()
   # ... run operations ...
   snapshot = tracemalloc.take_snapshot()
   top_stats = snapshot.statistics('lineno')
   for stat in top_stats[:10]:
       print(stat)
   ```

2. **Limit history size**:
   ```python
   config = BridgeConfig(
       max_command_history=1000,
       max_telemetry_history=1000,
   )
   ```

3. **Restart periodically** (workaround):
   ```bash
   # Use systemd to auto-restart
   systemctl restart agent-ros-bridge
   ```

---

## Safety System Issues

### "Emergency stop not working"

**Symptoms:**
- E-stop command sent but robot keeps moving
- No response to emergency commands

**Solutions:**

1. **Check safety node status**:
   ```bash
   # ROS1
   rostopic echo /safety/emergency_stop/status
   
   # ROS2
   ros2 topic echo /safety/emergency_stop/status
   ```

2. **Verify hardware E-stop**:
   ```python
   from agent_ros_bridge.safety import EmergencyStop
   
   e_stop = EmergencyStop()
   status = e_stop.get_status()
   print(f"Hardware E-stop: {status['hardware_engaged']}")
   print(f"Software E-stop: {status['software_engaged']}")
   ```

3. **Test software E-stop**:
   ```python
   await robot.emergency_stop()
   # Check if cmd_vel is zeroed
   ```

---

### "Safety validation rejecting valid commands"

**Symptoms:**
- Safe commands are being rejected
- Safety violations reported incorrectly

**Solutions:**

1. **Check safety limits**:
   ```python
   from agent_ros_bridge.safety import SafetyLimits
   
   limits = SafetyLimits.load('config/safety_limits.yaml')
   print(f"Max velocity: {limits.max_linear_velocity}")
   ```

2. **Adjust confidence threshold**:
   ```python
   robot = RobotAgent(
       min_confidence_for_auto=0.90,  # Lower from 0.95
   )
   ```

3. **Check workspace bounds**:
   ```python
   # Verify robot is within configured workspace
   position = robot.get_position()
   if not limits.is_within_workspace(position):
       print("Robot outside configured workspace!")
   ```

---

## Fleet Management Issues

### "Robot not appearing in fleet"

**Symptoms:**
- Fleet status shows fewer robots than expected
- Discovery not finding all robots

**Solutions:**

1. **Check network discovery**:
   ```python
   from agent_ros_bridge.discovery import ROSDiscovery
   
   discovery = ROSDiscovery()
   robots = discovery.discover_all(timeout=10.0)
   print(f"Found {len(robots)} robots")
   ```

2. **Verify robot is publishing status**:
   ```bash
   rostopic echo /robot_status  # ROS1
   ros2 topic echo /robot_status  # ROS2
   ```

3. **Check namespace configuration**:
   ```bash
   # Ensure robots have unique namespaces
   echo $ROS_NAMESPACE  # Should be unique per robot
   ```

---

### "Task assignment failing"

**Symptoms:**
- Tasks not being assigned to robots
- Assignment returns errors

**Solutions:**

1. **Check robot status**:
   ```python
   from agent_ros_bridge.fleet import FleetOrchestrator
   
   fleet = FleetOrchestrator()
   status = await fleet.get_fleet_status()
   for rid, info in status.items():
       print(f"{rid}: {info['status']}")  # Should be IDLE
   ```

2. **Verify capabilities**:
   ```python
   robot = fleet.get_robot('bot1')
   print(robot.capabilities)  # Check if required capability exists
   ```

3. **Check battery levels**:
   ```python
   if robot.battery_level < 20:
       print("Robot battery too low for task assignment")
   ```

---

## Docker Issues

### "Cannot connect to ROS in Docker"

**Symptoms:**
- Bridge runs in Docker but can't connect to host ROS

**Solutions:**

1. **Use host network**:
   ```bash
   docker run --network=host agent-ros-bridge
   ```

2. **Expose ROS ports**:
   ```bash
   docker run -p 11311:11311 agent-ros-bridge
   ```

3. **Share ROS master**:
   ```bash
   # On host
   export ROS_IP=<host_ip>
   
   # In container
   export ROS_MASTER_URI=http://<host_ip>:11311
   ```

---

### "Gazebo simulation not starting in Docker"

**Symptoms:**
- Gazebo fails to start
- No display output

**Solutions:**

1. **Enable X11 forwarding**:
   ```bash
   xhost +local:docker
   docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix agent-ros-bridge
   ```

2. **Use headless mode**:
   ```bash
   export GAZEBO_HEADLESS=1
   gz sim -s  # Server-only mode
   ```

3. **Check GPU access**:
   ```bash
   # For NVIDIA GPUs
   docker run --gpus all agent-ros-bridge
   ```

---

## Getting Help

### Collect Diagnostic Information

```python
from agent_ros_bridge import diagnose

# Generate full diagnostic report
report = diagnose.full_check()
report.save('diagnostic_report.json')

# Include in bug report:
# - diagnostic_report.json
# - ~/.agent_ros_bridge/logs/
# - ROS logs: ~/.ros/log/
```

### Log Locations

| Log Type | Location |
|----------|----------|
| Bridge logs | `~/.agent_ros_bridge/logs/` |
| ROS1 logs | `~/.ros/log/` |
| ROS2 logs | `~/.ros/log/` |
| System logs | `/var/log/syslog` (Linux) |

### Bug Report Template

```markdown
**Version:** v0.6.7
**OS:** Ubuntu 22.04 / macOS 14 / etc.
**ROS:** ROS2 Jazzy / ROS1 Noetic
**Python:** 3.12

**Problem:**
[Clear description]

**Steps to Reproduce:**
1. [Step 1]
2. [Step 2]
3. [Step 3]

**Expected Behavior:**
[What should happen]

**Actual Behavior:**
[What actually happens]

**Logs:**
[Attach diagnostic_report.json and relevant logs]
```

### Community Support

- **GitHub Issues:** https://github.com/your-org/agent-ros-bridge/issues
- **Documentation:** https://docs.agent-ros-bridge.ai
- **ROS Discourse:** Tag with `agent-ros-bridge`

---

## Common Error Codes

| Error Code | Meaning | Solution |
|------------|---------|----------|
| E001 | Connection refused | Check if bridge is running |
| E002 | Authentication failed | Verify JWT token |
| E003 | ROS not available | Start ROS master |
| E004 | Robot not found | Check discovery |
| E005 | Command timeout | Increase timeout or check robot |
| E006 | Safety violation | Review safety limits |
| E007 | Navigation failed | Check map and localization |
| E008 | Fleet error | Check robot status |

---

*Last updated: 2026-04-10*  
*Version: v0.6.7*
