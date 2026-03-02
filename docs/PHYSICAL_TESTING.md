# Physical Robot Testing Guide

Comprehensive guide for testing agent-ros-bridge with physical robots.

## 🎯 Test Philosophy

### Safety First
1. **Never test alone** - Have a safety observer present
2. **Emergency stop** must be accessible at all times
3. **Start small** - Non-motion tests before motion
4. **Graduated exposure** - Small motions before complex patterns
5. **Document everything** - Logs, videos, sensor data

### Test Progression
```
Level 1: Connectivity (no risk)
    ↓
Level 2: Telemetry (no risk)
    ↓
Level 3: Non-motion commands (minimal risk)
    ↓
Level 4: Minimal motion (< 10cm movement)
    ↓
Level 5: Basic motions (rotation, short linear)
    ↓
Level 6: Complex patterns (squares, waypoints)
    ↓
Level 7: Autonomous behaviors
```

## 📝 Pre-Test Checklist

### Environment
- [ ] Clear area (2x the robot's size in all directions)
- [ ] Flat, non-slip surface
- [ ] No drop-offs, stairs, or obstacles
- [ ] Good lighting
- [ ] Emergency stop accessible
- [ ] First aid kit nearby

### Robot
- [ ] Battery > 50% (or connected to power)
- [ ] All sensors functional
- [ ] Motors responding normally
- [ ] Emergency stop button tested
- [ ] Firmware up to date
- [ ] Wireless connection stable

### Software
- [ ] agent-ros-bridge installed
- [ ] ROS/ROS2 environment configured
- [ ] Test framework downloaded
- [ ] Log directory writable
- [ ] Backup plan ready

## 🚀 Running Tests

### 1. Dry Run (No Commands Sent)
```bash
# ROS2 Robot (e.g., TurtleBot3)
python tests/physical/test_physical_robot.py \
    --ros2 \
    --uri ros2://0/ \
    --robot turtlebot3 \
    --dry-run \
    --suite basic
```

### 2. Basic Connectivity (Safe)
```bash
python tests/physical/test_physical_robot.py \
    --ros2 \
    --uri ros2://0/ \
    --robot turtlebot3 \
    --suite basic
```

### 3. Minimal Motion (Low Risk)
```bash
python tests/physical/test_physical_robot.py \
    --ros2 \
    --uri ros2://0/ \
    --robot turtlebot3 \
    --suite minimal
```

### 4. Full Test Suite (All Motion Tests)
```bash
python tests/physical/test_physical_robot.py \
    --ros2 \
    --uri ros2://0/ \
    --robot turtlebot3 \
    --suite full
```

### 5. ROS1 Robot (e.g., Fetch)
```bash
python tests/physical/test_physical_robot.py \
    --ros1 \
    --uri ros1:/// \
    --robot fetch \
    --suite basic
```

## 📊 Test Suites

### `basic` - Connectivity Only (Safe)
- Connection verification
- Telemetry reception
- Non-motion commands
- Tool discovery
- **No robot movement**

### `minimal` - Basic Motion (Low Risk)
- All `basic` tests
- Small forward motion (0.1 m/s, 1 second)
- **~10cm movement expected**

### `full` - Complete Test (Higher Risk)
- All `minimal` tests
- Rotation test (in-place)
- Square pattern (1m x 1m area)
- **Requires clear space**

## 🔧 Robot Configurations

### TurtleBot3 (ROS2)
```yaml
robot: turtlebot3
ros_version: ros2
default_uri: ros2://0/
cmd_vel_topic: /cmd_vel
odom_topic: /odom
max_linear_vel: 0.22  # m/s
max_angular_vel: 2.84  # rad/s
test_area: 2x2_meters
```

### Fetch (ROS1)
```yaml
robot: fetch
ros_version: ros1
default_uri: ros1:///
cmd_vel_topic: /base_controller/command
odom_topic: /odom
max_linear_vel: 1.0  # m/s
max_angular_vel: 1.0  # rad/s
test_area: 3x3_meters
safety_note: Has arm - keep clear
```

### TurtleSim (Simulation - Safe for Practice)
```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
python tests/physical/test_physical_robot.py \
    --ros2 \
    --uri ros2://0/ \
    --robot turtlesim \
    --suite full
```

## 📈 Interpreting Results

### Test Report Location
```
test_reports/
├── physical_test_<robot_id>_<timestamp>.json
└── physical_test_<robot_id>_<timestamp>.html (future)
```

### Success Criteria
| Test | Expected Result | Failure Indicators |
|------|----------------|-------------------|
| Connection | Robot responds | Timeout, auth error |
| Telemetry | > 3 msgs/sec | No data, high latency |
| Non-motion | Commands succeed | Error responses |
| Motion | Smooth movement | Jerky, wrong direction, no response |
| Safety | E-stop works | Delayed response |

### Common Issues

#### Connection Failures
```
Error: Failed to connect to robot
```
- Check ROS_MASTER_URI (ROS1) or ROS_DOMAIN_ID (ROS2)
- Verify network connectivity
- Check firewall settings

#### No Telemetry
```
Error: Only received 0 messages
```
- Verify topic names match (`/odom` vs `/odom_combined`)
- Check robot is actually publishing
- Increase timeout for slow robots

#### Motion Failures
```
Error: Command executed but no movement
```
- Check motor power/enabled state
- Verify velocity limits not exceeded
- Check for safety lockouts

## 🛡️ Safety Features

### Automatic Triggers
The test framework monitors:
- Telemetry timeout (> 5 seconds)
- Excessive velocity (> configured limits)
- Connection loss
- Manual interrupt (Ctrl+C)

### Emergency Stop
Triggered automatically or manually:
```python
# Automatic triggers
if telemetry_timeout > 5.0:
    emergency_stop("Telemetry timeout")

if velocity > max_limit:
    emergency_stop("Velocity limit exceeded")
```

### Velocity Limits (Configurable)
```python
limits = {
    "max_linear_velocity": 1.0,   # m/s
    "max_angular_velocity": 2.0,  # rad/s
    "max_joint_velocity": 1.0,    # rad/s
}
```

## 🎥 Recording Tests

### Recommended
- Screen recording of terminal
- Video of robot movement
- ROS bag recording:
  ```bash
  # ROS2
  ros2 bag record -o test_run /odom /cmd_vel /scan
  
  # ROS1
  rosbag record -O test_run.bag /odom /cmd_vel /scan
  ```

### Data to Capture
- Test report JSON
- Terminal output (redirect to file)
- Robot logs
- Video recording
- Environment photos

## 🔬 Adding Custom Tests

```python
async def test_my_custom_scenario(self):
    async def _test():
        # Your test logic here
        result = await self.robot.execute(Command(
            action="publish",
            parameters={
                "topic": "/my_topic",
                "type": "std_msgs/String",
                "data": {"data": "test"}
            }
        ))
        assert result["status"] == "published"
        
    return await self.run_test(_test, "My Custom Test")
```

Then add to a test suite:
```python
TEST_SUITES["custom"] = TestSuite(
    name="Custom Tests",
    description="My custom test scenarios",
    tests=["test_connection", "test_my_custom_scenario"],
    requires_motion=False,
    safety_level="low"
)
```

## 📞 Emergency Contacts

Before testing, ensure you have:
- Lab safety officer contact
- Robot vendor support
- IT support (for network issues)
- Nearby first aid certified person

## 🏆 Best Practices

1. **Always start with dry-run**
2. **Run basic suite first** - even on known robots
3. **Document anomalies** - log everything
4. **Test incrementally** - don't skip levels
5. **Have a spotter** - for motion tests
6. **Time limit tests** - don't run indefinitely
7. **Clean up** - disconnect properly
8. **Review logs** - check for warnings

## 🐛 Troubleshooting

### Import Errors
```bash
# Install agent-ros-bridge
pip install -e .

# Or from PyPI
pip install agent-ros-bridge
```

### ROS Not Found
```bash
# ROS2
source /opt/ros/humble/setup.bash

# ROS1
source /opt/ros/noetic/setup.bash
```

### Permission Denied
```bash
# Make test script executable
chmod +x tests/physical/test_physical_robot.py
```

---

**Remember: When in doubt, use dry-run mode. Safety over speed.**
