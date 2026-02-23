# ROS Community Release Process

This document describes how to release Agent ROS Bridge as an official ROS package.

## Overview

Once released to the ROS community:

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ros-${ROS_DISTRO}-agent-ros-bridge

# Then use:
ros2 run agent_ros_bridge bridge
```

## Supported ROS Distributions

| Distribution | Status | Ubuntu Version | Release Track |
|-------------|--------|----------------|---------------|
| **Jazzy Jalisco** | Target | 24.04 Noble | `jazzy` |
| **Humble Hawksbill** | Target | 22.04 Jammy | `humble` |
| **Iron Irwini** | Target | 22.04 Jammy | `iron` |
| Rolling Ridley | Target | 24.04 Noble | `rolling` |

## Prerequisites

1. **Bloom** — ROS release tool:
   ```bash
   sudo apt install python3-bloom python3-catkin-pkg
   ```

2. **GitHub Access** — Write access to release repository

3. **ROS Distro Fork** — Fork of [ros/rosdistro](https://github.com/ros/rosdistro)

## Release Steps

### 1. Prepare Release

```bash
# Ensure you're on main branch
git checkout main
git pull origin main

# Update version
# Edit: pyproject.toml, package.xml, CHANGELOG.md

# Update CHANGELOG.rst (ROS format)
catkin_generate_changelog

# Commit
git add -A
git commit -m "Prepare release 0.1.0"
```

### 2. Tag Release

```bash
# Create annotated tag
git tag -a v0.1.0 -m "Release version 0.1.0"

# Push tag
git push origin v0.1.0
```

### 3. Bloom Release

```bash
# Release to Jazzy
bloom-release --rosdistro jazzy --track jazzy agent_ros_bridge

# Release to Humble
bloom-release --rosdistro humble --track humble agent_ros_bridge

# Release to Iron
bloom-release --rosdistro iron --track iron agent_ros_bridge

# Release to Rolling
bloom-release --rosdistro rolling --track rolling agent_ros_bridge
```

Each command will:
1. Clone the release repository
2. Create a debian package
3. Open a PR to rosdistro

### 4. Create rosdistro PR

Bloom will output a URL like:
```
https://github.com/ros/rosdistro/compare/master...openclaw:release/jazzy/agent_ros_bridge
```

1. Open the URL
2. Create pull request
3. Fill in the template:
   - Package name: `agent_ros_bridge`
   - Upstream repository: `https://github.com/webthree549-bot/agent-ros-bridge`
   - Release repository: `https://github.com/webthree549-bot/agent_ros_bridge-release`
   - Distro file: `jazzy/distribution.yaml`
   - Bloom version: `0.11.2`

### 5. Wait for Build

After rosdistro PR is merged:

1. Packages are built on [build.ros2.org](http://build.ros2.org/)
2. Testing repository is updated
3. Sync to main repository (monthly)

Track build status:
- http://repo.ros2.org/status_page/ros_jazzy_default.html

### 6. Announce

Once synced:

```markdown
## Agent ROS Bridge 0.1.0 Released

Available via apt:

```bash
sudo apt update
sudo apt install ros-jazzy-agent-ros-bridge
```

Features:
- MCP (Model Context Protocol) support
- WebSocket and gRPC transports
- ROS1 and ROS2 compatibility
- Claude Desktop integration

Full changelog: https://github.com/webthree549-bot/agent-ros-bridge/blob/main/CHANGELOG.md
```

## Package Structure

```
ros-${ROS_DISTRO}-agent-ros-bridge
├── opt/ros/${ROS_DISTRO}/lib/agent_ros_bridge/
│   ├── __init__.py
│   ├── config.py
│   ├── mcp/
│   ├── gateway_v2/
│   └── openclaw.py
├── opt/ros/${ROS_DISTRO}/bin/agent-ros-bridge
├── opt/ros/${ROS_DISTRO}/share/agent_ros_bridge/
│   ├── package.xml
│   └── config/
└── usr/share/doc/ros-${ROS_DISTRO}-agent-ros-bridge/
    ├── changelog.Debian.gz
    └── copyright
```

## Post-Release Updates

### Patch Release

```bash
# Fix bug, update version
git add -A
git commit -m "Fix critical bug"

catkin_generate_changelog
catkin_prepare_release --bump patch  # 0.1.0 -> 0.1.1

bloom-release --rosdistro jazzy --track jazzy agent_ros_bridge
```

### New Distribution

When new ROS distro is released:

```bash
# Create new track
bloom-release --rosdistro new_distro --new-track --track new_distro agent_ros_bridge
```

## Troubleshooting

### Bloom Errors

**"Failed to resolve dependencies"**
- Check `package.xml` dependencies
- Ensure all deps exist in target distro

**"Release repository not found"**
- Create `agent_ros_bridge-release` repo
- Run `bloom-release --edit-track` to update URL

### Build Failures

**ImportError during test**
- Add test dependencies to `package.xml`
- Use `<test_depend>` tags

**Missing Python packages**
- Declare as `<exec_depend>` in `package.xml`
- Or vendor (include) the dependency

## Release Checklist

- [ ] Version updated in `pyproject.toml`
- [ ] Version updated in `package.xml`
- [ ] CHANGELOG.md updated
- [ ] CHANGELOG.rst generated (ROS format)
- [ ] Git tag created (`vX.Y.Z`)
- [ ] GitHub Release created
- [ ] Bloom release for each distro
- [ ] rosdistro PRs created
- [ ] rosdistro PRs merged
- [ ] Build farm successful
- [ ] Package synced to main repo
- [ ] Announcement made

## Resources

- [Bloom Tutorial](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease)
- [ROS Distro Contribution Guide](https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md)
- [REP 3: Target Platforms](https://www.ros.org/reps/rep-0003.html)
- [REP 2000: ROS2 Releases](https://www.ros.org/reps/rep-2000.html)
- [build.ros2.org](http://build.ros2.org/)

## Contact

For release issues:
- [ROS Discourse](https://discourse.ros.org/c/release/16)
- [ROS Release Category](https://discourse.ros.org/c/release/16)
- Maintainer: hello@openclaw.ai
