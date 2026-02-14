# Agent ROS Bridge - Build, Test & Clean Workflow

**Date:** February 13, 2026  
**Version:** 2.0.0

---

## Quick Reference

```bash
# One command - full test
./build-and-test.sh all

# Clean everything when done
./build-and-test.sh clean
```

---

## Directory Structure

### Source Code (Git Tracked)
```
~/dev/agent-ros-bridge/          ← PURE SOURCE (never install here)
├── agent_ros_bridge/             # Python package
├── demo/                         # Demos
├── tests/                        # Tests
├── docs/                         # Documentation
├── config/                       # Config templates
├── docker/                       # Docker files
├── scripts/                      # Scripts
├── .github/                      # CI/CD
├── SKILL.md                      # ClawHub manifest
├── README.md
├── pyproject.toml
├── Makefile
└── build-and-test.sh             # ← This workflow script
```

### Build Artifacts (Outside Source)
```
~/agent-ros-bridge-test-venv/     # Virtual environment
~/agent-ros-bridge-build-artifacts/
├── dist/                         # Built packages (.whl, .tar.gz)
└── ...
```

---

## Option 1: Automated Workflow (Recommended)

### Step 1: Build & Test

```bash
cd ~/dev/agent-ros-bridge
./build-and-test.sh all
```

**What it does:**
1. Creates virtual environment at `~/agent-ros-bridge-test-venv/`
2. Installs dependencies
3. Runs tests from source (`pytest tests/`)
4. Builds package (wheel + sdist)
5. Installs built package
6. Verifies CLI works

**Output:**
- Built packages in `~/agent-ros-bridge-build-artifacts/dist/`
- Verified working installation

### Step 2: Clean Everything

```bash
./build-and-test.sh clean
```

**What it removes:**
- Virtual environment (`~/agent-ros-bridge-test-venv/`)
- Build artifacts (`~/agent-ros-bridge-build-artifacts/`)
- Source directory build files (`make clean`)
- Pip cache

**Result:** Fresh source directory, ready to start over

---

## Option 2: Manual Workflow

### Step 1: Setup Virtual Environment (Outside Source)

```bash
# Create OUTSIDE source directory
cd ~
python3 -m venv agent-ros-bridge-test-venv
source agent-ros-bridge-test-venv/bin/activate
```

### Step 2: Install Dependencies

```bash
pip install --upgrade pip
pip install build twine pytest pytest-asyncio websockets
```

### Step 3: Test from Source

```bash
cd ~/dev/agent-ros-bridge
pip install -e ".[dev]"
pytest tests/ -v
```

### Step 4: Build Package

```bash
# Build to OUTSIDE directory
mkdir -p ~/agent-ros-bridge-build-artifacts
cd ~/dev/agent-ros-bridge
python -m build --outdir ~/agent-ros-bridge-build-artifacts/dist
```

**Check output:**
```bash
ls -lh ~/agent-ros-bridge-build-artifacts/dist/
# Should show:
# agent_ros_bridge-2.0.0-py3-none-any.whl
# agent-ros-bridge-2.0.0.tar.gz
```

### Step 5: Test Installed Package

```bash
# Fresh virtual environment
deactivate
rm -rf ~/agent-ros-bridge-test-venv
cd ~
python3 -m venv agent-ros-bridge-test-venv
source agent-ros-bridge-test-venv/bin/activate

# Install built package (NOT from source)
pip install ~/agent-ros-bridge-build-artifacts/dist/*.whl

# Test it works
python -c "from agent_ros_bridge import Bridge; print('OK')"
agent-ros-bridge --version
```

### Step 6: Clean for Fresh Start

```bash
# Clean source directory
cd ~/dev/agent-ros-bridge
make clean
find . -name ".DS_Store" -delete

# Remove build artifacts
rm -rf ~/agent-ros-bridge-test-venv
rm -rf ~/agent-ros-bridge-build-artifacts

# Clean pip cache
pip cache purge
```

**Verify clean:**
```bash
# Should show nothing
ls ~/agent-ros-bridge-test-venv 2>/dev/null || echo "✓ Venv removed"
ls ~/agent-ros-bridge-build-artifacts 2>/dev/null || echo "✓ Build artifacts removed"

# Source should be clean
cd ~/dev/agent-ros-bridge
git status  # Should show no untracked build files
```

---

## Verification Checklist

### Before Build
- [ ] Source directory has no `build/`, `dist/`, `*.egg-info/`
- [ ] Virtual environment will be created OUTSIDE source
- [ ] Build artifacts will go OUTSIDE source

### During Build
- [ ] Tests pass (`pytest tests/`)
- [ ] Package builds successfully (`python -m build`)
- [ ] Wheel file created in `~/agent-ros-bridge-build-artifacts/dist/`

### After Build
- [ ] Installed package imports (`from agent_ros_bridge import Bridge`)
- [ ] CLI works (`agent-ros-bridge --version`)
- [ ] Source directory still clean

### After Clean
- [ ] Virtual environment removed
- [ ] Build artifacts removed
- [ ] Source directory has no new untracked files
- [ ] Ready for fresh start

---

## Common Commands Reference

### Build & Test Script

```bash
./build-and-test.sh all      # Full workflow
./build-and-test.sh test     # Run tests only
./build-and-test.sh build    # Build package only
./build-and-test.sh install  # Test installed package
./build-and-test.sh clean    # Clean everything
```

### Manual Commands

```bash
# Create venv outside source
cd ~ && python3 -m venv agent-ros-bridge-test-venv

# Activate
source ~/agent-ros-bridge-test-venv/bin/activate

# Test from source
cd ~/dev/agent-ros-bridge
pip install -e ".[dev]"
pytest tests/

# Build
python -m build --outdir ~/agent-ros-bridge-build-artifacts/dist

# Clean
make clean
rm -rf ~/agent-ros-bridge-test-venv ~/agent-ros-bridge-build-artifacts
```

---

## Production Installation

When ready for actual use (not testing):

```bash
# Install from PyPI (when published)
pip install agent-ros-bridge

# Or install from built wheel
pip install ~/agent-ros-bridge-build-artifacts/dist/agent_ros_bridge-2.0.0-py3-none-any.whl

# Or use install.sh
./install.sh
```

---

## Summary

| Phase | Location | Command |
|-------|----------|---------|
| **Development** | `~/dev/agent-ros-bridge/` | Edit source |
| **Testing** | `~/agent-ros-bridge-test-venv/` | `./build-and-test.sh all` |
| **Build** | `~/agent-ros-bridge-build-artifacts/dist/` | `python -m build` |
| **Clean** | Remove artifacts | `./build-and-test.sh clean` |
| **Production** | System locations | `pip install agent-ros-bridge` |

---

**Key Principle:** Source directory stays PURE - all artifacts live OUTSIDE.
