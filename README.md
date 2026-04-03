# Agent ROS Bridge 🔒

<p align="center">
  <img src="https://img.shields.io/badge/version-0.6.5-blue.svg" alt="Version">
  <img src="https://img.shields.io/badge/python-3.11%20%7C%203.12%20%7C%203.13%20%7C%203.14-blue.svg" alt="Python">
  <img src="https://img.shields.io/badge/tests-2,021%2B-passing.svg" alt="Tests">
  <img src="https://img.shields.io/badge/coverage-65%25-yellow.svg" alt="Coverage">
  <img src="https://img.shields.io/badge/safety-validated-success.svg" alt="Safety">
  <img src="https://img.shields.io/badge/Gate%202-PASSED-brightgreen.svg" alt="Gate 2">
  <img src="https://img.shields.io/badge/license-MIT-green.svg" alt="License">
</p>

<p align="center">
  <b>The Safety-First Production Gateway for AI-to-Robot Integration</b><br>
  <b>When robots matter, safety comes first.</b>
</p>

<p align="center">
  <a href="#why-agent-ros-bridge">Why Us?</a> •
  <a href="#quick-start">Quick Start</a> •
  <a href="#safety-first">Safety First</a> •
  <a href="#comparison">Comparison</a> •
  <a href="#documentation">Docs</a>
</p>

---

## Why Agent ROS Bridge?

Unlike diagnostic tools ([NASA ROSA](https://github.com/nasa-jpl/rosa)) or research platforms ([ROS-LLM](https://github.com/Auromix/ROS-LLM)), **Agent ROS Bridge is the only production-ready gateway with built-in safety validation.**

### The Safety Problem

Deploying LLM-controlled robots in production is dangerous:
- AI hallucinations can damage equipment
- Wrong commands can injure humans  
- No validation of AI decisions
- No learning from operator corrections

### Our Solution

```yaml
🛡️ Safety-First Architecture:
├── Human-in-the-Loop (enforced by default)
├── Shadow Mode Validation (200+ hours required)
├── Simulation Testing (10K scenarios, 95.93% success)
├── Gradual Rollout (0% → 100% autonomy)
└── Emergency Stop (always available)
```

### Comparison

| Feature | Agent ROS Bridge | NASA ROSA | ROS-LLM |
|---------|-----------------|-----------|---------|
| **Safety Validation** | ✅ Shadow mode | ❌ None | ❌ None |
| **Human-in-the-Loop** | ✅ Enforced | ⚠️ Optional | ❌ No |
| **Production Tests** | ✅ 2,021 tests | ❓ Unknown | ❓ Unknown |
| **Simulation** | ✅ 10K scenarios | ⚠️ Basic | ❌ No |
| **Multi-Protocol** | ✅ 4 protocols | ❌ CLI only | ❌ ROS2 only |
| **Fleet Support** | ✅ Multi-robot | ❌ Single | ❌ Single |
| **Published Research** | 📝 Whitepaper in progress | ✅ arXiv | ✅ Nature |

**[Full Comparison →](docs/COMPARISON.md)**

---

## Quick Start

### Installation

```bash
pip install agent-ros-bridge
```

### Basic Usage

```python
from agent_ros_bridge import RobotAgent

# Create agent with SAFETY enforced
agent = RobotAgent(
    device_id='bot1',
    llm_provider='moonshot',
    require_confirmation=True,  # Human approval required
)

# AI proposes, human approves, robot executes
result = agent.execute("Go to the kitchen")

print(f"Success: {result.success}")
print(f"AI confidence: {result.ai_confidence:.2f}")
print(f"Human approvals: {result.human_approvals}")
```

**Output:**
```
============================================================
🛡️  SAFETY STATUS
============================================================
Device: bot1 (mobile_robot)
Autonomous Mode: False ✅
Human-in-the-Loop: True ✅
Shadow Mode: True ✅
Validation Status: simulation_only
============================================================

🤖 AI Proposal: navigate_to(kitchen)
👤 Human: Approve? (y/n): y
✅ Executed successfully
```

---

## Safety First

### Safe-by-Default Configuration

```yaml
# config/global_config.yaml
safety:
  autonomous_mode: false              # Human approval required
  human_in_the_loop: true             # All AI proposals need approval
  shadow_mode_enabled: true           # Collect validation data
  min_confidence_for_auto: 0.95       # High confidence threshold
  gradual_rollout_stage: 0            # Start at 0% autonomy
  safety_validation_status: "simulation_only"
  required_shadow_hours: 200.0        # Target for validation
  min_agreement_rate: 0.95            # Required agreement %
```

### Deployment Stages

```
Stage 0: Simulation-Only (Current)
├── ✅ 10K scenarios tested
├── ✅ 95.93% success rate
└── ⚠️ No real-world validation yet

Stage 1: Supervised Operation
├── Human approves all actions
├── Shadow mode collects data
└── Target: 200+ hours, >95% agreement

Stage 2: Gradual Rollout
├── 10% → 25% → 50% → 75% → 100%
├── High confidence only
└── Monitor at each stage

Stage 3: Full Autonomy
├── After validation complete
├── Emergency stop always available
└── Continuous monitoring
```

**[Safety Documentation →](docs/SAFETY.md)**

---

## Architecture

```
AI Agents ─┬─ WebSocket ─┐
           ├─ gRPC ──────┼──► ┌─────────────┐
           ├─ MQTT ──────┤    │   Gateway   │
           └─ TCP ───────┘    └──────┬──────┘
                                     │
                    ┌────────────────┼────────────────┐
                    ▼                ▼                ▼
            ┌──────────────┐  ┌──────────────┐  ┌──────────┐
            │Safety Layer  │  │ Shadow Mode  │  │  ROS1/   │
            │(Validation)  │  │ (Data Coll.) │  │  ROS2    │
            └──────────────┘  └──────────────┘  └──────────┘
```

### Key Components

| Component | Purpose | Status |
|-----------|---------|--------|
| **Gateway** | Multi-protocol support | ✅ Stable |
| **Safety Layer** | Validation & enforcement | ✅ Implemented |
| **Shadow Mode** | AI-human decision logging | ✅ Active |
| **Simulation** | 10K scenario testing | ✅ Complete |
| **Fleet** | Multi-robot orchestration | ✅ Beta |

---

## Features

### 🛡️ Safety & Validation
- **Shadow Mode** - Log AI proposals vs human decisions
- **Human-in-the-Loop** - Enforced by default
- **Simulation Testing** - 10K scenarios before deployment
- **Gradual Rollout** - Increase autonomy slowly
- **Emergency Stop** - Always available

### 🌐 Multi-Protocol
- **WebSocket** - Real-time bidirectional
- **gRPC** - High-performance RPC
- **MQTT** - IoT messaging
- **TCP** - Raw socket support

### 🤖 Universal Support
- **Mobile Robots** - Navigation, mapping
- **Drones** - Flight control, aerial missions
- **Robot Arms** - Manipulation, grasping
- **Humanoids** - Walking, balancing
- **Sensors** - Data collection, monitoring

### 🧠 AI Integration
- **Multi-LLM** - OpenAI, Moonshot, Anthropic
- **Intent Parsing** - Natural language understanding
- **Context Awareness** - Scene understanding
- **Multi-Language** - English, Chinese, +4 more

---

## Documentation

- **[Safety Guide](docs/SAFETY.md)** - Deployment safety requirements
- **[Architecture](docs/ARCHITECTURE.md)** - System design
- **[API Reference](docs/API_REFERENCE.md)** - Complete API docs
- **[Comparison](docs/COMPARISON.md)** - vs NASA ROSA vs ROS-LLM
- **[Changelog](CHANGELOG.md)** - Version history

---

## Web Dashboard

Agent ROS Bridge includes a modern web dashboard for robot control and monitoring.

### Features

- **🎮 Robot Control** - D-pad interface, natural language commands
- **📊 Real-time Telemetry** - Position, velocity, battery, sensors
- **🧠 Shadow Mode Metrics** - AI-human agreement tracking
- **🛡️ Safety Status** - Validation gates, deployment readiness
- **🚁 Fleet Management** - Multi-robot coordination

### Quick Start

#### Option 1: Docker (Recommended)
```bash
# Start bridge + web dashboard
docker-compose --profile web up -d

# Access dashboard
open http://localhost:8081
```

#### Option 2: Host (Development)
```bash
# Start bridge
agent-ros-bridge --websocket-port 8765

# In another terminal, serve dashboard
cd agent_ros_bridge/web
python3 -m http.server 8081

# Access dashboard
open http://localhost:8081
```

### Dashboard URLs

| Dashboard | URL | Purpose |
|-----------|-----|---------|
| **Control Dashboard** | http://localhost:8081 | Robot control + shadow metrics |
| **3D Visualization** | http://localhost:8080 | Gazebo/3D view (in ros2_jazzy) |
| **Grafana** | http://localhost:3000 | System monitoring |

### WebSocket Connection

The dashboard connects to the bridge via WebSocket:
- **URL**: `ws://localhost:8765`
- **Protocol**: JSON messages
- **Auth**: JWT token (if enabled)

[Full Dashboard Guide →](agent_ros_bridge/web/README.md)

---

## Installation

### Prerequisites
- Python 3.11+
- ROS1 (Noetic) or ROS2 (Humble/Jazzy)
- Docker (optional, for simulation)

### PyPI Install
```bash
pip install agent-ros-bridge
```

### Docker Install
```bash
docker pull agent-ros-bridge:jazzy-with-nav2
docker run -it agent-ros-bridge:jazzy-with-nav2
```

### Development Install
```bash
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge
pip install -e ".[dev]"
```

---

## Testing

```bash
# Run all tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=agent_ros_bridge --cov-report=html

# Run safety-critical tests only
pytest tests/unit/safety/ -v

# Run simulation tests (requires Docker)
pytest tests/e2e/ -v
```

**Current Status:**
- 2,021 tests passing
- 65% code coverage
- Gate 2 validation: PASSED (95.93% success)

---

## Safety Certification Roadmap

| Milestone | Target | Status |
|-----------|--------|--------|
| Shadow Mode Data | 200 hours | 🟡 In Progress (0 hrs) |
| Agreement Rate | >95% | 🟡 In Progress (0%) |
| Simulation Validation | 10K scenarios | ✅ PASSED (95.93%) |
| ISO 10218 Review | Q3 2026 | ⏳ Planned |
| Insurance Review | Q4 2026 | ⏳ Planned |

---

## Community

- **ROS Discourse:** [Announcement Thread](https://discourse.ros.org/t/agent-ros-bridge/52604)
- **GitHub Issues:** [Report Issues](https://github.com/webthree549-bot/agent-ros-bridge/issues)
- **Documentation:** [Full Docs](https://agent-ros-bridge.readthedocs.io)

---

## Citation

If you use Agent ROS Bridge in research, please cite:

```bibtex
@software{agent_ros_bridge,
  title = {Agent ROS Bridge: Safety-First Production Gateway for AI-to-Robot Integration},
  author = {Agent ROS Bridge Contributors},
  year = {2026},
  url = {https://github.com/webthree549-bot/agent-ros-bridge}
}
```

---

## License

MIT License - See [LICENSE](LICENSE) for details.

---

<p align="center">
  <b>Built with safety in mind for production robotics deployments.</b><br>
  <sub>When robots matter, safety comes first.</sub>
</p>
