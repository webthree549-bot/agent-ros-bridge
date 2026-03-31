# Agent ROS Bridge Examples

**Version:** v0.6.5  
**Organization:** Safety-First Production Gateway  
**Last Updated:** 2026-03-30

---

## Directory Structure

Examples are organized by use case and deployment stage:

```
examples/
├── safety/              # Safety-critical deployments
│   ├── healthcare_assistant.py      # Zero-tolerance safety
│   └── demo_shadow_mode.py          # Shadow mode validation
│
├── production/          # Production warehouse/factory deployments
│   └── warehouse_automation.py      # Fleet coordination & gradual rollout
│
├── protocols/           # Multi-protocol communication
│   ├── multiprotocol_iot_fleet.py   # WebSocket, gRPC, MQTT, TCP
│   └── grpc_example.py              # High-performance RPC
│
├── integrations/        # Third-party integrations
│   ├── mcp_integration.py           # Model Context Protocol
│   ├── ros1_real_bridge.py          # ROS1 connectivity
│   └── ros2_real_bridge.py          # ROS2 connectivity
│
├── auth/                # Authentication & security
│   └── jwt_client_example.py        # JWT token handling
│
├── quickstart/          # Getting started tutorials
│   ├── turtlebot3_bridge_demo.py    # First robot setup
│   ├── demo_llm_moonshot.py         # LLM integration
│   └── README.md                    # Quick start guide
│
├── actions/             # Action system examples
├── arm/                 # Robot arm manipulation
├── fleet/               # Fleet management basics
├── metrics/             # Metrics and monitoring
├── mqtt_iot/            # MQTT IoT messaging
├── playground/          # Experimental code
├── tutorials/           # Step-by-step tutorials
│
└── legacy/              # Pre-v0.6.0 examples (for reference)
    ├── agentic_vs_traditional.py
    ├── blueprint_example.py
    ├── full_demo.py
    └── full_demo_performance.py
```

---

## Quick Start by Use Case

### 🏥 Healthcare (Safety-Critical)
**Priority:** ZERO safety violations tolerated

```bash
# Run healthcare assistant example
python examples/safety/healthcare_assistant.py

# Run tests
pytest tests/examples/safety/test_healthcare_assistant.py -v
```

**Key Features:**
- Human-in-the-loop enforced (always)
- 500+ hours shadow mode validation
- 99% agreement threshold
- 2-nurse consensus for high-risk

---

### 🏭 Warehouse Automation (Production)
**Priority:** Gradual rollout with safety validation

```bash
# Run warehouse automation
python examples/production/warehouse_automation.py

# Run tests
pytest tests/examples/production/test_warehouse_automation.py -v
```

**Key Features:**
- 200+ hours shadow mode
- Gradual autonomy (0% → 100%)
- Multi-robot fleet coordination
- Production monitoring

---

### 🌐 Multi-Protocol IoT Fleet (Connectivity)
**Priority:** Optimal protocol per use case

```bash
# Run multi-protocol example
python examples/protocols/multiprotocol_iot_fleet.py

# Run tests
pytest tests/examples/protocols/test_multiprotocol_iot_fleet.py -v
```

**Key Features:**
- 4 protocols: WebSocket, gRPC, MQTT, TCP
- Heterogeneous robot coordination
- Protocol-agnostic commands
- Real-time performance

---

## Example Categories

### 1. Safety Examples (`safety/`)

For deployments where safety is paramount:
- **Healthcare** - Patient care with zero tolerance
- **Shadow Mode** - Data collection for validation

**Deployment Stage:** Stage 0 (human approval required)

### 2. Production Examples (`production/`)

For commercial deployments:
- **Warehouse** - Fulfillment centers, logistics

**Deployment Stage:** Stage 0 → Stage 3 (gradual rollout)

### 3. Protocol Examples (`protocols/`)

For multi-protocol deployments:
- **IoT Fleet** - Mixed robot types, optimal protocols
- **gRPC** - High-performance RPC

**Use When:** Multiple robot types with different needs

### 4. Integration Examples (`integrations/`)

For connecting to existing systems:
- **MCP** - Model Context Protocol integration
- **ROS1/ROS2** - Robot Operating System bridges

### 5. Quick Start (`quickstart/`)

For learning and first-time users:
- **TurtleBot3** - First robot setup
- **LLM Demo** - Language model integration

---

## Running Examples

### Prerequisites

```bash
pip install agent-ros-bridge
```

### Run All Examples

```bash
# Safety examples
python examples/safety/healthcare_assistant.py
python examples/safety/demo_shadow_mode.py

# Production examples
python examples/production/warehouse_automation.py

# Protocol examples
python examples/protocols/multiprotocol_iot_fleet.py
python examples/protocols/grpc_example.py

# Integration examples
python examples/integrations/mcp_integration.py
```

### Run Tests

```bash
# All example tests
pytest tests/examples/ -v

# Safety tests only
pytest tests/examples/safety/ -v

# Production tests only
pytest tests/examples/production/ -v

# Protocol tests only
pytest tests/examples/protocols/ -v
```

---

## TDD Approach

All new examples follow Test-Driven Development:

1. **Write Tests First** (`tests/examples/<category>/test_*.py`)
2. **Implement Solution** (`examples/<category>/*.py`)
3. **Verify Tests Pass**
4. **Refactor**

### Test Organization

```
tests/examples/
├── safety/
│   └── test_healthcare_assistant.py      # 19 tests
├── production/
│   └── test_warehouse_automation.py      # 17 tests
└── protocols/
    └── test_multiprotocol_iot_fleet.py   # 15 tests
```

**Total:** 51 tests across 3 example categories

---

## Safety-First Deployment Stages

### Stage 0: Simulation-Only (All Examples Start Here)
- ✅ Human approval required for ALL actions
- ✅ Shadow mode enabled
- ✅ No autonomous execution

**Files:** All examples default to Stage 0

### Stage 1: Supervised Operation
- ✅ Collect 200+ hours shadow data
- ✅ Track AI-human agreement
- ✅ Still 100% human approval

**Target:** >95% agreement rate

### Stage 2: Gradual Rollout
- ✅ 10% → 25% → 50% → 75% → 100% autonomy
- ✅ High confidence actions first
- ✅ Continuous monitoring

### Stage 3: Full Autonomy (After Validation)
- ✅ Post 200+ hours + >95% agreement
- ✅ Emergency stop always available
- ✅ Shadow mode still active

---

## Key Features by Example

### Healthcare Assistant (`safety/`)
- ✅ ZERO autonomous mode (never enabled)
- ✅ 500+ hours validation required
- ✅ 99% agreement threshold
- ✅ 2-nurse consensus
- ✅ HIPAA/FDA compliance ready

### Warehouse Automation (`production/`)
- ✅ Shadow mode (200+ hours)
- ✅ Gradual rollout support
- ✅ Fleet coordination
- ✅ Production monitoring

### Multi-Protocol IoT (`protocols/`)
- ✅ 4 protocols: WebSocket, gRPC, MQTT, TCP
- ✅ Protocol-agnostic commands
- ✅ Heterogeneous robot support
- ✅ Real-time optimization

---

## Migration from Old Structure

If you were using examples from the old flat structure:

| Old Location | New Location |
|--------------|--------------|
| `examples/warehouse_automation.py` | `examples/production/warehouse_automation.py` |
| `examples/healthcare_assistant.py` | `examples/safety/healthcare_assistant.py` |
| `examples/multiprotocol_iot_fleet.py` | `examples/protocols/multiprotocol_iot_fleet.py` |
| `examples/full_demo.py` | `examples/legacy/full_demo.py` |
| `examples/grpc_example.py` | `examples/protocols/grpc_example.py` |

**Note:** Legacy examples still work but are not actively maintained.

---

## Contributing New Examples

When adding new examples:

1. **Choose the right category** based on use case
2. **Follow TDD** - Write tests first
3. **Document** the example with docstrings
4. **Update this README** with the new example
5. **Test** with `pytest tests/examples/<category>/`

### Example Template

```python
"""
Example: [Brief Description]

This demonstrates [key features].

Use Case: [When to use this]
Safety Level: [Stage 0/1/2/3]

Run:
    python examples/<category>/example_name.py
    
Test:
    pytest tests/examples/<category>/test_example_name.py -v
"""

from agent_ros_bridge import RobotAgent

# Implementation here
```

---

## Documentation

- **Examples Guide:** `docs/EXAMPLES_TDD.md`
- **Safety Guide:** `docs/SAFETY.md`
- **API Reference:** `docs/API_REFERENCE.md`
- **Comparison:** `docs/COMPARISON.md`

---

## Support

- **Issues:** GitHub Issues
- **Discussions:** ROS Discourse
- **Documentation:** https://agent-ros-bridge.readthedocs.io

---

*Examples organized for safety-first production deployments.*  
*Version: v0.6.5*  
*When robots matter, safety comes first.*
