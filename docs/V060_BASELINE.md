# v0.6.0 Baseline: Starting Point for Engineering Team

## Executive Summary

This document provides the complete baseline of Agent ROS Bridge v0.6.0 — the starting point for all v0.6.1+ development. All engineers must understand this baseline before implementing new features.

---

## 1. v0.6.0 System Overview

### 1.1 What v0.6.0 Does

Agent ROS Bridge v0.6.0 is a **production-ready universal bridge** that connects AI agents to ROS robots:

```
AI Agent (LangChain, AutoGPT, Claude, etc.)
    ↓
[Agent ROS Bridge v0.6.0]
    ├── Transports: WebSocket, MQTT, gRPC
    ├── Protocols: MCP, OpenAI, LangChain
    └── Safety: Basic confirmation flows
    ↓
ROS1/ROS2 Robots (TurtleBot, UR5, drones, etc.)
```

### 1.2 Core Capabilities (v0.6.0)

| Capability | Status | Description |
|------------|--------|-------------|
| **Multi-Transport** | ✅ Complete | WebSocket, MQTT, gRPC with TLS/mTLS |
| **Multi-ROS** | ✅ Complete | ROS1 (rospy) + ROS2 (rclpy) support |
| **Dynamic Messages** | ✅ Complete | 50+ message types, auto-import |
| **Safety Manager** | ✅ Basic | Confirmation flows, emergency stop |
| **Agent Memory** | ✅ Complete | SQLite/Redis with TTL |
| **Tool Discovery** | ✅ Complete | MCP/OpenAI format export |
| **Fleet Support** | ✅ Complete | Multi-robot task management |
| **Metrics** | ✅ Complete | Prometheus/Grafana integration |

### 1.3 Architecture (v0.6.0)

```
agent_ros_bridge/ (v0.6.0)
├── gateway_v2/              # Core bridge
│   ├── core.py              # Bridge class
│   ├── auth.py              # JWT authentication
│   ├── config.py            # Configuration loader
│   ├── connectors/          # ROS1/ROS2 connectors
│   │   ├── ros1_connector.py
│   │   └── ros2_connector.py
│   ├── transports/          # WebSocket, MQTT, gRPC
│   │   ├── websocket.py
│   │   ├── mqtt_transport.py
│   │   └── grpc_transport.py
│   └── plugins/             # Robot plugins
│       └── greenhouse_plugin.py
├── integrations/            # AI framework integrations
│   ├── memory.py            # AgentMemory
│   ├── safety.py            # SafetyManager
│   ├── discovery.py         # ToolDiscovery
│   ├── langchain_adapter.py # ROSBridgeTool
│   ├── mcp_transport.py     # MCP server
│   ├── nl_interpreter.py    # RuleBasedInterpreter
│   └── dashboard_server.py  # Web dashboard
├── fleet/                   # Fleet orchestration
│   └── orchestrator.py
└── actions/                 # ROS Action clients
    └── arm_action_client.py
```

---

## 2. Key Components (What Engineers Must Understand)

### 2.1 Gateway Core (`gateway_v2/core.py`)

**Purpose:** Central bridge managing transports, connectors, and plugins

```python
class Bridge:
    """v0.6.0 Bridge - Starting point for v0.6.1+"""
    
    def __init__(self, config: BridgeConfig):
        self.transports: Dict[str, Transport] = {}
        self.connectors: Dict[str, Connector] = {}
        self.plugins: Dict[str, Plugin] = {}
        self.safety = SafetyManager()
        self.memory = AgentMemory()
    
    async def handle_command(self, cmd: Command) -> Result:
        """Handle incoming command from AI agent."""
        # 1. Safety check
        if not self.safety.validate(cmd):
            return Result.error("Safety check failed")
        
        # 2. Route to appropriate connector
        connector = self.connectors[cmd.robot_id]
        return await connector.execute(cmd)
```

**Key Points:**
- Bridge is transport-agnostic (WebSocket/MQTT/gRPC all use same interface)
- Safety validation happens at bridge level (v0.6.0: basic, v0.6.1+: enhanced)
- Memory persists across sessions

---

### 2.2 ROS2 Connector (`gateway_v2/connectors/ros2_connector.py`)

**Purpose:** Connect to ROS2 robots via rclpy

```python
class ROS2Connector:
    """v0.6.0 ROS2 Connector - Starting point for v0.6.1+"""
    
    def __init__(self, node: rclpy.Node):
        self.node = node
        self.publishers: Dict[str, Publisher] = {}
        self.subscribers: Dict[str, Subscriber] = {}
        self.action_clients: Dict[str, ActionClient] = {}
    
    def cmd_publish(self, topic: str, msg_type: str, payload: dict):
        """Publish message to ROS topic."""
        # Dynamic message type import
        msg_class = self._get_message_class(msg_type)
        msg = msg_class()
        self._dict_to_ros_msg(payload, msg)
        
        publisher = self.publishers.get(topic)
        if not publisher:
            publisher = self.node.create_publisher(msg_class, topic, 10)
            self.publishers[topic] = publisher
        
        publisher.publish(msg)
    
    def cmd_subscribe(self, topic: str, msg_type: str, callback):
        """Subscribe to ROS topic."""
        msg_class = self._get_message_class(msg_type)
        subscriber = self.node.create_subscription(
            msg_class, topic, callback, 10
        )
        self.subscribers[topic] = subscriber
    
    def cmd_action(self, action_name: str, goal_type: str, goal: dict):
        """Execute ROS action."""
        # v0.6.0: Basic action client
        # v0.6.1+: Enhanced with safety certificates
        pass
```

**Key Points:**
- Dynamic message type loading (50+ types supported)
- Publishers/subscribers cached for reuse
- Action clients for long-running tasks

**v0.6.1 Enhancement:** Add safety certificate validation before action execution

---

### 2.3 Safety Manager (`integrations/safety.py`)

**Purpose:** Basic safety confirmation flows (v0.6.0) → Hardware-enforced safety (v0.6.1+)

```python
class SafetyManager:
    """v0.6.0 Safety - Software-only confirmation"""
    
    def __init__(self):
        self.dangerous_actions = [
            "move", "navigate", "pick", "place"
        ]
        self.confirmation_pending: Dict[str, Command] = {}
    
    def validate(self, cmd: Command) -> bool:
        """Check if command requires confirmation."""
        if cmd.action in self.dangerous_actions:
            return self._request_confirmation(cmd)
        return True
    
    def _request_confirmation(self, cmd: Command) -> bool:
        """v0.6.0: Send confirmation request to operator."""
        # Human must approve via dashboard
        self.confirmation_pending[cmd.id] = cmd
        return False  # Defer execution
    
    def confirm(self, cmd_id: str) -> bool:
        """Operator confirms command."""
        if cmd_id in self.confirmation_pending:
            cmd = self.confirmation_pending.pop(cmd_id)
            return True
        return False
```

**Key Points:**
- v0.6.0: Software-only, human-in-the-loop confirmation
- v0.6.1+: Hardware-enforced limits, independent safety validator
- Emergency stop available but software-triggered

**v0.6.1 Enhancement:** Replace with `/safety/validator` node (hardware-enforced, <10ms)

---

### 2.4 Natural Language Interpreter (`integrations/nl_interpreter.py`)

**Purpose:** Rule-based NL parsing (v0.6.0) → ROS-native AI (v0.6.1+)

```python
class RuleBasedInterpreter:
    """v0.6.0 NL Interpreter - Regex-based, deterministic"""
    
    PATTERNS = {
        "navigate": [
            r"go\s+to\s+(?P<location>\w+)",
            r"navigate\s+to\s+(?P<location>\w+)",
            r"move\s+to\s+(?P<location>\w+)"
        ],
        "manipulate": [
            r"pick\s+up\s+(?P<object>\w+)",
            r"place\s+(?P<object>\w+)"
        ]
    }
    
    def interpret(self, utterance: str) -> Optional[Command]:
        """Parse natural language to command."""
        for action, patterns in self.PATTERNS.items():
            for pattern in patterns:
                match = re.search(pattern, utterance, re.IGNORECASE)
                if match:
                    return Command(
                        action=action,
                        params=match.groupdict()
                    )
        return None
```

**Key Points:**
- v0.6.0: Regex patterns, no learning, limited coverage
- v0.6.1+: ROS-native intent parser with rule-based fast path + bounded LLM
- No context awareness in v0.6.0

**v0.6.1 Enhancement:** Replace with `/ai/intent_parser` (rule-based + LLM fallback)

---

### 2.5 Tool Discovery (`integrations/discovery.py`)

**Purpose:** Discover ROS capabilities and export to AI formats

```python
class ToolDiscovery:
    """v0.6.0 Tool Discovery - Static + runtime introspection"""
    
    def __init__(self, robot):
        self.robot = robot
        self.tools: List[Tool] = []
    
    def discover_all(self) -> List[Tool]:
        """Discover all available tools."""
        tools = []
        tools.extend(self._discover_topics())
        tools.extend(self._discover_services())
        tools.extend(self._discover_actions())
        return tools
    
    def _discover_topics(self) -> List[Tool]:
        """Discover publish/subscribe topics."""
        topics = self.robot.get_topic_names_and_types()
        return [self._topic_to_tool(t) for t in topics]
    
    def export_mcp(self) -> dict:
        """Export tools in MCP format."""
        # For Claude Desktop integration
        pass
    
    def export_openai(self) -> List[dict]:
        """Export tools in OpenAI format."""
        # For GPT function calling
        pass
```

**Key Points:**
- Runtime introspection of ROS graph
- Exports to MCP (Claude) and OpenAI formats
- Static configuration fallback

**v0.6.1 Enhancement:** Add semantic enrichment ("camera" = /camera/image_raw + capabilities)

---

### 2.6 Agent Memory (`integrations/memory.py`)

**Purpose:** Persistent storage for agent state

```python
class AgentMemory:
    """v0.6.0 Memory - SQLite/Redis with TTL"""
    
    def __init__(self, backend: str = "sqlite"):
        if backend == "sqlite":
            self.store = SQLiteStore()
        elif backend == "redis":
            self.store = RedisStore()
    
    def remember(self, key: str, value: Any, ttl: int = None):
        """Store value with optional TTL."""
        self.store.set(key, value, ttl)
    
    def recall(self, key: str) -> Optional[Any]:
        """Retrieve value."""
        return self.store.get(key)
    
    def forget(self, key: str):
        """Delete value."""
        self.store.delete(key)
```

**Key Points:**
- SQLite for local, Redis for distributed
- TTL support for automatic expiration
- Used for conversation history, robot state

**v0.6.1 Enhancement:** Add context manager with ROS topology integration

---

## 3. v0.6.0 Test Suite (483+ Tests)

### 3.1 Test Structure

```
tests/
├── unit/                    # 200+ tests
│   ├── test_bridge.py
│   ├── test_ros2_connector.py
│   ├── test_safety.py
│   └── test_memory.py
├── integration/             # 50+ tests
│   ├── test_transports.py
│   └── test_connectors.py
├── e2e/                     # 30+ tests
│   └── test_full_pipeline.py
└── simulation/              # New in v0.6.1
    └── (to be created)
```

### 3.2 Running Tests

```bash
# All tests
pytest tests/ -v

# Unit tests only
pytest tests/unit/ -v

# With coverage
pytest tests/ --cov=agent_ros_bridge --cov-report=html

# Specific test
pytest tests/unit/test_safety.py::TestSafetyManager::test_confirmation
```

### 3.3 Test Coverage (v0.6.0)

| Component | Coverage | Status |
|-----------|----------|--------|
| Bridge core | 92% | ✅ Good |
| ROS2 connector | 88% | ✅ Good |
| Safety manager | 85% | ⚠️ Needs improvement |
| Memory | 90% | ✅ Good |
| Transports | 87% | ✅ Good |
| **Overall** | **89%** | ✅ Good |

**v0.6.1 Goal:** >90% coverage for all new components

---

## 4. v0.6.0 Configuration

### 4.1 Configuration Files

```yaml
# config/robots.yaml (v0.6.0)
robots:
  turtlebot_01:
    type: "turtlebot3_waffle"
    ros_version: "ros2"
    connection:
      transport: "websocket"
      uri: "ws://192.168.1.101:9090"
    capabilities: ["navigate", "sense"]
    topics:
      cmd_vel: "/turtlebot_01/cmd_vel"
      odom: "/turtlebot_01/odom"
      scan: "/turtlebot_01/scan"
    safety:
      max_speed: 0.5
      requires_confirmation: true
```

**v0.6.1 Enhancement:** Auto-discovery reduces manual configuration

### 4.2 Environment Variables

```bash
# Core
BRIDGE_CONFIG_PATH=/etc/agent-ros-bridge/config.yaml
BRIDGE_LOG_LEVEL=INFO

# Safety
BRIDGE_SAFETY_MODE=confirmation  # v0.6.0
# BRIDGE_SAFETY_MODE=hardware    # v0.6.1+

# Memory
BRIDGE_MEMORY_BACKEND=sqlite
BRIDGE_MEMORY_TTL=3600

# Transports
BRIDGE_WEBSOCKET_PORT=8765
BRIDGE_MQTT_BROKER=localhost:1883
```

---

## 5. v0.6.0 → v0.6.1 Transition Guide

### 5.1 What Stays the Same

| Component | v0.6.0 | v0.6.1 | Change |
|-----------|--------|--------|--------|
| **Transports** | WebSocket, MQTT, gRPC | Same | ✅ No change |
| **Connectors** | ROS1, ROS2 | Same | ✅ No change |
| **Memory** | SQLite/Redis | Same | ✅ No change |
| **Fleet** | Basic orchestration | Enhanced | 🔄 Extend |
| **Tests** | 483+ | 483+ + new | 🔄 Add |

### 5.2 What Changes

| Component | v0.6.0 | v0.6.1 | Change |
|-----------|--------|--------|--------|
| **Safety** | Software confirmation | Hardware-enforced | 🔴 Replace |
| **NL Parser** | Rule-based only | Rule + LLM | 🔴 Replace |
| **Context** | None | ROS topology | 🟡 Add |
| **Planning** | Direct ROS calls | Verified + certified | 🟡 Add |
| **Simulation** | None | Gazebo + CI/CD | 🟡 Add |

### 5.3 Migration Strategy

```
v0.6.0 System (Running)
    ↓
v0.6.1 Components (Parallel Development)
    ├── /ai/intent_parser (NEW)
    ├── /ai/context_manager (NEW)
    ├── /safety/validator (NEW - replaces old)
    └── /ai/motion_planner (NEW)
    ↓
Integration Testing (Both systems)
    ↓
Gradual Cutover (Feature flags)
    ↓
v0.6.1 Full Deployment
```

---

## 6. Key Files for Engineers

### 6.1 Must Read

| File | Purpose | Why Important |
|------|---------|---------------|
| `gateway_v2/core.py` | Bridge core | Central orchestration |
| `gateway_v2/connectors/ros2_connector.py` | ROS interface | All robot communication |
| `integrations/safety.py` | Safety (v0.6.0) | To be replaced in v0.6.1 |
| `integrations/nl_interpreter.py` | NL parsing (v0.6.0) | To be replaced in v0.6.1 |
| `integrations/discovery.py` | Tool discovery | Extended in v0.6.1 |
| `tests/unit/test_*.py` | Unit tests | Understand testing patterns |

### 6.2 Must Understand

```python
# Key interfaces that v0.6.1 components must implement

# Transport interface (unchanged)
class Transport(ABC):
    @abstractmethod
    async def send(self, message: str) -> None: ...
    @abstractmethod
    async def receive(self) -> str: ...

# Connector interface (unchanged)
class Connector(ABC):
    @abstractmethod
    async def execute(self, cmd: Command) -> Result: ...
    @abstractmethod
    def get_capabilities(self) -> List[Capability]: ...

# Safety interface (v0.6.1: enhanced)
class SafetyValidator(ABC):
    @abstractmethod
    def validate(self, plan: MotionPlan) -> SafetyResult: ...
```

---

## 7. Development Environment Setup

### 7.1 v0.6.0 Development Setup

```bash
# 1. Clone repository
git clone https://github.com/webthree549-bot/agent-ros-bridge.git
cd agent-ros-bridge

# 2. Install dependencies
pip install -e .
pip install -r requirements-dev.txt

# 3. Setup ROS2 (if testing with real robots)
source /opt/ros/humble/setup.bash

# 4. Run tests
pytest tests/ -v

# 5. Start bridge (development mode)
agent-ros-bridge --config config/dev.yaml --verbose
```

### 7.2 v0.6.1 Development Setup (New)

```bash
# Additional setup for v0.6.1

# 1. Install Gazebo Ignition
sudo apt install ignition-fortress

# 2. Setup simulation environment
pip install -r requirements-sim.txt

# 3. Build robot models
cd simulation/models
make all

# 4. Run simulation tests
pytest tests/simulation/ -v
```

---

## 8. Common Patterns (v0.6.0 → v0.6.1)

### 8.1 ROS Node Pattern

```python
# v0.6.0 Pattern (to be followed in v0.6.1)

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Initialize publishers, subscribers, services
        
    def my_callback(self, msg):
        # Handle message
        pass

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 8.2 Service Pattern

```python
# v0.6.0 Pattern (to be followed in v0.6.1)

from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Result: {response.sum}')
        return response
```

### 8.3 Testing Pattern

```python
# v0.6.0 Pattern (to be followed in v0.6.1)

import pytest
from agent_ros_bridge.my_module import MyClass

class TestMyClass:
    def setup_method(self):
        self.obj = MyClass()
    
    def test_my_feature(self):
        result = self.obj.do_something()
        assert result == expected
    
    def test_edge_case(self):
        with pytest.raises(ValueError):
            self.obj.do_something(invalid_input)
```

---

## 9. Documentation References

### 9.1 v0.6.0 Documentation

| Document | Location | Purpose |
|----------|----------|---------|
| README.md | Root | Overview, quick start |
| CHANGELOG.md | Root | Version history |
| ROADMAP.md | Root | Future plans |
| docs/api/ | docs/ | API documentation |

### 9.2 v0.6.1+ Architecture (Must Read)

| Document | Purpose |
|----------|---------|
| `docs/TODO.md` | Unified roadmap |
| `docs/NL2ROS_DEEP_ANALYSIS.md` | NL architecture |
| `docs/ROS_NATIVE_AI_ARCHITECTURE.md` | ROS-native design |
| `docs/SIMULATION_FIRST_STRATEGY.md` | Simulation approach |
| `docs/V061_SPRINT_PLAN.md` | Sprint plan |

---

## 10. Quick Reference

### 10.1 v0.6.0 Commands

```bash
# Start bridge
agent-ros-bridge --config config.yaml

# Run with specific transport
agent-ros-bridge --transport websocket --port 8765

# Enable debug logging
agent-ros-bridge --log-level DEBUG

# Run tests
pytest tests/ -v

# Run specific test
pytest tests/unit/test_safety.py -v

# Check version
agent-ros-bridge --version  # 0.6.0
```

### 10.2 Key Metrics (v0.6.0)

| Metric | Value | Notes |
|--------|-------|-------|
| **Lines of Code** | ~15,000 | Python + C++ |
| **Test Count** | 483+ | Unit + integration |
| **Test Coverage** | 89% | Overall |
| **Documentation** | 30+ pages | API + guides |
| **Dependencies** | 25 | Python packages |

---

## Summary

**v0.6.0 is a production-ready foundation with:**
- ✅ Multi-transport support (WebSocket/MQTT/gRPC)
- ✅ Multi-ROS support (ROS1/ROS2)
- ✅ Basic safety (software confirmation)
- ✅ 483+ tests (89% coverage)
- ✅ AI framework integrations

**v0.6.1 builds on this by adding:**
- 🔴 ROS-native AI components (replace nl_interpreter)
- 🔴 Hardware-enforced safety (replace safety.py)
- 🟡 Context awareness (new)
- 🟡 Motion planning with verification (new)
- 🟡 Simulation-first development (new)

**Engineers must:**
1. Understand v0.6.0 architecture (this document)
2. Read v0.6.1 architecture docs (referenced above)
3. Follow v0.6.1 sprint plan (V061_SPRINT_PLAN.md)
4. Maintain backward compatibility where possible

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Engineering Baseline Complete
