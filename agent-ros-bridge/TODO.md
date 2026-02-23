# TODO — Agent ROS Bridge

## 🔴 CRITICAL — Block Public Release

### Testing & Quality

- [ ] **#1 Create test directory structure**
  ```
  tests/
  ├── __init__.py
  ├── conftest.py          # pytest fixtures
  ├── unit/
  │   ├── test_action_registry.py
  │   ├── test_transport_manager.py
  │   ├── test_connector_manager.py
  │   └── test_topic_manager.py
  ├── integration/
  │   ├── test_websocket_transport.py
  │   ├── test_grpc_transport.py
  │   └── test_mcp_server.py
  └── e2e/
      └── test_full_pipeline.py
  ```

- [ ] **#2 Write unit tests for ROSBridge core** (8 hours)
  - Action registry: register, get, list, execute
  - Transport manager: register, start, stop
  - Connector manager: register, lifecycle
  - Topic manager: subscribe, publish, cache
  - Session management: create, get, close

- [ ] **#3 Write integration tests** (8 hours)
  - WebSocket transport: connect, send, receive, auth
  - gRPC transport: server start/stop, TLS
  - ROS2 connector (with mocked rclpy)
  - Full pipeline: agent → transport → bridge → connector

- [ ] **#4 Set up pytest + coverage** (4 hours)
  - pytest.ini configuration
  - pytest-asyncio for async tests
  - pytest-cov for coverage reporting
  - Target: 80%+ coverage

- [ ] **#5 GitHub Actions CI** (4 hours)
  ```yaml
  # .github/workflows/ci.yml
  name: CI
  on: [push, pull_request]
  jobs:
    test:
      runs-on: ubuntu-latest
      strategy:
        matrix:
          python-version: ['3.8', '3.9', '3.10', '3.11', '3.12']
      steps:
        - uses: actions/checkout@v3
        - uses: actions/setup-python@v4
        - run: pip install -e ".[all,dev]"
        - run: pytest --cov=agent_ros_bridge --cov-report=xml
        - uses: codecov/codecov-action@v3
  ```

- [ ] **#6 Pre-commit hooks** (2 hours)
  - black (formatting)
  - ruff (linting)
  - mypy (type checking)
  - pytest (test on commit)

### Docker & Deployment

- [ ] **#7 Base Dockerfile** (4 hours)
  ```dockerfile
  FROM python:3.11-slim
  WORKDIR /app
  COPY pyproject.toml .
  RUN pip install -e ".[all]"
  COPY . .
  EXPOSE 8765 50051
  CMD ["python", "-m", "agent_ros_bridge.mcp"]
  ```

- [ ] **#8 Multi-arch Docker build** (4 hours)
  - linux/amd64 (servers)
  - linux/arm64 (Apple Silicon, Jetson)
  - GitHub Actions buildx for multi-arch

- [ ] **#9 Docker Compose example** (2 hours)
  ```yaml
  # docker-compose.yml
  version: '3'
  services:
    bridge:
      image: ghcr.io/openclaw/agent-ros-bridge:latest
      ports:
        - "8765:8765"
        - "50051:50051"
      environment:
        - JWT_SECRET=${JWT_SECRET}
      volumes:
        - ./bridge.yaml:/etc/bridge.yaml
  ```

### Documentation

- [ ] **#10 Architecture Decision Records (ADRs)** (4 hours)
  - ADR-001: Why WebSocket + gRPC (not just one)
  - ADR-002: Why JWT for auth (vs mTLS only)
  - ADR-003: Why async/await throughout
  - ADR-004: Why support both ROS1 and ROS2

- [ ] **#11 API Reference** (4 hours)
  - Auto-generated from docstrings
  - Sphinx or MkDocs
  - Hosted on GitHub Pages

- [ ] **#12 Tutorials** (8 hours)
  - Tutorial 1: Quick start with mock mode
  - Tutorial 2: Connect to ROS2 robot
  - Tutorial 3: Build custom action
  - Tutorial 4: Deploy with Docker
  - Tutorial 5: Integrate with LangChain

- [ ] **#13 Troubleshooting Guide** (2 hours)
  - Common errors and solutions
  - Debug logging
  - Connection issues
  - TLS certificate problems

## 🟡 HIGH PRIORITY — Post-Launch

### Agentic AI Features

- [ ] **#14 Agent Memory System** (32 hours)
  ```python
  from agent_ros_bridge.memory import AgentMemory
  
  # Redis backend for production
  memory = AgentMemory(backend="redis", url="redis://localhost:6379")
  
  # Store conversation history
  await memory.set("conversation", messages, ttl=3600)
  
  # Semantic search with embeddings
  similar = await memory.search("pick up the red object", k=5)
  ```

- [ ] **#15 Tool Discovery** (24 hours)
  ```python
  # Auto-discover from ROS introspection
  tools = bridge.discover_tools()
  
  # Returns MCP-compatible tool definitions
  [
    {
      "name": "ros_navigate",
      "description": "Navigate to coordinates",
      "parameters": {...}
    }
  ]
  ```

- [ ] **#16 Action Confirmation System** (24 hours)
  ```python
  @bridge.action("move_arm", safety_level="dangerous")
  async def move_arm(position: str, confirm: Confirmation):
      await confirm.request(f"Move arm to {position}?")
  ```

- [ ] **#17 Multi-Agent Orchestrator** (40 hours)
  ```python
  from agent_ros_bridge.multi_agent import FleetManager
  
  fleet = FleetManager(bridge)
  await fleet.deploy([
      {"robot": "bot_1", "task": "patrol_zone_a"},
      {"robot": "bot_2", "task": "patrol_zone_b"}
  ])
  ```

### Integrations

- [ ] **#18 LangChain Tool** (16 hours)
  ```python
  from langchain.agents import Tool
  
  ros_tool = Tool(
      name="ros",
      func=bridge.execute,
      description="Control ROS robot"
  )
  ```

- [ ] **#19 AutoGPT Plugin** (8 hours)
  - Implement AutoGPT plugin interface
  - Expose all actions as commands

- [ ] **#20 ROS2 Actions Support** (16 hours)
  - Native ROS2 action client integration
  - Support for Navigation2, MoveIt actions

### Observability

- [ ] **#21 Prometheus Metrics** (16 hours)
  ```python
  from prometheus_client import Counter, Histogram, Gauge
  
  action_counter = Counter('ros_bridge_actions_total', 'Actions executed', ['action', 'status'])
  action_latency = Histogram('ros_bridge_action_latency_seconds', 'Action latency', ['action'])
  connected_agents = Gauge('ros_bridge_connected_agents', 'Number of connected agents')
  ```

- [ ] **#22 OpenTelemetry Tracing** (16 hours)
  - Distributed tracing across agents
  - Trace action execution: agent → bridge → ROS

- [ ] **#23 Real-time Dashboard** (32 hours)
  - React + WebSocket
  - Live robot telemetry
  - Agent activity stream
  - Action history
  - Emergency stop

## 🟢 MEDIUM PRIORITY — Nice to Have

- [ ] **#24 Gazebo/Isaac Sim Connector** (24 hours)
  - Connect to simulation environments
  - Test before deploying to real robots

- [ ] **#25 Learning from Demonstration** (32 hours)
  - Record human teleop
  - Replay as learned skills
  - Store in skill library

- [ ] **#26 Helm Chart** (8 hours)
  - Kubernetes deployment
  - ConfigMaps for configuration
  - Secrets for TLS/JWT

- [ ] **#27 Terraform Module** (8 hours)
  - Deploy to AWS/GCP/Azure
  - EC2/GKE with ROS

- [ ] **#28 Benchmark Suite** (16 hours)
  - Performance tests
  - Latency benchmarks
  - Throughput tests

## ✅ Phase 1 Complete: Testing Foundation

- [x] **Test Directory Structure** — tests/{unit,integration,e2e,fixtures}/
- [x] **pytest Configuration** — markers, coverage (80% target)
- [x] **Unit Tests** — ActionRegistry, TransportManager, ROSBridge
- [x] **Integration Tests** — WebSocket transport
- [x] **Fixtures** — conftest.py with reusable fixtures
- [x] **CI/CD Pipeline** — GitHub Actions (lint, test, integration)

### Test Files Created
- `tests/unit/test_action_registry.py` — 6 test cases
- `tests/unit/test_transport_manager.py` — 6 test cases  
- `tests/unit/test_rosbridge.py` — 12 test cases
- `tests/integration/test_websocket_transport.py` — 4 test cases
- `tests/conftest.py` — Shared fixtures

### CI/CD Workflows
- `.github/workflows/ci.yml` — Lint, test on Python 3.8-3.12
- `.github/workflows/release.yml` — PyPI, Docker, ClawHub

## ✅ Phase 2 Complete: Agentic AI Features

- [x] **Agent Memory** — SQLite/Redis backends, TTL, conversation history
- [x] **Tool Discovery** — Auto-discover ROS, MCP/OpenAI format export
- [x] **Action Confirmation** — Safety levels, emergency stop, audit

### New Modules
- `agent_ros_bridge/memory.py` — Memory system (11 tests)
- `agent_ros_bridge/discovery.py` — Tool discovery (9 tests)
- `agent_ros_bridge/safety.py` — Confirmation system (12 tests)

### Total: 53 Test Cases

## ✅ Phase 3 Complete: Observability

- [x] **Prometheus Metrics** — Action counts, latency, connections, topics
- [x] **OpenTelemetry Tracing** — Jaeger/OTLP export, span context
- [x] **Real-time Dashboard** — Web UI with live updates, emergency stop
- [x] **Health Checks** — Liveness/readiness probes

### New Modules
- `agent_ros_bridge/metrics.py` — Prometheus metrics (6 tests)
- `agent_ros_bridge/tracing.py` — Distributed tracing
- `agent_ros_bridge/dashboard.py` — Web dashboard

### Total: 59 Test Cases

## ✅ Phase 4 Complete: Ecosystem

- [x] **LangChain Integration** — ROSBridgeTool, ROSAgent for LangChain agents
- [x] **AutoGPT Plugin** — Native AutoGPT plugin support
- [x] **ROS2 Actions** — Nav2, MoveIt action clients
- [x] **Navigation2 Client** — Simplified nav API

### New Modules
- `agent_ros_bridge/langchain.py` — LangChain tools
- `agent_ros_bridge/autogpt.py` — AutoGPT plugin
- `agent_ros_bridge/actions.py` — ROS2 action clients

### Total: 37 Python Files

## ✅ Integrity Verified

- [x] **All imports working** — Core, transports, connectors, MCP
- [x] **All Python files compile** — No syntax errors
- [x] **Module exports verified** — `__all__` defined correctly
- [x] **Package metadata valid** — pyproject.toml correct
- [x] **Demo functional** — Mock mode works
- [x] **Documentation complete** — LICENSE, CONTRIBUTING.md added
- [x] **Internal links valid** — No broken references

## ✅ Release Infrastructure (Complete)

- [x] **PyPI** — `pip install agent-ros-bridge` (via release.yml)
- [x] **GitHub Releases** — Automated via GitHub Actions
- [x] **ClawHub** — skill.yaml manifest for clawhub.ai
- [x] **Docker** — Multi-arch images (amd64 + arm64)
- [x] **OpenClaw Integration** — Privileged cloud features in openclaw.py

## ✅ Platform Support (Complete)

- [x] **macOS** — Homebrew formula, Docker, RoboStack, pip
- [x] **ROS Community Release** — Bloom config, package.xml, release process
- [x] **Linux** — apt, pip, Docker (already supported)
- [x] **Windows** — WSL2, Docker (experimental)

## 📋 Previous Work

- [x] Core ROSBridge class
- [x] ROS1/2 connectors
- [x] WebSocket transport with TLS
- [x] gRPC transport with mutual TLS
- [x] MCP server
- [x] Configuration system
- [x] JWT authentication
- [x] CORS support
- [x] Example dashboard
- [x] README with architecture
- [x] Launch strategy

## 🎯 Current Sprint (This Week)

### Day 1-2: Testing Foundation
- [ ] #1 Create test directory
- [ ] #2 Write first unit test (ActionRegistry)
- [ ] #5 Set up GitHub Actions

### Day 3-4: Docker
- [ ] #7 Base Dockerfile
- [ ] #8 Multi-arch build
- [ ] Test on both amd64 and arm64

### Day 5: Documentation
- [ ] #10 Write first ADR
- [ ] Record 2-minute demo video
- [ ] Draft HN post

---

## How to Contribute

1. Pick an issue from above
2. Comment "Working on #[issue number]"
3. Create feature branch: `git checkout -b feature/[issue-number]-description`
4. Write tests first (TDD)
5. Implement feature
6. Ensure CI passes
7. Submit PR with description

## Definition of Done

- [ ] Code follows project style (black, ruff)
- [ ] Tests written and passing
- [ ] Documentation updated
- [ ] CHANGELOG.md updated
- [ ] No breaking changes (or properly documented)

---

*Last updated: 2026-02-22*  
*Next review: Weekly*
