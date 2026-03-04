# ADR 0008: OpenClaw Integration Architecture

## Status

**Accepted** — Implemented via ClawHub skill with optional extension mode

## Context

Agent ROS Bridge aims to support multiple AI agent frameworks (LangChain, AutoGPT, MCP/Claude). The OpenClaw ecosystem provides another pathway for AI agents to interact with robots through:

1. **ClawHub** — Skill registry and distribution platform
2. **OpenClaw Framework** — AI agent runtime that can use skills
3. **RosClaw** — Existing ROS2 integration (similar project)

We need to decide how Agent ROS Bridge integrates with OpenClaw.

## Decision

We will provide **two integration modes**:

### 1. Skill Mode (Primary)

Create a ClawHub skill that teaches OpenClaw agents how to use Agent ROS Bridge via its existing APIs (WebSocket, MQTT, gRPC).

**Rationale:**
- Follows OpenClaw ecosystem conventions
- No runtime coupling between systems
- Easy distribution via ClawHub
- Leverages Agent ROS Bridge's existing security (JWT)
- Can be updated independently

**Implementation:**
- SKILL.md with YAML frontmatter
- references/ros1-guide.md and references/ros2-guide.md
- scripts/package_skill.py for packaging

### 2. Extension Mode (Optional)

Provide an OpenClawAdapter class that can be used as a native OpenClaw extension for tighter integration.

**Rationale:**
- Power users may want tighter integration
- Real-time bidirectional streaming
- State management across connections
- Future-proofing for advanced use cases

**Implementation:**
- `openclaw_adapter.py` with tool definitions
- `get_openclaw_adapter()` method on Bridge class
- Direct tool execution support

## Consequences

### Positive

1. **Ecosystem Alignment**: Skill mode follows ClawHub conventions
2. **Flexibility**: Users can choose integration depth
3. **Maintainability**: Skill is easier to maintain than native extension
4. **Security**: Leverages existing JWT auth, no new attack surface
5. **Discoverability**: Can be published to ClawHub for discovery

### Negative

1. **Complexity**: Two integration modes to maintain
2. **Documentation**: Need to document both approaches
3. **Testing**: Additional test coverage needed for OpenClaw path

### Neutral

1. **RosClaw Compatibility**: Extension mode provides RosClaw-compatible tools
2. **Migration**: Users can start with skill, migrate to extension if needed

## Alternatives Considered

### Alternative 1: Native Extension Only

**Rejected**: Too complex for initial integration, tight coupling, harder to distribute.

### Alternative 2: Skill Only

**Considered**: Simpler but limits power users who need tight integration.

**Decision**: Provide skill as primary, extension as optional.

### Alternative 3: Fork RosClaw

**Rejected**: RosClaw is ROS2-only, TypeScript-based, and has different architecture goals.

## Implementation

### Phase 1: Skill (Completed)

```
skills/agent-ros-bridge/
├── SKILL.md              # Main skill definition
├── references/
│   ├── ros1-guide.md    # ROS1-specific instructions
│   └── ros2-guide.md    # ROS2-specific instructions
└── scripts/
    └── package_skill.py # Packaging for ClawHub
```

**Tests:** 29 passing (18 structure + 12 references + 5 packaging)

### Phase 2: Extension (Optional)

```
agent_ros_bridge/integrations/
└── openclaw_adapter.py   # Extension mode support
```

**Features:**
- Tool definitions (RosClaw-compatible + extensions)
- Direct execution support
- Skill packaging integration

## Usage Examples

### Skill Mode

```bash
# Package skill
cd skills/agent-ros-bridge
python scripts/package_skill.py

# Upload to ClawHub
# Users install: npx clawhub install agent-ros-bridge
```

```python
# In OpenClaw agent
# Skill automatically provides guidance for:
# - Connecting to Agent ROS Bridge
# - Sending robot commands
# - Reading sensors
# - Managing fleets
```

### Extension Mode

```python
from agent_ros_bridge import Bridge

bridge = Bridge()
adapter = bridge.get_openclaw_adapter()

# Direct tool execution
result = await adapter.execute_tool("ros2_publish", {
    "topic": "/cmd_vel",
    "message": {"linear": {"x": 0.5}}
})
```

## Comparison with RosClaw

| Aspect | RosClaw | Agent ROS Bridge + OpenClaw |
|--------|---------|----------------------------|
| **Integration** | Tight (extension) | Loose (skill) + Tight (extension) |
| **ROS Support** | ROS2 only | ROS1 + ROS2 |
| **Language** | TypeScript | Python |
| **Distribution** | GitHub | ClawHub (skill) + PyPI (extension) |
| **Security** | Not specified | JWT + RBAC |
| **Fleet Mgmt** | Portal concept | Built-in orchestrator |

## References

- [ClawHub](https://clawhub.ai) — Skill registry
- [OpenClaw](https://openclaw.ai) — AI agent framework
- [RosClaw](https://github.com/Plaipin/rosclaw) — Similar project
- [SKILL.md](../skills/agent-ros-bridge/SKILL.md) — Our skill definition
- [TDD Plan](TDD_PLAN_OPENCLAW.md) — Implementation details

## Decision Record

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-03-04 | Skill + Extension | Best of both worlds |
| 2026-03-04 | TDD approach | Ensure quality |
| 2026-03-04 | ClawHub distribution | Ecosystem alignment |
