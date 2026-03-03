# ADR-0001: Gateway V2 Architecture

## Status

**Accepted**

## Context

The original Agent ROS Bridge (v0.1-v0.3) had a monolithic architecture that tightly coupled transport protocols, ROS connectors, and AI integrations. This made it difficult to:

- Add new transport protocols without modifying core code
- Support both ROS1 and ROS2 simultaneously
- Integrate with multiple AI frameworks
- Test components in isolation
- Maintain and evolve the codebase

As the project grew to support WebSocket, MQTT, gRPC, LangChain, AutoGPT, and MCP, the need for a modular, plugin-based architecture became critical.

## Decision

We will adopt a layered architecture called "Gateway V2" with the following structure:

```
┌─────────────────────────────────────┐
│         AI Agent Layer              │
│  LangChain · AutoGPT · MCP · Custom │
└──────────────┬──────────────────────┘
               │ WebSocket/MQTT/gRPC
┌──────────────▼──────────────────────┐
│      Gateway V2 Core                │
│  ┌─────────┐ ┌─────────┐ ┌────────┐ │
│  │Transport│ │  Auth   │ │ Fleet  │ │
│  │ Manager │ │  (JWT)  │ │  Mgmt  │ │
│  └────┬────┘ └─────────┘ └────────┘ │
│       │                             │
│  ┌────▼──────────────────────────┐  │
│  │      Connector Registry       │  │
│  │   ROS1 Connector · ROS2      │  │
│  │   Simulator · Industrial     │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
```

### Key Components

1. **Transport Manager** — Pluggable transport protocols (WebSocket, MQTT, gRPC)
2. **Connector Registry** — Abstracted robot connectors (ROS1, ROS2, simulator)
3. **Plugin System** — Hot-reloadable plugins for extensibility
4. **Auth Layer** — JWT-based authentication with RBAC
5. **Fleet Management** — Multi-robot orchestration

### Design Principles

1. **Separation of Concerns** — Each layer has a single responsibility
2. **Plugin Architecture** — Components are loadable at runtime
3. **Async-First** — All I/O operations are async/await
4. **Type Safety** — Full type hints throughout
5. **Testability** — Components can be mocked and tested in isolation

## Consequences

### Positive

- **Modularity** — New transports/connectors can be added without core changes
- **Testability** — Each component can be unit tested independently
- **Flexibility** — Users can mix and match components (e.g., WebSocket + ROS2)
- **Maintainability** — Clear boundaries make debugging and refactoring easier
- **Scalability** — Fleet management enables multi-robot deployments

### Negative

- **Complexity** — More abstractions to understand for new contributors
- **Overhead** — Plugin system adds slight runtime overhead
- **Documentation** — Requires comprehensive docs to explain architecture

### Neutral

- **Migration Cost** — Required significant refactoring from v0.3 to v0.5
- **Learning Curve** — Developers must understand the layered architecture

## Alternatives Considered

### Monolithic Architecture (Status Quo)

**Rejected:** Would become unmaintainable as features grew. Already experiencing pain points in v0.3.

### Microservices Architecture

**Rejected:** Overkill for this use case. Would introduce deployment complexity, network latency, and operational overhead not justified by the problem domain.

### Event-Driven Architecture (Pure)

**Rejected:** While we use events internally, a pure event-driven approach would make request-response patterns (common in robot control) more complex.

## References

- [Architecture Documentation](../ARCHITECTURE_V2.md)
- [Plugin System Design](../PLUGIN_SYSTEM.md) (if exists)
- Related: [ADR-0002: JWT Authentication](0002-jwt-authentication.md)
- Related: [ADR-0003: Multi-ROS Support](0003-multi-ros-support.md)
