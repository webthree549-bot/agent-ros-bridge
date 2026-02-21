# Unified Demo Server

**All examples in one Docker container with web-based selector.**

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Unified Demo Container                                      │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Web Dashboard (port 8080)                            │   │
│  │  - Example selector                                   │   │
│  │  - Start/Stop controls                                │   │
│  │  - Live logs                                          │   │
│  └──────────────────────────────────────────────────────┘   │
│                          │                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Example Manager                                      │   │
│  │  - talking-garden                                     │   │
│  │  - mars-colony                                        │   │
│  │  - theater-bots                                       │   │
│  │  - art-studio                                         │   │
│  │  - actions, auth, metrics, mqtt, fleet, arm          │   │
│  └──────────────────────────────────────────────────────┘   │
│                          │                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Shared ROS2 Environment                              │   │
│  │  - Single ROS_DOMAIN_ID                               │   │
│  │  - Shared topics                                      │   │
│  └──────────────────────────────────────────────────────┘   │
│                          │                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Agent ROS Bridge (port 8765)                         │   │
│  │  - Single bridge instance                             │   │
│  │  - WebSocket API                                      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
  http://localhost:8080
```

## Benefits

| Aspect | Separate Containers | Unified Container |
|--------|-------------------|-------------------|
| **Memory** | 4GB × N examples | 4GB × 1 (shared ROS2) |
| **Startup** | `cd example && docker-compose up` | `docker-compose up` |
| **Switching** | Stop A, start B | Click UI button |
| **Discovery** | Read READMEs | Browse web UI |
| **Demo Mode** | ❌ Multiple terminals | ✅ Single window |

## When to Use

**✅ Use Unified Demo:**
- First-time users exploring the project
- Conference demos / presentations
- Quick evaluation
- Users who want to see all features

**✅ Use Separate Examples:**
- Learning specific features
- Development and debugging
- Custom modifications
- Production deployment

## Implementation

```yaml
# docker-compose.yml
services:
  unified-demo:
    build: .
    ports:
      - "8080:8080"    # Web dashboard
      - "8765:8765"    # WebSocket API
    environment:
      - JWT_SECRET=${JWT_SECRET}
    volumes:
      - ./examples:/app/examples:ro
```

## Trade-offs

**Pros:**
- ✅ One command to run everything
- ✅ Web UI for easy navigation
- ✅ Shared ROS2 (lower memory)
- ✅ Better first-time experience

**Cons:**
- ❌ Larger Docker image
- ❌ All examples bundled (even if unused)
- ❌ Harder to modify individual examples
- ❌ Port conflicts if running multiple instances

## Recommendation

**Keep both approaches:**

1. **Individual examples** (current) — For learning/development
2. **Unified demo** (new) — For showcase/quickstart

This gives users choice based on their needs.
