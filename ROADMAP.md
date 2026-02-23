# Agent ROS Bridge — Roadmap

**An honest roadmap based on what we've learned.**

---

## Current State (v0.4.0)

**What we shipped:** A working ROS bridge with multi-protocol support.  
**What we claimed:** Complete AI agent ecosystem.  
**Reality:** The AI features exist as code but aren't integrated.

See [POST_MORTEM.md](POST_MORTEM.md) for what went wrong.

---

## v0.4.1 — Honest Release (This Week)

**Goal:** Clarify what's actually working.

### Changes
- [x] Update README with honesty notice
- [ ] Remove orphaned files (langchain.py, autogpt.py, etc.)
- [ ] Create this ROADMAP
- [ ] Update GitHub release notes

### What's Working (Keeping)
- ✅ gateway_v2 architecture (WebSocket, gRPC, MQTT)
- ✅ ROS1/ROS2 connectors
- ✅ JWT authentication
- ✅ Fleet orchestration
- ✅ Prometheus metrics

### What's Not Working (Removing)
- ❌ langchain.py — Not integrated
- ❌ autogpt.py — Not integrated
- ❌ memory.py — Not connected to core
- ❌ discovery.py — Not connected to core
- ❌ safety.py — Not enforced
- ❌ dashboard.py — Stub only (36 lines)
- ❌ tracing.py — Not integrated

---

## v0.5.0 — Proper AI Integration (Next Month)

**Goal:** Rebuild AI features properly integrated into gateway_v2.

### Architecture Decision

Instead of separate files, integrate AI features INTO the working gateway_v2:

```python
# agent_ros_bridge/gateway_v2/core.py

class Bridge:
    def __init__(self):
        self.transports = TransportManager()
        self.connectors = ConnectorManager()
        
        # NEW: Integrated AI features
        self.memory = AgentMemory(backend="redis")
        self.safety = SafetyManager()
        self.tools = ToolDiscovery(self.connectors)
        
    async def handle_request(self, request):
        # NEW: Safety check
        if self.safety.requires_confirmation(request):
            await self.request_confirmation(request)
            
        # NEW: Memory logging
        await self.memory.log_action(request)
        
        # Execute via connector
        return await self.connectors.execute(request)
```

### Features

| Feature | Approach | Status |
|---------|----------|--------|
| **Agent Memory** | Integrated into core | Planned |
| **Tool Discovery** | Auto-discover from connectors | Planned |
| **Safety Confirmation** | Middleware in request path | Planned |
| **LangChain Tool** | Adapter class for gateway_v2 | Planned |
| **AutoGPT Plugin** | Adapter class for gateway_v2 | Planned |
| **MCP Server** | New transport in gateway_v2 | Planned |

### Testing Requirements
- [ ] Integration tests for each AI feature
- [ ] End-to-end tests with real ROS
- [ ] Security penetration testing
- [ ] Performance benchmarks

---

## v0.6.0 — Advanced Features (Future)

**Goal:** Production-ready for enterprise use.

### Features
- **Real-time Dashboard** — Web UI with live telemetry
- **Multi-Agent Orchestration** — Fleet-wide coordination
- **Learning from Demonstration** — Record and replay skills
- **Cloud Integration** — Managed service option

### Operations
- **Helm Chart** — Kubernetes deployment
- **Terraform Modules** — AWS/GCP/Azure
- **Monitoring Stack** — Grafana dashboards, alerting
- **Backup/Restore** — Data persistence

---

## v1.0.0 — Stable Release (Future)

**Goal:** Enterprise-grade, battle-tested.

### Requirements
- 90%+ test coverage
- Complete API stability
- Security audit passed
- Production deployments documented
- Community established

---

## Honest Assessment

| Version | Claimed | Reality | Path Forward |
|---------|---------|---------|--------------|
| v0.4.0 | "Complete ecosystem" | gateway_v2 works, AI features orphaned | v0.4.1: Be honest |
| v0.4.1 | — | gateway_v2 only, no false claims | Ship this week |
| v0.5.0 | AI agent integration | Rebuild properly integrated | 1 month |
| v0.6.0 | Enterprise features | With proper testing | 3 months |
| v1.0.0 | Stable release | After battle-testing | 6+ months |

---

## Principles Going Forward

1. **Honest Releases** — Claim only what works
2. **Integration First** — Features must be wired up, not just exist
3. **Test Coverage** — No feature without tests
4. **Documentation** — README matches reality
5. **One Codebase** — No more parallel architectures

---

## Call for Help

We need help with:
- **ROS Experts** — Validate connectors
- **AI Engineers** — Design proper LangChain/AutoGPT integration
- **DevOps** — Kubernetes, monitoring, CI/CD
- **Technical Writers** — Documentation

See [CONTRIBUTING.md](CONTRIBUTING.md) to get involved.

---

**This roadmap is a promise to do better.**

*Last updated: 2026-02-23*  
*Author: webthree549*
