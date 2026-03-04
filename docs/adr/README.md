# Architecture Decision Records (ADR)

This directory contains Architecture Decision Records (ADRs) for the Agent ROS Bridge project.

## What is an ADR?

An Architecture Decision Record (ADR) captures an important architectural decision made along with its context and consequences. ADRs help teams understand why certain decisions were made and provide historical context for future developers.

## ADR Index

| Number | Title | Status | Date |
|--------|-------|--------|------|
| [0001](0001-gateway-v2-architecture.md) | Gateway V2 Architecture | Accepted | 2026-02 |
| [0002](0002-jwt-authentication.md) | JWT Authentication for All Transports | Accepted | 2026-02 |
| [0003](0003-multi-ros-support.md) | Support Both ROS1 and ROS2 | Accepted | 2026-02 |
| [0004](0004-dynamic-message-types.md) | Dynamic ROS Message Type System | Accepted | 2026-02 |
| [0005](0005-ai-framework-integrations.md) | AI Framework Integration Strategy | Accepted | 2026-02 |
| [0006](0006-tdd-workflow.md) | Test-Driven Development Workflow | Accepted | 2026-03 |
| [0007](0007-code-quality-standards.md) | Code Quality and Linting Standards | Accepted | 2026-03 |
| [0008](0008-openclaw-integration.md) | OpenClaw Integration Architecture | Accepted | 2026-03 |

## Status Definitions

- **Proposed** — Under discussion, not yet decided
- **Accepted** — Decision accepted, being implemented
- **Deprecated** — Decision no longer relevant, superseded
- **Superseded** — Replaced by a newer ADR

## Contributing

When creating a new ADR:
1. Use the next available number
2. Follow the template in [template.md](template.md)
3. Submit as a PR for review
4. Update this index

## References

- [ADR GitHub Organization](https://adr.github.io/)
- [Documenting Architecture Decisions](http://thinkrelevance.com/blog/2011/11/15/documenting-architecture-decisions)
