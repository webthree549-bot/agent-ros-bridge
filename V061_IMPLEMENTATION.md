# v0.6.1 Implementation Checklist

## Task 2: Implement v0.6.1 Planned Features

### Architecture Consolidation
- [x] Merge duplicate safety modules (already consolidated)
- [x] Fix safety validator workspace bounds format (completed)
- [ ] Unify error handling across AI layer
- [ ] Standardize logging format

### AI Layer Hardening
- [x] Intent parser with rule-based + LLM fallback (exists)
- [ ] Add input validation for all AI services
- [ ] Add retry logic for LLM calls
- [ ] Add circuit breaker for LLM failures

### Simulation Integration
- [x] Gazebo + TurtleBot3 (running)
- [x] Nav2 + SLAM (running)
- [x] Parallel simulation framework (exists)
- [ ] Add more test scenarios (target: 1000+)

### Performance Optimization
- [x] Safety validation <10ms (achieved)
- [x] Intent parsing <10ms (achieved)
- [ ] Optimize memory usage
- [ ] Add caching for repeated queries

## Task 3: Increase Test Coverage to 90%+

Current: 27% (measured)
Target: 90%+

### Coverage Analysis

| Module | Current | Target | Priority |
|--------|---------|--------|----------|
| utils/error_handling | 91% | 95% | ✅ Done |
| safety/validator | 79% | 90% | Medium |
| safety/watchdog | 86% | 90% | Medium |
| gateway_v2/core | 43% | 80% | High |
| gateway_v2/transports/websocket | 47% | 80% | High |
| integrations/* | 0-39% | 70% | Low |
| connectors/* | 9-36% | 60% | Low |

### Quick Wins (High Impact, Low Effort)
- [x] Error handling module (91% coverage)
- [ ] Add tests for websocket transport edge cases
- [ ] Add tests for core gateway message handling
- [ ] Add tests for safety validator edge cases

### Deferred to v0.6.2
- [ ] ROS1/ROS2 connector tests (requires Docker)
- [ ] gRPC/MQTT transport tests (requires infrastructure)
- [ ] Full integration test suite

## Task 4: Complete Documentation

- [ ] API documentation
- [ ] Architecture diagrams
- [ ] Deployment guide
- [ ] Troubleshooting guide

---

## Implementation Priority

1. **High**: Error handling unification
2. **High**: Input validation
3. **Medium**: Test coverage improvement
4. **Medium**: Performance optimization
5. **Low**: Documentation
