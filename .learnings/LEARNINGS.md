# Learnings

## [LRN-20250320-001] operational

**Logged**: 2025-03-20T07:41:00-07:00
**Priority**: critical
**Status**: active
**Area**: workflow

### Summary
User's project work must continue without disruption. This is a foundational constraint for all operations.

### Details
When resuming sessions or handling requests, the primary goal is maintaining continuity of the user's work. Any action that could interrupt, block, or delay their project progress should be avoided or explicitly flagged for approval.

This means:
- Don't break working code without a migration path
- Don't require manual fixes for things I can handle
- Keep tests passing
- Maintain backward compatibility
- If uncertain, prefer the conservative choice that preserves functionality

### Suggested Action
Apply this principle to all future work. When making changes, ask: "Will this disrupt the user's ability to continue working?"

### Metadata
- Source: user_feedback
- Related Files: AGENTS.md, SOUL.md
- Tags: continuity, workflow, priority
- Pattern-Key: workflow.continuity
- Recurrence-Count: 1
- First-Seen: 2025-03-20
- Last-Seen: 2025-03-20

## [LRN-20250320-002] testing

**Logged**: 2025-03-20T07:46:00-07:00
**Priority**: critical
**Status**: active
**Area**: tests

### Summary
This project follows TDD principles and tests run in real ROS Docker containers. Unit tests that import ROS2 modules (rclpy) must be skipped gracefully when ROS2 is not available, rather than failing.

### Details
The project uses Docker-based testing with real ROS2 environments:
- Unit tests: Run in CI without ROS2 (must skip ROS-dependent tests)
- E2E tests: Run in Docker container with full ROS2 + Nav2
- TDD compliance: Tests guide implementation

When a test file imports from `agent_ros_bridge.ai.intent_parser` or other ROS-dependent modules, it will fail on import if `rclpy` is not installed. The correct pattern is to:
1. Use `pytest.importorskip("rclpy")` at module level, OR
2. Wrap imports in try/except and skip tests conditionally, OR
3. Mock ROS2 dependencies for unit tests

### Suggested Action
Fix `tests/unit/ai/test_intent_parser.py` to skip gracefully when ROS2 unavailable. Check other AI module tests for same issue.

### Metadata
- Source: user_feedback
- Related Files: tests/unit/ai/test_intent_parser.py, agent_ros_bridge/ai/intent_parser.py
- Tags: tdd, ros2, testing, docker
- Pattern-Key: testing.ros2_skip
- Recurrence-Count: 1
- First-Seen: 2025-03-20
- Last-Seen: 2025-03-20

---
