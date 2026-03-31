---
name: opc-async-task-manager
description: "OPC Journal Suite Async Task Management Module - Local task tracking for background work. Use when: (1) tracking tasks for later, (2) organizing non-urgent work, (3) managing personal todo lists. NOT for: automated execution → tasks are tracked, not executed."
metadata:
  {
    "openclaw": {
      "emoji": "⏰",
      "requires": {}
    }
  }
---

# opc-async-task-manager

**Version**: 2.3.0  
**Status**: Production Ready

## When to Use

✅ **Use this skill for:**

- Creating and tracking async tasks locally
- Organizing non-urgent but important work
- Managing personal task queues
- Setting task deadlines and reminders
- Tracking task completion status

## When NOT to Use

❌ **Don't use when:**

- Need automated task execution
- Require external notifications on deadline
- Want team task assignment
- Need integration with external task tools

## Description

OPC Journal Suite Async Task Management Module - Local task tracking for background work organization.

**LOCAL-ONLY**: Tasks are tracked and stored locally. No external execution, no network calls, no notifications to external services.

## When to use

- User says "track this for later", "remind me tomorrow"
- Organizing non-urgent but important work
- Managing task queue locally

## Core Concepts

### Task Lifecycle

```
Created → Tracked → Completed/Failed
```

Tasks are local data objects only. No external execution is performed.

### Task Types

| Type | Typical Duration | Example |
|------|-----------------|---------|
| Quick | < 5 min | Simple query |
| Standard | 5-60 min | Document review |
| Long | 1-8 hours | Research task |

## Usage

### Create Task

```python
result = create_task({
    "customer_id": "OPC-001",
    "input": {
        "type": "research",
        "description": "Competitor analysis report",
        "timeout_hours": 8
    }
})
```

Response:
```json
{
  "status": "success",
  "result": {
    "task_id": "TASK-20260321-007",
    "task": {
      "task_type": "research",
      "description": "Competitor analysis report",
      "status": "created",
      "estimated_completion": "2026-03-22T06:00:00"
    },
    "storage_hint": "Store task in memory for tracking"
  }
}
```

**Note**: This creates a task record only. No automatic execution or notification occurs.

### Check Status

```python
result = check_status({
    "customer_id": "OPC-001",
    "input": {
        "task_id": "TASK-20260321-007"
    }
})
```

## Configuration

子 skill 配置继承自主 `config.yml` 的 `async_task` 部分。完整配置参见：
`~/.openclaw/skills/opc-journal-suite/config.yml`

```yaml
async_task:
  max_concurrent: 5
  default_timeout: "8h"
  retry_policy:
    max_retries: 3
    backoff: "exponential"
  # NOTE: Notification channels reserved for future versions
```

## Data Privacy

- All task data stored locally
- No external service calls
- No credentials required
- No network access

## Scripts

- `create.py`: Create local task record
- `status.py`: Check task status

## Tests

6 tests covering task creation and status scenarios.

## License

MIT - OPC200 Project
