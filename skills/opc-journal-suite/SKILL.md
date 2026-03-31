---
name: opc-journal-suite
description: "OPC200 User Journal Experience Suite - Personal growth tracking, memory management, and insight generation for One Person Companies. Use when: (1) recording daily progress or journal entries, (2) analyzing work patterns and habits, (3) tracking milestones and achievements, (4) creating async tasks for later, (5) generating personalized insights and advice. NOT for: complex project management → use project tools, team collaboration → use team tools, external notifications → skill is local-only."
user-invocable: true
command-dispatch: tool
tool: coordinate
command-arg-mode: raw
metadata:
  {
    "openclaw": {
      "emoji": "📔",
      "always": false,
      "requires": { "config": ["opc_journal.enabled"] },
      "homepage": "https://docs.opc200.coidea.ai/journal-suite"
    }
  }
---

# opc-journal-suite

**Version**: 2.3.0  
**Status**: Production Ready  
**Last Updated**: 2026-03-30

## When to Use

✅ **Use this skill when you want to:**

- **Record daily progress** - Log what you worked on, decisions made, blockers encountered
- **Journal your journey** - Capture thoughts, ideas, reflections in a structured format
- **Analyze work patterns** - Understand your productive hours, decision style, habits
- **Track milestones** - Automatically detect and celebrate achievements (first launch, first sale, etc.)
- **Create async tasks** - Queue non-urgent work for later tracking
- **Get personalized insights** - Receive advice based on your historical patterns
- **Generate reports** - Weekly summaries, growth trajectories, milestone reviews

## When NOT to Use

❌ **Don't use this skill when:**

- **Complex project management** - Use dedicated PM tools (Linear, Jira, Notion)
- **Team collaboration** - This is for solo founders/OPCs only
- **Real-time notifications** - Skill is local-only, no external alerts (email, Slack, Feishu)
- **Multi-user sharing** - Data is customer-scoped and private
- **External integrations** - No GitHub, no Google Calendar, no third-party APIs
- **Automated execution** - Async tasks are tracked locally, not executed automatically

## Overview

OPC200 User Journal Experience Suite - Complete growth tracking, memory management, and insight generation for One Person Companies (OPC). Includes journaling, pattern recognition, milestone tracking, async task management, and more.

**LOCAL-ONLY**: This is a local-only skill. No network calls, no external APIs, no data sharing. All data is stored in customer-scoped local directories only.

This is a **coordinating skill** that routes user intents to appropriate sub-skills.

## Command Usage

Use `/opc-journal` for direct command-line access to the journal suite:

```
/opc-journal <action> [options]
```

### Actions

| Action | Description | Example |
|--------|-------------|---------|
| `init` | Initialize journal for customer | `/opc-journal init --customer-id OPC-001` |
| `record` | Record a journal entry | `/opc-journal record "Completed MVP today"` |
| `search` | Search journal entries | `/opc-journal search --topic pricing` |
| `export` | Export journal to file | `/opc-journal export --format markdown` |
| `analyze` | Analyze behavior patterns | `/opc-journal analyze --dimension work_hours` |
| `milestone` | Detect milestones | `/opc-journal milestone` |
| `task` | Create async task | `/opc-journal task "Research competitors"` |
| `insight` | Generate insight | `/opc-journal insight` |
| `cron` | Check cron schedules | `/opc-journal cron --action check` |

### Global Options

| Option | Description |
|--------|-------------|
| `--customer-id` | Customer identifier (default: from context) |
| `--day` | Day number in journey |
| `--format` | Output format: yaml, markdown, json |
| `--dry-run` | Preview without executing |

### Examples

```bash
# Initialize new customer journal
/opc-journal init --customer-id OPC-001 --day 1

# Record today's progress
/opc-journal record "Launched landing page, got 50 signups!"

# Generate weekly report
/opc-journal export --period weekly --format markdown

# Analyze work patterns
/opc-journal analyze --days 30 --dimension decision_style

# Create async task for tomorrow
/opc-journal task "Draft investor pitch deck" --due tomorrow
```

## Install

Install the full suite (coordinating skill + all sub-skills):

```bash
clawhub install coidea/opc-journal-suite
```

Or install individual sub-skills:

```bash
clawhub install coidea/opc-journal-core
clawhub install coidea/opc-pattern-recognition
clawhub install coidea/opc-milestone-tracker
clawhub install coidea/opc-async-task-manager
clawhub install coidea/opc-insight-generator
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    OPC Journal Suite                         │
│              (Coordinating Skill - Unified Entry)            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           Intent Detection & Routing                │   │
│  │         (scripts/coordinate.py - 25 tests)          │   │
│  └───────────────────────┬─────────────────────────────┘   │
│                          │                                  │
│         ┌────────────────┼────────────────┐               │
│         │                │                │               │
│         ▼                ▼                ▼               │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐      │
│  │   Journal    │ │   Pattern    │ │  Milestone   │      │
│  │    Core      │ │ Recognition  │ │   Tracker    │      │
│  └──────┬───────┘ └──────────────┘ └──────────────┘      │
│         │                                                  │
│         │  ┌──────────────┐ ┌──────────────┐             │
│         └──┤    Async     │ │   Insight    │             │
│            │  Task Mgr    │ │  Generator   │             │
│            └──────────────┘ └──────────────┘             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Unified Entry Point

Instead of calling individual sub-skills, use the suite as a unified entry point:

### Intent-Based Routing

The suite automatically detects user intent and routes to the appropriate sub-skill:

| User Says | Detected Intent | Routed To |
|-----------|-----------------|-----------|
| "Record my progress today" | `journal_record` | opc-journal-core |
| "分析我的工作习惯" | `pattern_analyze` | opc-pattern-recognition |
| "Detect milestones in my journey" | `milestone_detect` | opc-milestone-tracker |
| "Run this in background" | `task_create` | opc-async-task-manager |
| "Give me advice on what to do" | `insight_generate` | opc-insight-generator |

### Usage

```python
# Unified entry - suite routes to correct sub-skill
result = opc_journal_suite.coordinate(
    customer_id="OPC-001",
    input={"text": "Record my progress today"}
)

# Result includes delegation info
{
    "status": "success",
    "result": {
        "action": "delegate",
        "delegation": {
            "intent": "journal_record",
            "confidence": 0.85,
            "target_skill": "opc-journal-core",
            "customer_id": "OPC-001"
        }
    }
}
```

## Quick Start

### 1. Initialize Journal for Customer

```bash
# Auto-triggered when new customer onboard
opc-journal-init --customer-id OPC-001 --day 1
```

### 2. Record Journal Entry

User: "Just finished the product prototype, but a bit worried about tech stack choices"

System automatically:
- Creates journal entry JE-20260321-001
- Links to previous tech discussion JE-20260318-003
- Flags emotional state: "anxious_but_accomplished"
- Creates follow-up task for tech validation

### 3. Pattern Recognition (Weekly)

Auto-triggered every Sunday:
```
📊 Weekly Pattern Analysis

Work Rhythm:
• Peak hours: Wed afternoon, Fri morning
• Low hours: Mon morning
• Average focus duration: 2.3 hours

Decision Patterns:
• Risk appetite: Conservative
• Common hesitation points: Tech stack, pricing strategy
• Help-seeking timing: Usually 2 days after problem occurs

Recommendations:
"Try scheduling important decisions for Wed afternoon"
"Consider seeking tech advice 1 day earlier"
```

### 4. Milestone Detection

Auto-detected:
```
🎉 Milestone Achieved: First Product Launch

Day: 45
Time: 2026-03-21 14:30
Context: Independently completed full cycle from idea to launch

Previous milestone: MVP Completed (Day 28)
Next predicted: First Sale (Est. Day 52)
```

### 5. Async Task Example

User: "I need a competitive analysis report, due tomorrow morning"

Bot: "Got it! Created async task #RESEARCH-007
     Estimated completion: Tomorrow 8:00 AM
     Will generate summary when done"

[Next morning]
Bot: "☀️ #RESEARCH-007 Ready!
     Discovered 3 key insights, synced to your Journal"

**Note**: Tasks are tracked locally. No external execution or notification services are used.

## Configuration

完整配置参见 `config.yml`，主要配置项如下：

```yaml
# ~/.openclaw/skills/opc-journal-suite/config.yml

journal:
  storage_path: "customers/{customer_id}/journal/"
  retention_days: 365
  privacy_level: "standard"  # standard / sensitive / vault
  
pattern_recognition:
  analysis_frequency: "weekly"
  insight_depth: "detailed"  # brief / detailed / deep
  
milestone:
  auto_detect: true
  celebration_enabled: true
  
async_task:
  max_concurrent: 5
  default_timeout: "8h"

# Cron 调度器配置 (v2.3 新增)
# 支持定时自动生成日报、周分析、里程碑检查等
cron_scheduler:
  enabled: true
  check_interval: "5m"
  schedules:
    daily_summary:
      enabled: true
      schedule: "0 8 * * *"      # 每天 8:00
      target_skill: "opc-insight-generator"
    weekly_pattern:
      enabled: true
      schedule: "0 9 * * 0"      # 每周日 9:00
      target_skill: "opc-pattern-recognition"
    milestone_check:
      enabled: true
      schedule: "0 21 * * *"     # 每天 21:00
      target_skill: "opc-milestone-tracker"
```

### Cron Scheduler (v2.3 新功能)

OPC Journal Suite 2.3.0 新增了内置的 Cron 调度器，支持定时自动触发以下任务：

| 任务 | 默认时间 | 说明 |
|------|---------|------|
| `daily_summary` | 每天 8:00 | 生成每日洞察摘要 |
| `weekly_pattern` | 每周日 9:00 | 进行周度行为模式分析 |
| `milestone_check` | 每天 21:00 | 检测新的里程碑 |
| `memory_compaction` | 每天 23:00 | 内存整理与归档 |

**注意**: 所有调度任务均为本地执行，不涉及外部网络调用。
  # NOTE: Notification channels are reserved for future versions
  # Current implementation is local-only
```

## Data Privacy & Security

### Local-Only Design
- ✅ All data stored in `~/.openclaw/customers/{customer_id}/`
- ✅ No network calls
- ✅ No external APIs
- ✅ No data sharing
- ✅ No credentials required

### Privacy Levels
```yaml
privacy_level: "standard"   # Normal operation
privacy_level: "sensitive"  # Extra care with personal data
privacy_level: "vault"      # Maximum privacy, minimal retention
```

### Data Retention
- Configurable `retention_days` (default: 365)
- Automatic cleanup of old entries
- Customer-scoped isolation

## API Reference

### Journal Core

```python
# Create entry
journal.create_entry(
    customer_id="OPC-001",
    content="用户输入内容",
    context={
        "agents_involved": ["Support"],
        "tasks_created": ["TASK-001"],
        "emotional_state": "confident"
    }
)

# Retrieve with context
entries = journal.query(
    customer_id="OPC-001",
    topic="pricing_strategy",
    time_range="last_30_days",
    include_related=True
)

# Generate digest
digest = journal.generate_digest(
    customer_id="OPC-001",
    period="weekly",
    format="markdown"
)
```

### Pattern Recognition

```python
# Analyze patterns
patterns = pattern_analyzer.analyze(
    customer_id="OPC-001",
    dimensions=["work_hours", "decision_style", "stress_triggers"],
    time_range="last_90_days"
)

# Predict behavior
prediction = pattern_analyzer.predict(
    customer_id="OPC-001",
    scenario:"product_launch"
)
```

### Async Task Manager

```python
# Create async task (local tracking only)
task = async_manager.create(
    customer_id="OPC-001",
    task_type:"research",
    description:"竞品分析报告",
    deadline:"tomorrow 08:00"
)

# Check status
status = async_manager.status(task.id)

# Task results are stored locally, no external callbacks
```

## Directory Structure

```
opc-journal-suite/
├── README.md
├── SKILL.md                      # This file
├── config.yml                    # Default configuration
├── scripts/
│   └── coordinate.py             # Intent routing (25 tests)
├── tests/
│   ├── __init__.py
│   └── test_coordinate.py        # Coordination tests (25 cases)
│
├── opc-journal-core/             # Sub-skill (11 tests)
│   ├── SKILL.md
│   ├── scripts/
│   │   ├── init.py
│   │   ├── record.py
│   │   ├── search.py
│   │   └── export.py
│   └── tests/
│       └── test_core.py
│
├── opc-pattern-recognition/      # Sub-skill (4 tests)
│   ├── SKILL.md
│   ├── scripts/
│   │   └── analyze.py
│   └── tests/
│       └── test_patterns.py
│
├── opc-milestone-tracker/        # Sub-skill (6 tests)
│   ├── SKILL.md
│   ├── scripts/
│   │   └── detect.py
│   └── tests/
│       └── test_milestones.py
│
├── opc-async-task-manager/       # Sub-skill (6 tests)
│   ├── SKILL.md
│   ├── scripts/
│   │   ├── create.py
│   │   └── status.py
│   └── tests/
│       └── test_tasks.py
│
└── opc-insight-generator/        # Sub-skill (3 tests)
    ├── SKILL.md
    ├── scripts/
    │   └── daily_summary.py
    └── tests/
        └── test_insights.py
```

## Test Coverage

| Component | Tests | Status |
|-----------|-------|--------|
| opc-journal-suite (coordination) | 25 | ✅ Pass |
| opc-journal-core | 11 | ✅ Pass |
| opc-pattern-recognition | 4 | ✅ Pass |
| opc-milestone-tracker | 6 | ✅ Pass |
| opc-async-task-manager | 6 | ✅ Pass |
| opc-insight-generator | 3 | ✅ Pass |
| **Total** | **55** | ✅ **All Pass** |

All tests are local unit tests with no network dependencies.

## Development Roadmap

| Version | Features | ETA |
|---------|----------|-----|
| v1.0 | Core journal, basic patterns | 2026.03 |
| v1.1 | Milestone auto-detection | 2026.04 |
| v1.2 | Advanced async tasks | 2026.04 |
| **v2.3** | **Cron scheduler, unified coordination** | **2026.03 (Current)** |
| v2.5 | Voice journal, emotional AI | 2026.06 |

## Contributing

1. Fork the repository
2. Create your feature branch
3. Add tests
4. Submit PR

## License

MIT License - OPC200 Project

## Support

- GitHub Issues: https://github.com/coidea-ai/opc-journal-suite/issues
- Documentation: https://docs.opc200.co/journal-suite

**Note**: This is a local-only skill. For support, please open a GitHub issue rather than using external channels.
