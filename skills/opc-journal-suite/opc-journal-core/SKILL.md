---
name: opc-journal-core
description: "OPC Journal Suite Core Module - Smart journaling with automatic linking and retrieval. Use when: (1) recording journal entries, (2) searching historical entries, (3) exporting journals. NOT for: complex queries across multiple users → data is customer-scoped."
metadata:
  {
    "openclaw": {
      "emoji": "📔",
      "requires": {}
    }
  }
---

# opc-journal-core

**Version**: 2.3.0  
**Status**: Production Ready

## When to Use

✅ **Use this skill for:**

- Recording structured journal entries with context
- Searching and retrieving historical journal data
- Linking related entries automatically
- Exporting journals to various formats
- Generating daily/weekly digests

## When NOT to Use

❌ **Don't use when:**

- Need complex multi-customer analytics
- Require real-time collaborative editing
- Need external sync (cloud, other devices)

## Description

OPC Journal Suite Core Module - Provides foundational capabilities for journal recording, retrieval, linking, and summary generation.

## When to use

- User says "log this", "summarize", "journal"
- Need to retrieve historical conversations or decisions
- Generate weekly/monthly/100-day reports
- New user onboarding initialization

## Tools

- `memory_search` - Retrieve historical memories
- `memory_get` - Get specific memory content
- `write` - Write to journal files
- `read` - Read historical journals
- `sessions_list` - View session history

## Usage

### Initialize Journal

```python
# Initialize new user
init_journal(
    customer_id="OPC-001",
    day=1,
    goals=["Complete product MVP", "Acquire first paying customer"],
    preferences={
        "communication_style": "friendly_professional",
        "work_hours": "09:00-18:00",
        "timezone": "Asia/Shanghai"
    }
)
```

### Record Entry

```python
# Record single journal entry
entry = create_entry(
    customer_id="OPC-001",
    content="Completed user registration feature today, but encountered database connection issues",
    metadata={
        "agents_involved": ["DevAgent", "Support"],
        "tasks_completed": ["FEAT-001"],
        "blockers": ["DB-CONN-001"],
        "emotional_state": "frustrated_but_determined",
        "energy_level": 6  # 1-10
    }
)
# Returns: JE-20260321-003
```

### Query with Context

```python
# 关联查询
entries = query_journal(
    customer_id="OPC-001",
    filters={
        "topics": ["database", "technical_debt"],
        "time_range": "last_30_days",
        "emotional_states": ["frustrated", "stuck"]
    },
    include_insights=True
)
```

### Generate Digest

```python
# 生成周报
digest = generate_digest(
    customer_id="OPC-001",
    period="2026-W12",  # Week 12
    format="markdown",
    sections=["summary", "milestones", "blockers", "next_week_focus"]
)
```

## Entry Schema

```yaml
journal_entry:
  id: "JE-{YYYYMMDD}-{SEQ}"
  version: "1.0"
  
  timestamp:
    created: "ISO8601"
    modified: "ISO8601"
    timezone: "Asia/Shanghai"
  
  customer:
    id: "OPC-{XXX}"
    day: 45  # 用户旅程第几天
  
  content:
    raw: "用户原始输入"
    summarized: "AI摘要"
    keywords: ["keyword1", "keyword2"]
  
  context:
    session_id: "session-{uuid}"
    conversation_thread: "thread-{id}"
    referenced_entries: ["JE-{id1}", "JE-{id2}"]
  
  interaction:
    agents_involved: ["Agent1", "Agent2"]
    tasks_created: ["TASK-{id}"]
    tasks_completed: ["TASK-{id}"]
    decisions_made: []
    decisions_deferred: []
  
  analysis:
    emotional_state: "confident|anxious|frustrated|excited|neutral"
    energy_level: 1-10
    urgency: "low|medium|high|critical"
    topic_category: "technical|business|marketing|personal"
  
  memory_flags:
    remember: "需要长期记忆的关键信息"
    follow_up: "需要跟进的事项"
    follow_up_date: "2026-03-28"
  
  privacy:
    level: "normal|sensitive|vault"
    encryption: false|true
    retention: "standard|extended|permanent"
```

## Storage Structure

```
customers/{customer_id}/journal/
├── entries/
│   └── {YYYY}/
│       └── {MM}/
│           └── {DD}/
│               └── JE-{YYYYMMDD}-{SEQ}.yml
├── index/
│   ├── by_topic/
│   ├── by_agent/
│   ├── by_task/
│   └── by_emotion/
├── digests/
│   ├── daily/
│   ├── weekly/
│   └── monthly/
└── export/
    └── {format}/
```

## Examples

### Example 1: Daily Standup

User: "总结一下今天的进展"

System:
```
📋 今日进展摘要 (Day 45)

✅ 已完成:
• 用户注册功能开发 (FEAT-001)
• 数据库 Schema 设计评审

🔄 进行中:
• 支付集成 (预计明天完成)

⚠️ 阻塞:
• 数据库连接池配置问题
  → 已创建技术支持任务 #DB-CONN-001

💭 状态:
您今天提到有些挫折感，但也很有决心。
记得 Day 23 时类似的技术问题，您用了 2 小时就解决了。

📅 明天建议:
基于您的模式，建议上午处理技术问题，下午进行创造性工作。
```

### Example 2: Historical Query

User: "我之前是怎么解决数据库问题的？"

System:
```
🔍 找到 3 个相关记录:

[Day 23] JE-20260228-005
"MySQL 连接超时问题"
解决: 调整连接池参数，增加重试机制
→ 相关文档: /shared/db-best-practices.md

[Day 31] JE-20260308-002  
"Redis 缓存穿透"
解决: 实现布隆过滤器
→ 代码片段: [链接]

[Day 44] JE-20260320-007
"迁移到 PostgreSQL 的考虑"
背景: 当时评估了性能与成本
→ 决策记录: [链接]

💡 模式发现:
您通常会在遇到问题的当天寻求解决方案，
而不是拖延。这是很好的习惯！
```

## Integration

```yaml
# 与其他 Skill 的集成

opc-pattern-recognition:
  input: journal.entries
  output: patterns.insights
  
opc-milestone-tracker:
  input: journal.entries  
  trigger: milestone.detected
  
opc-async-task-manager:
  input: journal.tasks_created
  output: tasks.completed → journal.tasks_completed
  
support_hub:
  access: journal.view_anonymized
  purpose: improve_support_quality
```

## Configuration

子 skill 配置继承自主 `config.yml` 的 `journal` 部分。完整配置参见：
`~/.openclaw/skills/opc-journal-suite/config.yml`

```yaml
journal_core:
  storage:
    backend: "filesystem"
    path: "customers/{customer_id}/journal/"
    compression: true
  indexing:
    enable_fulltext: true
    auto_tag: true
  privacy:
    default_level: "normal"
```

## Best Practices

1. **及时记录** - 重要对话后 5 分钟内记录
2. **完整上下文** - 保留决策背景和原因
3. **关联链接** - 自动链接相关历史记录
4. **情感标注** - 记录情绪状态有助于长期分析
5. **定期回顾** - 每周回顾 Journal 发现模式

## Troubleshooting

| 问题 | 原因 | 解决 |
|------|------|------|
| 查询慢 | 索引未建立 | 运行 `journal rebuild-index` |
| 存储满 | 未归档旧数据 | 配置自动归档策略 |
| 关联不准 | 语义模型问题 | 调整 embedding 模型 |
