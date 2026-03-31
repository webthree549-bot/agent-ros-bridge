---
name: opc-milestone-tracker
description: "OPC Journal Suite Milestone Tracking Module - Achievement detection and celebration. Use when: (1) detecting important moments in user journey, (2) generating milestone reports, (3) tracking progress toward goals. NOT for: automated sharing → celebrations are local-only."
metadata:
  {
    "openclaw": {
      "emoji": "🎯",
      "requires": {}
    }
  }
---

# opc-milestone-tracker

**Version**: 2.3.0  
**Status**: Production Ready

## When to Use

✅ **Use this skill for:**

- Auto-detecting milestones from journal entries
- Tracking product/business milestones (first sale, MVP, etc.)
- Generating celebration reports
- Monitoring progress toward goals
- Creating milestone timelines

## When NOT to Use

❌ **Don't use when:**

- Need automated social media sharing
- Require external notifications for milestones
- Want public leaderboards or comparisons

## Description

OPC Journal Suite Milestone Tracking Module - Automatically detects important moments in the user journey, generates achievement reports, and provides continuous motivation and direction.

**LOCAL-ONLY**: All milestone data is stored locally. No external sharing or notifications.

## When to use

- User says "I completed...", "Finally done"
- Auto-detect milestones (first launch, first sale, etc.)
- Generate 100-day reports, annual reviews
- Need to motivate user to keep going

## Milestone Types

### 1. Technical Milestones

- **first_deployment**: First independent app deployment
- **first_contribution**: First open source contribution
- **technical_breakthrough**: Solved long-standing technical problem

### 2. Business Milestones

- **first_sale**: First customer payment received
- **revenue_targets**: Monthly recurring revenue milestones
- **customer_targets**: Customer count milestones
- **product_milestones**: MVP complete, v1 release

### 3. Growth Milestones

- **ai_collaboration**: First agent delegation, multi-agent workflow
- **personal_development**: 100 days milestone, skill mastery
- **community**: First help given, knowledge sharing

## Usage

### Detect Milestones

```python
result = detect_milestone({
    "customer_id": "OPC-001",
    "input": {
        "content": "Finally launched the product today!",
        "day": 45
    }
})
```

Response:
```json
{
  "status": "success",
  "result": {
    "milestones_detected": [
      {
        "milestone_id": "first_product_launch",
        "description": "First product launch",
        "day": 45,
        "confidence": 0.8
      }
    ]
  }
}
```

## Configuration

子 skill 配置继承自主 `config.yml` 的 `milestone` 部分。完整配置参见：
`~/.openclaw/skills/opc-journal-suite/config.yml`

```yaml
milestone:
  auto_detect: true
  celebration_enabled: true
  min_confidence: 0.7
  types:
    - product_launch
    - first_customer
    - revenue_milestone
    - team_expansion
```

## Data Privacy

- All milestone data stored locally in customer directory
- No external sharing
- No network calls

## Scripts

- `init.py`: Initialize milestone tracker for customer
- `detect.py`: Detect milestones from journal entries

## Tests

6 tests covering milestone detection scenarios.

## License

MIT - OPC200 Project
