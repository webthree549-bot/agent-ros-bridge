---
name: opc-insight-generator
description: "OPC Journal Suite Insight Generation Module - Personalized recommendations and advice. Use when: (1) generating daily/weekly insights, (2) getting personalized recommendations, (3) understanding opportunities and risks. NOT for: professional advice → insights are suggestions, not professional guidance."
metadata:
  {
    "openclaw": {
      "emoji": "💡",
      "requires": {}
    }
  }
---

# opc-insight-generator

**Version**: 2.3.0  
**Status**: Production Ready

## When to Use

✅ **Use this skill for:**

- Generating daily or weekly insights
- Getting personalized recommendations
- Identifying opportunities and risks
- Understanding historical context
- Receiving actionable suggestions

## When NOT to Use

❌ **Don't use when:**

- Need professional business or legal advice
- Require medical or psychological guidance
- Want automated decision-making

## Description

OPC Journal Suite Insight Generation Module - Generates personalized insights and recommendations based on journal entries, patterns, and milestones.

## When to use

- User asks "What should I do?", "Give me advice"
- Generate daily/weekly insights
- Identify opportunities and risks
- Provide actionable recommendations

## Tools

- `memory_search` - Search historical data
- `read` - Read journal entries and patterns
- `write` - Write insights and recommendations
- `sessions_list` - Review session history

## Usage

### Generate Daily Insight

```python
generate_daily_insight(
    customer_id="OPC-001",
    day=45,
    sources=["journal_entries", "patterns", "milestones"],
    include_recommendations=True
)
```

**Output Example:**
```yaml
insight:
  day: 45
  theme: "Pivot Consideration"
  summary: "You've spent 2 weeks on marketing with limited traction"
  
  observations:
    - "Marketing efforts not yielding expected results"
    - "User feedback points to product-market fit issues"
    - "Previous pivots (Day 12, Day 28) led to breakthroughs"
    
  recommendations:
    - priority: "high"
      action: "Conduct 5 user interviews this week"
      rationale: "Previous interviews led to key insights"
      
    - priority: "medium"
      action: "Consider narrowing target audience"
      rationale: "General appeal may be diluting message"
      
  historical_context:
    similar_situations:
      - day: 12
        outcome: "Pivot led to current product direction"
      - day: 28
        outcome: "MVP pivot increased user engagement 3x"
```

### Generate Weekly Report

```python
generate_weekly_report(
    customer_id="OPC-001",
    week=7,
    include_patterns=True,
    include_milestones=True,
    include_recommendations=True
)
```

**Output Example:**
```yaml
weekly_report:
  week: 7
  days_tracked: [43, 44, 45, 46, 47, 48, 49]
  
  highlights:
    - "Completed product launch milestone (Day 45)"
    - "Identified key user pain point through feedback"
    - "Established consistent daily journaling habit"
    
  patterns_detected:
    - type: "work_rhythm"
      finding: "Peak productivity Wed-Fri afternoons"
      recommendation: "Schedule important decisions accordingly"
      
    - type: "decision_style"
      finding: "Conservative risk appetite, 2.3 day avg hesitation"
      recommendation: "Set 24-hour decision deadline for non-critical choices"
      
  upcoming_milestones:
    - name: "First Paying Customer"
      estimated_day: 52
      confidence: 0.73
      preparation_tasks:
        - "Set up payment processing"
        - "Prepare onboarding flow"
        
  recommendations:
    - "Focus on user interviews next week"
    - "Consider paid acquisition test with small budget"
    - "Document learnings for future reference"
```

## Configuration

子 skill 配置继承自主 `config.yml` 的 `insight` 部分。完整配置参见：
`~/.openclaw/skills/opc-journal-suite/config.yml`

```yaml
insight:
  generation_frequency: "daily"
  include_recommendations: true
  personalization_enabled: true
  sources:
    - journal_entries
    - patterns
    - milestones
```

## Integration

```python
# Trigger after journal entry
def on_journal_entry_created(entry):
    insight = generate_contextual_insight(
        customer_id=entry.customer_id,
        trigger_entry=entry,
        historical_context=30  # Last 30 days
    )
    
    if insight.urgency == "high":
        notify_user(insight.summary)
    
    return insight
```

## Best Practices

1. **Timely Generation** - Generate insights when patterns change or milestones occur
2. **Actionable Focus** - Prioritize recommendations that can be acted upon
3. **Historical Context** - Reference similar past situations for relevance
4. **Balanced Tone** - Celebrate wins, acknowledge struggles, provide hope
5. **User Control** - Allow users to adjust insight frequency and depth
