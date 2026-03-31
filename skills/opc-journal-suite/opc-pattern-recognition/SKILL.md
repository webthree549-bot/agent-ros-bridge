---
name: opc-pattern-recognition
description: "OPC Journal Suite Pattern Recognition Module - Behavioral analysis and habit identification. Use when: (1) analyzing work patterns and rhythms, (2) identifying decision-making styles, (3) detecting productivity trends. NOT for: predicting future outcomes with certainty → patterns are probabilistic."
metadata:
  {
    "openclaw": {
      "emoji": "🧠",
      "requires": {}
    }
  }
---

# opc-pattern-recognition

**Version**: 2.3.0  
**Status**: Production Ready

## When to Use

✅ **Use this skill for:**

- Analyzing work rhythms and productive hours
- Identifying decision-making patterns
- Detecting stress triggers and recovery patterns
- Tracking learning velocity and skill acquisition
- Understanding emotional patterns in work

## When NOT to Use

❌ **Don't use when:**

- Need guaranteed predictions of future behavior
- Require clinical psychological analysis
- Want to compare with other users (privacy-respecting, no cross-user data)

## Description

OPC Journal Suite Pattern Recognition Module - Analyzes user behavioral patterns, work habits, decision styles, and provides personalized insights and recommendations.

## When to use

- User asks "Why do I always...", "Analyze my habits"
- Auto-generate weekly/monthly insights
- Predict potential difficulties user may encounter
- Optimize Agent collaboration with user

## Tools

- `memory_search` - Retrieve historical data for analysis
- `read` - Read Journal entries
- `write` - Write analysis reports

## Analysis Dimensions

### 1. Work Rhythm

```python
analyze_work_rhythm(
    customer_id="OPC-001",
    metrics=[
        "productive_hours",      # Peak productivity hours
        "peak_focus_duration",   # Maximum focus duration
        "break_patterns",        # Break patterns
        "weekly_energy_curve"    # Weekly energy curve
    ]
)
```

**Output Example:**
```yaml
work_rhythm:
  productive_hours:
    - day: "wednesday"
      time_range: "14:00-17:00"
      effectiveness_score: 9.2
    - day: "friday"  
      time_range: "09:00-12:00"
      effectiveness_score: 8.7
      
  low_energy_periods:
    - day: "monday"
      time_range: "09:00-11:00"
      pattern: "Monday morning startup difficulty"
      
  recommendations:
    - "将重要决策安排在周三下午"
    - "周一早晨安排轻松任务作为热身"
    - "周五上午适合创造性工作"
```

### 2. 决策模式 (Decision Style)

```python
analyze_decision_patterns(
    customer_id="OPC-001",
    decision_types=["technical", "business", "hiring", "pricing"]
)
```

**输出示例:**
```yaml
decision_patterns:
  risk_appetite: "conservative"  # conservative / balanced / aggressive
  
  common_hesitations:
    - topic: "技术选型"
      avg_decision_time: "3.2 days"
      typical_concerns: ["长期维护", "社区活跃度", "学习成本"]
      
    - topic: "定价策略"  
      avg_decision_time: "5.7 days"
      typical_concerns: ["市场接受度", "利润空间", "竞争定位"]
      
  decision_triggers:
    - "收集 3 个以上同类案例"
    - "与至少 1 人讨论"
    - "设定明确截止日期"
    
  improvement_suggestions:
    - "技术选型: 创建标准化评估框架"
    - "定价策略: 准备 3 套方案供选择"
```

### 3. 求助模式 (Help Seeking)

```python
analyze_help_seeking(
    customer_id="OPC-001",
    metrics=["timing", "topics", "resolution_rate"]
)
```

**输出示例:**
```yaml
help_seeking:
  average_delay: "2.3 days"  # 遇到问题到求助的时间
  
  common_topics:
    - "technical_blockers": 45%
    - "business_strategy": 30%  
    - "emotional_support": 15%
    - "tool_usage": 10%
    
  patterns:
    - "倾向于独立尝试 2-3 次后才求助"
    - "深夜时段更容易表达焦虑"
    - "成功解决问题后会主动分享经验"
    
  early_intervention_opportunities:
    - "检测到技术关键词 + 长时间无进展 → 主动提供资源"
    - "连续 2 天能量水平 < 5 → 主动询问状态"
```

### 4. 成长轨迹 (Growth Trajectory)

```python
analyze_growth(
    customer_id="OPC-001",
    time_range="0-90_days",
    dimensions=["technical", "business", "ai_collaboration"]
)
```

**输出示例:**
```yaml
growth_trajectory:
  baseline:  # Day 1
    technical_confidence: 4/10
    ai_collaboration_skill: 2/10
    business_acumen: 3/10
    
  current:  # Day 45
    technical_confidence: 7/10  (+75%)
    ai_collaboration_skill: 8/10 (+300%)
    business_acumen: 5/10 (+67%)
    
  growth_velocity:
    fastest: "ai_collaboration"  # 从"操作员"到"指挥官"
    steady: "technical"          # 持续积累
    needs_attention: "business"  # 增长较慢，建议加强
    
  key_inflection_points:
    - day: 12
      event: "首次独立部署"
      impact: "技术自信心跃升"
    - day: 28  
      event: "完成首笔销售"
      impact: "商业模式验证"
    - day: 45
      event: "多 Agent 协作首次成功"
      impact: "AI 协作能力质变"
```

## Pattern Detection Algorithms

### 1. 重复模式检测

```python
# 检测反复出现的问题
recurring_issues = detect_cycles(
    data=journal_entries,
    pattern_types=["technical_debt", "procrastination", "scope_creep"],
    min_occurrences=3,
    time_window="30_days"
)
```

### 2. 异常检测

```python
# 检测异常行为（可能是机会或风险）
anomalies = detect_anomalies(
    data=journal_entries,
    baseline=historical_patterns,
    sensitivity="medium"
)
# 例如: 突然的工作时间变化、情绪低谷、异常高频求助
```

### 3. 预测模型

```python
# 预测可能遇到的困难
predictions = predict_challenges(
    customer_id="OPC-001",
    upcoming_milestones=["product_launch", "first_marketing_campaign"],
    based_on_similar_customers=True
)
```

## Weekly Pattern Report

自动生成的周模式报告:

```markdown
# 📊 第 12 周模式报告 (Day 78-84)

## 工作节奏
本周您的工作时间分布:
```
Mon ████████░░ 4.2h (低于平均)
Tue ██████████████ 7.8h (正常)  
Wed ████████████████████ 10.5h (高效日)
Thu ██████████████ 7.5h (正常)
Fri ██████████ 5.1h (提前结束)
Sat ████ 2.0h (周末工作)
Sun ██ 1.0h (规划)
```

**发现**: 周三仍然是您最高效的时段
**建议**: 将下周的重要会议安排在周三下午

## 决策回顾
本周做了 5 个重要决定:
- 3 个技术决策 (平均耗时 1.2 天) ← 比之前更快！
- 2 个业务决策 (平均耗时 3.5 天)

**模式**: 您在技术决策上变得更加果断
**建议**: 可以考虑建立业务决策的 SOP

## 情绪曲线
```
能量水平 (1-10):
Mon: 6 ████████░░
Tue: 7 █████████░░  
Wed: 9 ████████████  🌟
Thu: 6 ████████░░
Fri: 4 ██████░░░░  😰
Sat: 7 █████████░░  🎉
Sun: 8 ██████████░░
```

**观察**: 周五下午出现能量低谷，可能是周积累的压力
**建议**: 尝试在周五上午完成最重要的事，下午安排轻松任务

## 与 AI 的协作
- 主动委托任务: 12 次 (+50% vs 上周)
- 平均任务复杂度: 7.2/10 (↑ 说明您在挑战更难的事)
- AI 建议采纳率: 85% (↑ 信任度提升)

## 下周预测
基于您的模式，预测下周可能:
- 在周一早晨遇到启动困难 (概率 78%)
- 周三下午适合进行重要产品决策
- 可能需要在技术文档上寻求帮助

## 个性化建议
1. **利用周三**: 安排最重要的产品决策在周三下午
2. **预防周一低谷**: 周日晚上花 15 分钟规划周一任务
3. **扩展舒适区**: 您已准备好尝试更复杂的 AI 协作模式
```

## Configuration

子 skill 配置继承自主 `config.yml` 的 `pattern_recognition` 部分。完整配置参见：
`~/.openclaw/skills/opc-journal-suite/config.yml`

```yaml
pattern_recognition:
  analysis_schedule:
    daily: "22:00"
    weekly: "sunday_20:00"
    monthly: "last_day"
  dimensions:
    work_rhythm: { enabled: true }
    decision_patterns: { enabled: true }
  predictions:
    enabled: true
    lookahead_days: 7
```

## Privacy Considerations

模式识别涉及敏感的个人行为数据:

1. **数据最小化** - 只收集分析必需的数据
2. **本地优先** - 原始数据存储在客户本地
3. **脱敏共享** - 只有脱敏后的模式可用于比较
4. **用户控制** - 用户可查看、导出、删除自己的模式数据
5. **透明度** - 所有分析逻辑开源可审计

## Best Practices

1. **避免标签化** - 模式是参考，不是定义
2. **关注变化** - 重点检测模式的变化而非模式本身
3. **结合上下文** - 考虑外部因素（健康、家庭等）
4. **提供行动** - 每个洞察都应有可执行的建议
5. **尊重边界** - 不过度解读，保持专业距离
