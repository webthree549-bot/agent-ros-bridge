"""opc-journal-suite coordination module.

Routes user intents to appropriate sub-skills.
Acts as a unified entry point for the entire suite.
"""
import json
import re


# Intent patterns for routing
INTENT_PATTERNS = {
    "journal_init": [
        r"init\s+journal",
        r"start\s+journal",
        r"create\s+journal",
        r"initialize",
        r"开始.*日记",
        r"初始化",
    ],
    "journal_record": [
        r"record.*(entry|journal|progress|today)",
        r"log\s+(entry|journal)",
        r"write\s+(entry|journal)",
        r"journal\s+(entry|log)",
        r"记录",
        r"写日记",
        r"日志",
    ],
    "journal_search": [
        r"search\s+journal",
        r"find\s+(entry|entries)",
        r"query\s+journal",
        r"查找",
        r"搜索",
    ],
    "journal_export": [
        r"export\s+journal",
        r"export\s+(entry|entries)",
        r"generate\s+(report|digest)",
        r"导出",
        r"生成报告",
    ],
    "pattern_analyze": [
        r"analyze\s+(pattern|patterns|habits|my|work)",
        r"pattern\s+recognition",
        r"分析",
        r"分析.*习惯",
        r"模式识别",
        r"为什么我总是",
    ],
    "milestone_detect": [
        r"detect\s+milestone",
        r"track\s+milestone",
        r"里程碑",
        r"检测.*里程碑",
    ],
    "task_create": [
        r"create\s+task",
        r"run\s+.*background",
        r"async\s+task",
        r"创建任务",
        r"后台运行",
    ],
    "task_status": [
        r"check\s+task",
        r"task\s+status",
        r"查询任务",
        r"任务状态",
    ],
    "insight_generate": [
        r"generate\s+insight",
        r"give\s+(me\s+)?advice",
        r"what\s+should\s+i\s+do",
        r"建议",
        r"洞察",
    ],
    "daily_summary": [
        r"daily\s+summary",
        r"today.*summary",
        r"日报",
        r"今日总结",
    ],
    "cron_schedule": [
        r"cron\s+schedule",
        r"schedule\s+task",
        r"auto.*trigger",
        r"定时任务",
        r"自动执行",
        r"设置定时",
    ],
}


def detect_intent(user_input: str) -> tuple[str, float]:
    """Detect user intent from input text.
    
    Returns:
        Tuple of (intent_name, confidence_score)
    """
    if not user_input:
        return ("unknown", 0.0)
    
    user_input_lower = user_input.lower()
    
    for intent, patterns in INTENT_PATTERNS.items():
        for pattern in patterns:
            if re.search(pattern, user_input_lower, re.IGNORECASE):
                # Simple scoring: longer match = higher confidence
                confidence = min(0.5 + len(pattern) * 0.05, 0.95)
                return (intent, confidence)
    
    return ("unknown", 0.0)


def get_skill_for_intent(intent: str) -> str | None:
    """Map intent to target sub-skill."""
    skill_map = {
        "journal_init": "opc-journal-core",
        "journal_record": "opc-journal-core",
        "journal_search": "opc-journal-core",
        "journal_export": "opc-journal-core",
        "pattern_analyze": "opc-pattern-recognition",
        "milestone_detect": "opc-milestone-tracker",
        "task_create": "opc-async-task-manager",
        "task_status": "opc-async-task-manager",
        "insight_generate": "opc-insight-generator",
        "daily_summary": "opc-insight-generator",
        "cron_schedule": "opc-journal-suite",
    }
    return skill_map.get(intent)


def main(context: dict) -> dict:
    """Coordinate journal suite operations.
    
    Args:
        context: Dictionary containing:
            - customer_id: The customer identifier
            - input: User input and parameters
            - intent: Optional explicit intent (bypasses detection)
            - params: Optional parameters for the intent
            - memory: Memory context
    
    Returns:
        Dictionary with routing information or direct result
    """
    try:
        customer_id = context.get("customer_id")
        input_data = context.get("input", {})
        explicit_intent = context.get("intent")
        explicit_params = context.get("params", {})
        
        if not customer_id:
            return {
                "status": "error",
                "result": None,
                "message": "customer_id is required"
            }
        
        # If explicit intent provided (e.g., from CLI), use it directly
        if explicit_intent:
            intent = explicit_intent
            confidence = 1.0
            # Merge explicit params into input_data
            input_data.update(explicit_params)
        else:
            user_text = input_data.get("text", "") if isinstance(input_data, dict) else str(input_data)
            action = input_data.get("action", "") if isinstance(input_data, dict) else ""
            
            # Determine intent from text or action
            if action:
                intent = action
                confidence = 1.0
            else:
                intent, confidence = detect_intent(user_text)
        
        if intent == "unknown" or confidence < 0.3:
            return {
                "status": "needs_clarification",
                "result": {
                    "detected_intent": intent,
                    "confidence": confidence,
                    "available_actions": list(INTENT_PATTERNS.keys())
                },
                "message": "Could not determine intent. Please specify: record, analyze, milestone, task, or insight?"
            }
        
        # Map to sub-skill
        target_skill = get_skill_for_intent(intent)
        
        if not target_skill:
            return {
                "status": "error",
                "result": {"intent": intent},
                "message": f"No skill mapped for intent: {intent}"
            }
        
        # Build delegation context
        delegation = {
            "intent": intent,
            "confidence": confidence,
            "target_skill": target_skill,
            "original_input": input_data,
            "customer_id": customer_id
        }
        
        # Check if target skill is installed
        return {
            "status": "success",
            "result": {
                "action": "delegate",
                "delegation": delegation,
                "tool_hint": f"Call '{target_skill}' skill with the provided context"
            },
            "message": f"Routing to {target_skill} (intent: {intent}, confidence: {confidence:.2f})"
        }
        
    except Exception as e:
        return {
            "status": "error",
            "result": None,
            "message": f"Coordination failed: {str(e)}"
        }


if __name__ == "__main__":
    # Test examples
    test_cases = [
        {
            "customer_id": "OPC-TEST-001",
            "input": {"text": "Record a journal entry about today's progress"}
        },
        {
            "customer_id": "OPC-TEST-001",
            "input": {"text": "Analyze my work habits from last week"}
        },
        {
            "customer_id": "OPC-TEST-001",
            "input": {"text": "Create a background task to research competitors"}
        },
        {
            "customer_id": "OPC-TEST-001",
            "input": {"action": "daily_summary", "day": 7}
        }
    ]
    
    for test in test_cases:
        result = main(test)
        print(f"\nInput: {test['input'].get('text') or test['input'].get('action')}")
        print(f"Result: {json.dumps(result, indent=2, ensure_ascii=False)}")
