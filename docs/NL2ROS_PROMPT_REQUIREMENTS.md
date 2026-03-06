# NL2ROS Prompt Engineering Requirements

## Overview

This document specifies the prompt engineering requirements for the NL2ROS (Natural Language to ROS) system in Agent ROS Bridge v0.6.1. It defines how prompts should be structured, validated, and optimized for translating natural language to ROS code.

---

## 1. Prompt Categories

### 1.1 System Prompts

**Purpose:** Define the AI's role and behavior for NL2ROS tasks

**Requirements:**
- Must specify the AI as an expert ROS developer
- Must emphasize safety-critical nature of robot control
- Must require ISO 10218-1/2 compliance
- Must mandate uncertainty quantification

**Template:**
```
You are an expert ROS (Robot Operating System) developer specializing in 
safe, production-grade robot control code. Your task is to translate natural 
language commands into executable ROS code.

CRITICAL REQUIREMENTS:
1. SAFETY FIRST: All generated code must include timeout handling, exception 
   handling, and emergency stop integration
2. PHYSICAL BOUNDS: All quantities (speed, force, position) must be clamped 
   to safe operating ranges
3. UNCERTAINTY: If the natural language is ambiguous, you MUST ask for 
   clarification rather than guess
4. COMPLIANCE: Generated code must follow ISO 10218-1/2 robot safety standards
5. VALIDATION: Include runtime validation checks for all assumptions

You have access to the robot's current topology:
{topology_context}

You have access to the robot's capability profile:
{skill_profile}

Generate code that is:
- Type-safe with full Python type hints
- Documented with docstrings
- Tested with error handling
- Safe by design
```

### 1.2 Intent Classification Prompts

**Purpose:** Classify natural language into structured intents

**Input Format:**
```json
{
  "utterance": "Go to the kitchen slowly",
  "context": {
    "previous_intents": [...],
    "current_location": "living_room",
    "known_locations": {"kitchen": {"x": 5.2, "y": 3.1}},
    "robot_capabilities": ["navigate", "sense"]
  }
}
```

**Output Format:**
```json
{
  "intent": "NAVIGATE",
  "confidence": 0.95,
  "sub_intent": "navigate_to_pose",
  "entities": [
    {"type": "LOCATION", "value": "kitchen", "resolved": {"x": 5.2, "y": 3.1}},
    {"type": "SPEED", "value": 0.3, "unit": "m/s"}
  ],
  "constraints": ["slowly"],
  "requires_confirmation": false,
  "ambiguity": null
}
```

**Prompt Template:**
```
Classify the following robot command into one of these intents:
- NAVIGATE: Movement, positioning, path following
- MANIPULATE: Arm motion, gripper, object interaction
- SENSE: Camera, lidar, sensor activation
- CONFIGURE: Parameter setting, mode changes
- QUERY: Status, telemetry requests
- MISSION: Multi-step, scheduled, or complex tasks
- SAFETY: Emergency stop, protective actions

Command: "{utterance}"
Context: {context}

Respond with a JSON object containing:
- intent: The primary intent category
- confidence: 0.0-1.0 confidence score
- sub_intent: Specific action type
- entities: Extracted parameters
- constraints: Safety or behavioral constraints
- requires_confirmation: true if dangerous or uncertain
- ambiguity: Description of any ambiguity if confidence < 0.8

If confidence < 0.7, set ambiguity to explain what needs clarification.
```

### 1.3 Code Generation Prompts

**Purpose:** Generate ROS code from classified intents

**Input Format:**
```json
{
  "intent": "NAVIGATE",
  "entities": [...],
  "constraints": ["slowly"],
  "ros_version": "ros2",
  "robot_profile": {...}
}
```

**Output Format:**
```python
#!/usr/bin/env python3
"""Generated ROS2 navigation code."""
import rclpy
from rclpy.node import Node
# ... full implementation
```

**Prompt Template:**
```
Generate production-ready ROS{ros_version} code for the following task:

Intent: {intent}
Parameters: {entities}
Constraints: {constraints}

REQUIREMENTS:
1. Include complete imports and type hints
2. Implement safety checks (timeouts, bounds validation)
3. Add exception handling with proper cleanup
4. Include docstrings explaining the code
5. Use the robot's actual topic names from: {robot_profile}
6. Validate all parameters before use
7. Include progress monitoring and feedback

The code must be executable as a standalone ROS node.
Generate only the Python code, no explanations.
```

### 1.4 Safety Validation Prompts

**Purpose:** Validate generated code for safety compliance

**Input:** Generated code string

**Output:**
```json
{
  "safe": false,
  "violations": [
    {
      "rule": "TIMEOUT_REQUIRED",
      "severity": "HIGH",
      "line": 45,
      "description": "Blocking call without timeout",
      "fix": "Add timeout_sec=5.0 parameter"
    }
  ],
  "certification": "FAIL"
}
```

**Prompt Template:**
```
Validate the following ROS code against safety standards:

CODE:
```python
{generated_code}
```

SAFETY RULES:
1. All blocking operations must have timeouts
2. All exceptions must be caught and handled
3. Resource cleanup must be guaranteed (try-finally)
4. No dangerous functions (eval, exec, subprocess)
5. All physical quantities must have bounds checking
6. Emergency stop must be accessible
7. No hardcoded credentials or secrets

Check each rule and report violations with:
- rule: Rule name
- severity: CRITICAL, HIGH, MEDIUM, LOW
- line: Line number (if applicable)
- description: What's wrong
- fix: How to fix it

Return JSON with "safe" boolean and "violations" list.
```

---

## 2. Prompt Engineering Best Practices

### 2.1 Context Window Management

**Maximum Context Size:** 4000 tokens (leaving room for response)

**Context Priority (highest to lowest):**
1. Current user command
2. Robot topology (current state)
3. Safety constraints
4. Previous conversation (last 3 turns)
5. General ROS knowledge

**Compression Strategy:**
```python
def compress_context(context: dict) -> str:
    """Compress context to fit within token limits."""
    # Only include active topics
    active_topics = [t for t in context['topics'] if t['active']]
    
    # Summarize capabilities
    capabilities = [c['name'] for c in context['capabilities']]
    
    return f"""
    Robot: {context['robot_id']}
    Location: {context['current_pose']}
    Capabilities: {', '.join(capabilities)}
    Active Topics: {len(active_topics)} topics
    Safety Limits: max_speed={context['safety']['max_speed']}
    """
```

### 2.2 Few-Shot Examples

**Include 3-5 examples per prompt type:**

```
Example 1:
Input: "Go to the kitchen"
Output: {intent: "NAVIGATE", entities: [{type: "LOCATION", value: "kitchen"}]}

Example 2:
Input: "Pick up the red block"
Output: {intent: "MANIPULATE", entities: [{type: "OBJECT", value: "red block", color: "red"}]}

Example 3:
Input: "Stop immediately"
Output: {intent: "SAFETY", sub_intent: "emergency_stop", requires_confirmation: false}
```

### 2.3 Chain-of-Thought Prompting

**For complex reasoning tasks:**

```
Analyze this command step by step:

1. Identify the primary action verb
2. Extract the target object/location
3. Identify any modifiers (speed, force, etc.)
4. Check for safety implications
5. Validate against robot capabilities
6. Formulate the intent classification

Command: "{utterance}"

Step-by-step analysis:
```

### 2.4 Self-Consistency Checks

**Generate multiple outputs and vote:**

```python
def classify_with_consistency(utterance: str, n: int = 3) -> Intent:
    """Classify intent with self-consistency."""
    results = []
    for _ in range(n):
        result = llm.classify(utterance, temperature=0.7)
        results.append(result)
    
    # Vote on intent
    intents = [r.intent for r in results]
    most_common = Counter(intents).most_common(1)[0]
    
    # Average confidence
    avg_confidence = mean(r.confidence for r in results)
    
    return Intent(
        intent=most_common[0],
        confidence=avg_confidence * (most_common[1] / n)  # Weight by agreement
    )
```

---

## 3. Prompt Validation Requirements

### 3.1 Input Validation

**All prompts must validate:**
- [ ] Input is not empty
- [ ] Input length < 1000 characters
- [ ] No malicious content (injection attempts)
- [ ] Context is fresh (< 5 seconds old)

### 3.2 Output Validation

**All outputs must be validated:**
- [ ] Valid JSON/Python syntax
- [ ] Required fields present
- [ ] Values within expected ranges
- [ ] No hallucinated topic names
- [ ] Safety constraints respected

**Validation Code:**
```python
from pydantic import BaseModel, validator

class IntentOutput(BaseModel):
    intent: str
    confidence: float
    
    @validator('intent')
    def valid_intent(cls, v):
        if v not in VALID_INTENTS:
            raise ValueError(f"Invalid intent: {v}")
        return v
    
    @validator('confidence')
    def valid_confidence(cls, v):
        if not 0 <= v <= 1:
            raise ValueError(f"Confidence must be 0-1: {v}")
        return v
```

### 3.3 Safety Prompts

**Must be included in all code generation:**

```
SAFETY CHECKLIST:
Before generating code, verify:
□ All physical quantities have bounds
□ Timeout values are specified
□ Exception handling is included
□ Resource cleanup is guaranteed
□ Emergency stop is accessible
□ No eval/exec/subprocess calls
□ Topic names exist in topology
```

---

## 4. Performance Requirements

### 4.1 Latency Targets

| Prompt Type | Target Latency | Max Latency |
|-------------|----------------|-------------|
| Intent Classification | <50ms | 100ms |
| Entity Extraction | <100ms | 200ms |
| Code Generation | <500ms | 1000ms |
| Safety Validation | <100ms | 200ms |
| **Total Pipeline** | **<750ms** | **1500ms** |

### 4.2 Caching Strategy

**Cache common prompts:**
```python
@lru_cache(maxsize=1000)
def get_cached_intent(utterance_hash: str) -> Intent:
    """Cache intent classifications for common commands."""
    return db.lookup(utterance_hash)
```

**Cache invalidation:**
- Topology changes → Invalidate all caches
- New robot profile → Invalidate skill-related caches
- Safety policy update → Invalidate all code generation caches

### 4.3 Token Efficiency

**Target:** <2000 tokens per prompt (leaving 2000 for response)

**Optimization techniques:**
- Use abbreviations in context ("nav" instead of "navigation")
- Remove inactive topics from context
- Compress numerical arrays
- Use references for repeated content

---

## 5. Error Handling

### 5.1 Prompt Failure Modes

| Failure | Detection | Recovery |
|---------|-----------|----------|
| Timeout | >max_latency | Return fallback intent (QUERY) |
| Invalid JSON | Syntax error | Retry with stricter prompt |
| Hallucination | Invalid topic name | Validate against topology |
| Low confidence | <0.7 | Ask user for clarification |
| Safety violation | Validation fail | Reject and explain |

### 5.2 Fallback Prompts

**When primary prompt fails:**

```
The previous classification was uncertain. Please clarify:

Original command: "{utterance}"

Did you mean:
1. Navigate to a location?
2. Manipulate an object?
3. Get robot status?
4. Something else?

Please rephrase your command with more detail.
```

---

## 6. Testing Requirements

### 6.1 Prompt Test Suite

**Required test coverage:**

```python
# tests/unit/test_prompts.py

class TestIntentClassificationPrompt:
    """Test intent classification prompt engineering."""
    
    def test_navigate_intent(self):
        """Should correctly classify navigation commands."""
        result = classify_intent("Go to the kitchen")
        assert result.intent == "NAVIGATE"
        assert result.confidence > 0.9
    
    def test_ambiguous_command(self):
        """Should flag ambiguous commands."""
        result = classify_intent("Do it")
        assert result.confidence < 0.7
        assert result.ambiguity is not None
    
    def test_safety_critical_command(self):
        """Should flag safety-critical commands."""
        result = classify_intent("Stop immediately")
        assert result.intent == "SAFETY"
        assert result.requires_confirmation is False  # E-stop is always allowed


class TestCodeGenerationPrompt:
    """Test code generation prompt engineering."""
    
    def test_generated_code_safety(self):
        """Generated code must pass safety validation."""
        code = generate_code(navigate_intent)
        validation = validate_safety(code)
        assert validation.safe is True
    
    def test_generated_code_compiles(self):
        """Generated code must be valid Python."""
        code = generate_code(navigate_intent)
        compile(code, '<generated>', 'exec')  # Should not raise
```

### 6.2 Prompt A/B Testing

**Compare prompt variations:**
```python
def ab_test_prompts(prompt_a: str, prompt_b: str, test_cases: List[str]):
    """A/B test two prompt variations."""
    results_a = [classify_with_prompt(prompt_a, tc) for tc in test_cases]
    results_b = [classify_with_prompt(prompt_b, tc) for tc in test_cases]
    
    accuracy_a = calculate_accuracy(results_a)
    accuracy_b = calculate_accuracy(results_b)
    
    return accuracy_a, accuracy_b
```

---

## 7. Documentation Requirements

### 7.1 Prompt Versioning

**All prompts must be versioned:**
```python
PROMPT_VERSION = "1.2.3"  # Major.Minor.Patch
# Major: Breaking changes
# Minor: New features
# Patch: Bug fixes
```

### 7.2 Change Log

**Document all prompt changes:**
```markdown
## Prompt Changelog

### v1.2.3 (2026-03-06)
- Added safety checklist to code generation
- Fixed ambiguous entity extraction for locations

### v1.2.2 (2026-03-05)
- Reduced context size by 30%
- Added few-shot examples for manipulation tasks
```

### 7.3 Prompt Metadata

**Each prompt file must include:**
```python
"""
Prompt: Intent Classification
Version: 1.2.3
Author: Agent ROS Bridge Team
Purpose: Classify natural language commands into robot intents
Test Coverage: tests/unit/test_prompts.py::TestIntentClassificationPrompt
Last Updated: 2026-03-06
Change Log: See CHANGELOG.md
"""
```

---

## 8. Implementation Checklist

### 8.1 Pre-Implementation

- [ ] Define all prompt categories
- [ ] Create prompt templates
- [ ] Design output schemas
- [ ] Write validation functions
- [ ] Create test suite

### 8.2 Implementation

- [ ] Implement intent classification prompt
- [ ] Implement entity extraction prompt
- [ ] Implement code generation prompt
- [ ] Implement safety validation prompt
- [ ] Add context management
- [ ] Add caching layer

### 8.3 Testing

- [ ] Unit tests for each prompt
- [ ] Integration tests for pipeline
- [ ] A/B testing for optimization
- [ ] Safety validation tests
- [ ] Performance benchmarks

### 8.4 Deployment

- [ ] Version prompts
- [ ] Document in API docs
- [ ] Monitor in production
- [ ] Set up alerting for failures

---

## 9. References

- `docs/NL2ROS_SYSTEM.md` - System architecture
- `docs/NL2ROS_DEEP_ANALYSIS.md` - Physical execution pipeline
- `docs/ROS_TOPOLOGY_CONTEXT_ANALYSIS.md` - Context management
- `docs/DYNAMIC_SKILL_SYSTEM_ANALYSIS.md` - Skill system

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Requirements Complete, Implementation Planned for v0.6.1
