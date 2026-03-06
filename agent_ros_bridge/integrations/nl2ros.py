# NL2ROS Implementation Module

See full implementation in the agent_ros_bridge package.

## Quick Start

```python
from agent_ros_bridge.integrations.nl2ros import NL2ROSTranslator

translator = NL2ROSTranslator(ros_version='ros2')
result = translator.translate("Go to the kitchen slowly")

print(result.generated_code.code)
print(result.safety_validation.status)
print(result.human_readable_summary)
```

## Architecture

The NL2ROS system uses a 7-stage pipeline:

1. **Intent Classification** - Categorize command type
2. **Entity Extraction** - Extract locations, quantities, speeds
3. **Context Resolution** - Resolve ambiguities using context
4. **ROS Primitive Mapping** - Map to ROS topics/services/actions
5. **Code Generation** - Generate industrial-grade Python code
6. **Safety Validation** - Validate against safety rules
7. **Output Formatting** - Structure final result

## Features

- Rule-based intent classification (7 categories)
- Entity extraction with SI unit normalization
- Safety validation with critical issue detection
- Code generation for navigation, manipulation, safety
- Execution metadata estimation
- Human-readable summaries

## Integration

Integrates with Agent ROS Bridge via:
- WebSocket protocol for real-time translation
- Context manager for conversation history
- Safety manager for dangerous operation confirmation
- Tool discovery for available ROS primitives
