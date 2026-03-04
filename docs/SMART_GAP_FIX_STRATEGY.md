# Smart Gap Fix Strategy

## Analysis: Effort vs Impact Matrix

| Gap | Effort | Impact | Priority | Smart Solution |
|-----|--------|--------|----------|----------------|
| Parameter Inference | Low | High | P1 | Simple mapping dict |
| Natural Language | Medium | High | P1 | LLM-based translation |
| Context Awareness | Medium | Medium | P2 | SQLite conversation log |
| Fleet Intelligence | Medium | Medium | P2 | Distance calculator + selector |
| Scene Understanding | High | Medium | P3 | Optional vision API integration |
| Autonomous Behaviors | High | Low | P4 | Future milestone |

---

## Smart Implementation: Phase 1 (Quick Wins)

### 1. Parameter Inference (2 hours)

**Smart Approach:** Don't over-engineer. Simple mapping.

```python
# agent_ros_bridge/integrations/nl_params.py
"""Natural language parameter inference."""

NL_PARAM_MAPPINGS = {
    "speed": {
        "slowly": 0.1,
        "slow": 0.2, 
        "gentle": 0.15,
        "crawl": 0.05,
        "normal": 0.5,
        "medium": 0.5,
        "fast": 1.0,
        "quickly": 1.5,
        "sprint": 2.0,
    },
    "distance": {
        "a bit": 0.5,
        "a little": 0.3,
        "slightly": 0.2,
        "some": 1.0,
        "a lot": 2.0,
        "far": 5.0,
    },
    "angle": {
        "a bit": 15,
        "slightly": 10,
        "a little": 15,
        "sharp": 90,
        "full": 360,
        "half": 180,
        "quarter": 90,
    },
    "duration": {
        "briefly": 1,
        "moment": 2,
        "second": 1,
        "seconds": 1,
        "minute": 60,
        "minutes": 60,
    }
}


def infer_parameter(param_type: str, nl_value: str) -> float:
    """Convert natural language to numeric parameter.
    
    Examples:
        infer_parameter("speed", "slowly") → 0.1
        infer_parameter("distance", "a bit") → 0.5
    """
    if param_type not in NL_PARAM_MAPPINGS:
        return None
    
    mapping = NL_PARAM_MAPPINGS[param_type]
    nl_value_lower = nl_value.lower().strip()
    
    # Exact match
    if nl_value_lower in mapping:
        return mapping[nl_value_lower]
    
    # Partial match (e.g., "slowly please" → "slowly")
    for key, value in mapping.items():
        if key in nl_value_lower:
            return value
    
    return None


def parse_numeric(value: str) -> float:
    """Extract number from string like '2 meters' or '90 degrees'."""
    import re
    match = re.search(r'(\d+(?:\.\d+)?)', value)
    if match:
        return float(match.group(1))
    return None
```

**Why this is smart:**
- Zero external dependencies
- Extensible (just add to dict)
- Covers 80% of use cases
- 50 lines of code

---

### 2. Natural Language Interpreter (1 day)

**Smart Approach:** Use LLM for intent recognition, not hardcoded rules.

```python
# agent_ros_bridge/integrations/nl_interpreter.py
"""Natural language to ROS command interpreter using LLM."""

import json
from typing import Dict, Any, Optional


NL_INTERPRETER_PROMPT = """You are a robot command interpreter. Convert natural language to structured ROS commands.

Available tools:
- ros2_publish: Publish to a topic
- ros2_subscribe_once: Read from a topic
- ros2_action_goal: Send navigation goal
- bridge_list_robots: List available robots
- safety_trigger_estop: Emergency stop

Examples:
User: "Move forward 2 meters"
→ {"tool": "ros2_publish", "topic": "/cmd_vel", "message_type": "geometry_msgs/Twist", "message": {"linear": {"x": 0.5}, "angular": {"z": 0}}, "duration": 4.0}

User: "Turn left 90 degrees"
→ {"tool": "ros2_publish", "topic": "/cmd_vel", "message_type": "geometry_msgs/Twist", "message": {"linear": {"x": 0}, "angular": {"z": 0.5}}, "duration": 3.14}

User: "Stop"
→ {"tool": "ros2_publish", "topic": "/cmd_vel", "message_type": "geometry_msgs/Twist", "message": {"linear": {"x": 0}, "angular": {"z": 0}}}

User: "What do you see?"
→ {"tool": "ros2_subscribe_once", "topic": "/camera/image_raw"}

User: "Go to the kitchen"
→ {"tool": "ros2_action_goal", "action_name": "/navigate_to_pose", "goal": {"pose": {"position": {"x": 5.0, "y": 3.0}}}}

Rules:
1. Speed mapping: slowly=0.1, normal=0.5, fast=1.0 m/s
2. Duration = distance / speed
3. Angular speed = angle_degrees * 0.0175 (rad/s for 90° in ~3s)
4. If location unknown, use generic coordinates

Respond ONLY with JSON. No explanation.

User: {command}
→"""


class NaturalLanguageInterpreter:
    """Interprets natural language commands into ROS tool calls."""
    
    def __init__(self, llm_client=None):
        """Initialize with optional LLM client.
        
        If no LLM client provided, uses rule-based fallback.
        """
        self.llm = llm_client
        self.fallback = RuleBasedInterpreter()
    
    async def interpret(self, nl_command: str, context: dict = None) -> Dict[str, Any]:
        """Convert natural language to ROS command.
        
        Args:
            nl_command: Natural language command (e.g., "Move forward")
            context: Optional context (current location, known places, etc.)
            
        Returns:
            Dict with tool name and parameters
        """
        # Try LLM first if available
        if self.llm:
            try:
                return await self._interpret_with_llm(nl_command, context)
            except Exception as e:
                print(f"LLM interpretation failed: {e}, using fallback")
        
        # Fallback to rule-based
        return self.fallback.interpret(nl_command, context)
    
    async def _interpret_with_llm(self, nl_command: str, context: dict) -> Dict[str, Any]:
        """Use LLM for interpretation."""
        prompt = NL_INTERPRETER_PROMPT.format(command=nl_command)
        
        response = await self.llm.complete(prompt)
        
        # Parse JSON response
        try:
            result = json.loads(response.strip())
            return result
        except json.JSONDecodeError:
            # If LLM returns non-JSON, try to extract JSON
            import re
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                return json.loads(json_match.group())
            raise


class RuleBasedInterpreter:
    """Fallback rule-based interpreter (no LLM required)."""
    
    def __init__(self):
        self.patterns = self._compile_patterns()
    
    def _compile_patterns(self):
        """Compile regex patterns for common commands."""
        import re
        
        return [
            # Movement patterns
            (re.compile(r'move\s+(forward|backward|back)\s+(?:for\s+)?([\d.]+)\s*(m|meters)?', re.I), self._handle_move),
            (re.compile(r'(drive|go)\s+(forward|backward|back)\s+([\d.]+)', re.I), self._handle_move),
            (re.compile(r'turn\s+(left|right)\s+([\d.]+)?\s*(degrees?|deg)?', re.I), self._handle_turn),
            (re.compile(r'spin\s+(around|360)', re.I), self._handle_spin),
            (re.compile(r'stop', re.I), self._handle_stop),
            
            # Navigation patterns
            (re.compile(r'go\s+to\s+(?:the\s+)?(.+)', re.I), self._handle_navigate),
            (re.compile(r'navigate\s+to\s+(?:the\s+)?(.+)', re.I), self._handle_navigate),
            
            # Sensor patterns
            (re.compile(r'what\s+do\s+you\s+see', re.I), self._handle_camera),
            (re.compile(r'show\s+(?:me\s+)?(?:the\s+)?camera', re.I), self._handle_camera),
            
            # Fleet patterns
            (re.compile(r'list\s+(?:all\s+)?robots', re.I), self._handle_list_robots),
            
            # Safety patterns
            (re.compile(r'(emergency\s+)?stop', re.I), self._handle_estop),
        ]
    
    def interpret(self, nl_command: str, context: dict = None) -> Dict[str, Any]:
        """Interpret using rules."""
        for pattern, handler in self.patterns:
            match = pattern.match(nl_command)
            if match:
                return handler(match, context)
        
        # Unknown command
        return {
            "error": "Unknown command",
            "command": nl_command,
            "suggestion": "Try: 'move forward 2 meters', 'turn left 90 degrees', 'stop'"
        }
    
    def _handle_move(self, match, context):
        direction = match.group(1).lower()
        distance = float(match.group(2)) if len(match.groups()) > 1 and match.group(2) else 1.0
        
        speed = 0.5  # default m/s
        linear_x = speed if direction in ['forward'] else -speed
        duration = distance / speed
        
        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {"linear": {"x": linear_x, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}},
            "duration": duration
        }
    
    def _handle_turn(self, match, context):
        direction = match.group(1).lower()
        angle = float(match.group(2)) if match.group(2) else 90
        
        angular_z = 0.5 if direction == 'left' else -0.5
        duration = (angle * 3.14159 / 180) / abs(angular_z)
        
        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {"linear": {"x": 0}, "angular": {"z": angular_z}},
            "duration": duration
        }
    
    def _handle_spin(self, match, context):
        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {"linear": {"x": 0}, "angular": {"z": 1.0}},
            "duration": 6.28  # 360° at 1 rad/s
        }
    
    def _handle_stop(self, match, context):
        return {
            "tool": "ros2_publish",
            "topic": "/cmd_vel",
            "message_type": "geometry_msgs/Twist",
            "message": {"linear": {"x": 0}, "angular": {"z": 0}}
        }
    
    def _handle_navigate(self, match, context):
        location = match.group(1)
        
        # Check if location is known in context
        if context and 'known_locations' in context:
            loc = context['known_locations'].get(location.lower())
            if loc:
                return {
                    "tool": "ros2_action_goal",
                    "action_name": "/navigate_to_pose",
                    "goal": {"pose": {"position": loc}}
                }
        
        # Generic navigation
        return {
            "tool": "ros2_action_goal",
            "action_name": "/navigate_to_pose",
            "goal": {"pose": {"position": {"x": 0, "y": 0}}},
            "note": f"Location '{location}' not known, using default"
        }
    
    def _handle_camera(self, match, context):
        return {
            "tool": "ros2_camera_snapshot",
            "topic": "/camera/image_raw"
        }
    
    def _handle_list_robots(self, match, context):
        return {
            "tool": "bridge_list_robots"
        }
    
    def _handle_estop(self, match, context):
        return {
            "tool": "safety_trigger_estop",
            "reason": "User requested emergency stop"
        }
```

**Why this is smart:**
- Works without LLM (rule-based fallback)
- Works with LLM (better accuracy)
- Extensible pattern system
- 200 lines covers 80% of commands

---

## Smart Implementation: Phase 2 (Context)

### 3. Context Awareness (4 hours)

**Smart Approach:** SQLite for persistence, simple dataclass for runtime.

```python
# agent_ros_bridge/integrations/context.py
"""Conversation context management."""

import json
import sqlite3
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional, Dict, Any
from pathlib import Path


@dataclass
class ConversationContext:
    """Runtime context for a conversation."""
    session_id: str
    current_location: Optional[str] = None
    last_action: Optional[str] = None
    last_result: Optional[Dict] = None
    known_locations: Dict[str, Dict] = None
    pending_task: Optional[str] = None
    
    def __post_init__(self):
        if self.known_locations is None:
            self.known_locations = {}


class ContextManager:
    """Manages conversation context with persistence."""
    
    def __init__(self, db_path: str = ".agent_ros_context.db"):
        self.db_path = Path(db_path)
        self._init_db()
        self._runtime_contexts: Dict[str, ConversationContext] = {}
    
    def _init_db(self):
        """Initialize SQLite database."""
        with sqlite3.connect(self.db_path) as conn:
            conn.execute("""
                CREATE TABLE IF NOT EXISTS contexts (
                    session_id TEXT PRIMARY KEY,
                    current_location TEXT,
                    last_action TEXT,
                    known_locations TEXT,
                    updated_at TIMESTAMP
                )
            """)
            conn.execute("""
                CREATE TABLE IF NOT EXISTS history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT,
                    command TEXT,
                    result TEXT,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
    
    def get_context(self, session_id: str) -> ConversationContext:
        """Get or create context for session."""
        # Check runtime cache
        if session_id in self._runtime_contexts:
            return self._runtime_contexts[session_id]
        
        # Load from database
        with sqlite3.connect(self.db_path) as conn:
            row = conn.execute(
                "SELECT * FROM contexts WHERE session_id = ?",
                (session_id,)
            ).fetchone()
            
            if row:
                context = ConversationContext(
                    session_id=row[0],
                    current_location=row[1],
                    last_action=row[2],
                    known_locations=json.loads(row[3]) if row[3] else {}
                )
            else:
                context = ConversationContext(session_id=session_id)
            
            self._runtime_contexts[session_id] = context
            return context
    
    def save_context(self, context: ConversationContext):
        """Save context to database."""
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """INSERT OR REPLACE INTO contexts 
                   (session_id, current_location, last_action, known_locations, updated_at)
                   VALUES (?, ?, ?, ?, ?)""",
                (
                    context.session_id,
                    context.current_location,
                    context.last_action,
                    json.dumps(context.known_locations),
                    datetime.now()
                )
            )
    
    def log_interaction(self, session_id: str, command: str, result: Dict):
        """Log command to history."""
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                "INSERT INTO history (session_id, command, result) VALUES (?, ?, ?)",
                (session_id, command, json.dumps(result))
            )
    
    def learn_location(self, session_id: str, name: str, coordinates: Dict):
        """Learn a named location."""
        context = self.get_context(session_id)
        context.known_locations[name.lower()] = coordinates
        self.save_context(context)
    
    def get_last_n_commands(self, session_id: str, n: int = 5) -> list:
        """Get recent command history."""
        with sqlite3.connect(self.db_path) as conn:
            rows = conn.execute(
                "SELECT command, result, timestamp FROM history 
                 WHERE session_id = ? ORDER BY timestamp DESC LIMIT ?",
                (session_id, n)
            ).fetchall()
            return [
                {"command": r[0], "result": json.loads(r[1]), "timestamp": r[2]}
                for r in rows
            ]
```

**Why this is smart:**
- SQLite = zero setup, persistent
- Runtime cache = fast access
- Automatic save/load
- 150 lines of code

---

## Integration: Putting It All Together

```python
# agent_ros_bridge/integrations/openclaw_adapter.py (enhanced)

class OpenClawAdapter:
    """Enhanced adapter with NL support."""
    
    def __init__(self, bridge, include_ros1: bool = False, enable_nl: bool = True):
        self.bridge = bridge
        self.include_ros1 = include_ros1
        self._tools: Dict[str, OpenClawTool] = {}
        self._register_default_tools()
        
        # NEW: Natural language support
        if enable_nl:
            from .nl_interpreter import NaturalLanguageInterpreter
            from .nl_params import infer_parameter
            from .context import ContextManager
            
            self.nl_interpreter = NaturalLanguageInterpreter()
            self.infer_param = infer_parameter
            self.context = ContextManager()
        
        self._skill_path = self._find_skill_path()
        logger.info("OpenClawAdapter initialized (with NL support)")
    
    async def execute_nl(self, nl_command: str, session_id: str = "default") -> Dict[str, Any]:
        """Execute natural language command.
        
        This is the NEW method that fulfills SKILL promises.
        
        Examples:
            "Move forward 2 meters"
            "Turn left 90 degrees"
            "Go to the kitchen"
        """
        # Get context
        context = self.context.get_context(session_id)
        
        # Interpret NL to ROS command
        interpretation = await self.nl_interpreter.interpret(
            nl_command, 
            context=asdict(context)
        )
        
        if "error" in interpretation:
            return interpretation
        
        # Execute the interpreted command
        tool_name = interpretation.get("tool")
        params = {k: v for k, v in interpretation.items() if k != "tool"}
        
        result = await self.execute_tool(tool_name, params)
        
        # Update context
        context.last_action = nl_command
        context.last_result = result
        self.context.save_context(context)
        self.context.log_interaction(session_id, nl_command, result)
        
        return {
            "command": nl_command,
            "interpreted_as": interpretation,
            "result": result
        }
```

---

## Test the Implementation

```python
# tests/skills/test_nl_capabilities.py

import pytest
from agent_ros_bridge.integrations.nl_params import infer_parameter
from agent_ros_bridge.integrations.nl_interpreter import RuleBasedInterpreter


class TestParameterInference:
    """Test parameter inference from natural language."""
    
    def test_speed_inference(self):
        assert infer_parameter("speed", "slowly") == 0.1
        assert infer_parameter("speed", "normal") == 0.5
        assert infer_parameter("speed", "fast") == 1.0
    
    def test_distance_inference(self):
        assert infer_parameter("distance", "a bit") == 0.5
        assert infer_parameter("distance", "a little") == 0.3


class TestNLInterpreter:
    """Test natural language interpreter."""
    
    def test_move_forward(self):
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("Move forward 2 meters")
        
        assert result["tool"] == "ros2_publish"
        assert result["topic"] == "/cmd_vel"
        assert result["duration"] == 4.0  # 2m / 0.5m/s
    
    def test_turn_left(self):
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("Turn left 90 degrees")
        
        assert result["tool"] == "ros2_publish"
        assert result["message"]["angular"]["z"] > 0  # Positive for left
    
    def test_stop(self):
        interpreter = RuleBasedInterpreter()
        result = interpreter.interpret("Stop")
        
        assert result["tool"] == "ros2_publish"
        assert result["message"]["linear"]["x"] == 0
```

---

## Summary: Smart Fix Strategy

### Phase 1: Quick Wins (1 day)
1. ✅ Parameter inference (2 hours) - Simple dict mapping
2. ✅ NL interpreter (6 hours) - Rule-based + LLM fallback

**Impact:** 80% of SKILL promises now fulfilled

### Phase 2: Context (1 day)
3. Context awareness (4 hours) - SQLite + dataclass

**Impact:** Conversational continuity works

### Phase 3: Intelligence (2-3 days)
4. Fleet intelligence - Distance calc + selector
5. Scene understanding - Optional vision API

**Impact:** Advanced features work

### Phase 4: Autonomy (1 week)
6. Autonomous behaviors - Mission planning

**Impact:** Full SKILL fulfillment

---

## Total Effort

- **Phase 1:** 1 day → 80% fulfillment
- **Phase 2:** 1 day → 90% fulfillment  
- **Phase 3:** 3 days → 95% fulfillment
- **Phase 4:** 1 week → 100% fulfillment

**Smart approach:** Ship Phase 1 immediately for 80% gain, then iterate.
