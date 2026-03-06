# Dynamic Multi-ROS Skill/Profile System Analysis

## Executive Summary

This analysis examines whether Agent ROS Bridge v0.6.1 needs a dynamic, auto-discoverable, AI-modifiable multi-ROS skill/profile data system. The conclusion: **Yes, but with careful architecture** to balance flexibility with safety.

---

## 1. The Problem: Static vs. Dynamic Robot Capabilities

### 1.1 Current State: Static Configuration

```yaml
# Current: config/robots.yaml
robots:
  turtlebot_01:
    type: "turtlebot3_waffle"
    capabilities: ["navigate", "sense"]
    topics:
      cmd_vel: "/turtlebot_01/cmd_vel"
      camera: "/turtlebot_01/camera/image_raw"
    # Problem: Hardcoded, requires restart to change
```

**Limitations:**
- ❌ Robot capabilities change at runtime (plugin loaded, sensor added)
- ❌ AI cannot adapt to new situations
- ❌ Multi-ROS version coordination is manual
- ❌ No learning from experience

### 1.2 The Dynamic Reality

```
Robot Runtime Evolution:
┌─────────────────────────────────────────────────────────────┐
│  T=0: Base robot (navigation only)                          │
│       capabilities = ["navigate"]                           │
├─────────────────────────────────────────────────────────────┤
│  T=1: Arm attached                                          │
│       capabilities += ["manipulate"]  ← Dynamic addition      │
├─────────────────────────────────────────────────────────────┤
│  T=2: Camera driver restarted                               │
│       /camera unavailable → "sense" capability degraded      │
├─────────────────────────────────────────────────────────────┤
│  T=3: Learned new skill "deliver_coffee"                    │
│       capabilities += ["deliver_coffee"]  ← AI-generated      │
├─────────────────────────────────────────────────────────────┤
│  T=4: Fleet coordination active                             │
│       capabilities += ["coordinate", "delegate"]              │
└─────────────────────────────────────────────────────────────┘
```

**Key Insight:** Robot capabilities are not static — they evolve over time.

---

## 2. Requirements Analysis

### 2.1 Who Needs Dynamic Skills?

| Stakeholder | Need | Use Case |
|-------------|------|----------|
| **Robot** | Self-advertise capabilities | "I can now manipulate" |
| **AI Agent** | Discover available actions | "What can this robot do?" |
| **Fleet** | Coordinate heterogeneous robots | "Robot A navigates, Robot B manipulates" |
| **Human** | Understand robot capabilities | "What can you do for me?" |
| **System** | Adapt to runtime changes | Handle sensor failures gracefully |

### 2.2 Multi-ROS Coordination Needs

```
Fleet with Mixed ROS Versions:
┌─────────────────────────────────────────────────────────────┐
│  Robot A: ROS2 Humble (newer)                               │
│    - Topics: /cmd_vel, /odom                                │
│    - Actions: /navigate_to_pose                             │
│    - Services: /slam_toolbox/save_map                       │
├─────────────────────────────────────────────────────────────┤
│  Robot B: ROS1 Noetic (legacy)                              │
│    - Topics: /cmd_vel, /odom                                │
│    - Actions: /move_base                                    │
│    - Services: /static_map                                  │
├─────────────────────────────────────────────────────────────┤
│  AI Agent View (Unified):                                   │
│    - Both have: "navigate" capability                       │
│    - Abstracted: /navigate (routes to appropriate action)   │
│    - Dynamic discovery hides ROS version differences        │
└─────────────────────────────────────────────────────────────┘
```

**Without dynamic discovery:**
- AI must know ROS version of each robot
- Code branches for ROS1 vs ROS2
- Fragile, error-prone

**With dynamic discovery:**
- AI sees unified capability interface
- Runtime adaptation to actual robot capabilities
- Robust to ROS version differences

---

## 3. Proposed Architecture: Dynamic Skill System

### 3.1 Core Concepts

```python
@dataclass
class RobotSkill:
    """A capability that a robot can perform."""
    skill_id: str                    # Unique identifier
    name: str                        # Human-readable name
    description: str                 # NL description
    version: str                     # Skill version
    
    # Requirements
    required_topics: List[str]       # Topics needed
    required_services: List[str]     # Services needed
    required_actions: List[str]      # Actions needed
    required_parameters: List[str]   # Parameters needed
    
    # Runtime info
    provided_by: str                 # Node providing this skill
    ros_version: str                 # ROS1 or ROS2
    status: SkillStatus              # ACTIVE, DEGRADED, UNAVAILABLE
    
    # AI-modifiable
    parameters: Dict[str, Any]       # Tunable parameters
    learned_patterns: List[str]      # NL patterns that trigger this skill
    success_rate: float              # Historical success rate


@dataclass
class RobotProfile:
    """Complete capability profile of a robot."""
    robot_id: str
    hardware_type: str
    ros_version: str
    
    # Dynamic capabilities
    skills: Dict[str, RobotSkill]    # All available skills
    skill_graph: SkillGraph          # Skill dependencies/combinations
    
    # Runtime state
    health: RobotHealth              # Current health status
    active_skills: List[str]         # Currently executing
    
    # AI learning data
    execution_history: List[ExecutionRecord]
    optimized_parameters: Dict[str, Any]
```

### 3.2 Dynamic Discovery Flow

```
┌─────────────────────────────────────────────────────────────────┐
│  DYNAMIC SKILL DISCOVERY PIPELINE                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. TOPOLOGY SCAN                                               │
│     ┌─────────────┐                                             │
│     │ ROS Graph   │ → Topics, Services, Actions, Nodes          │
│     │ Introspect  │                                             │
│     └──────┬──────┘                                             │
│            ↓                                                    │
│  2. SKILL INFERENCE                                             │
│     ┌─────────────┐                                             │
│     │ Pattern     │ → "Has /navigate_to_pose → can navigate"    │
│     │ Matching    │ → "Has /joint_states + arm → can manipulate"│
│     └──────┬──────┘                                             │
│            ↓                                                    │
│  3. SKILL VALIDATION                                            │
│     ┌─────────────┐                                             │
│     │ Test Skill  │ → Actually try to use the skill             │
│     │ (Dry Run)   │ → Verify it works before advertising        │
│     └──────┬──────┘                                             │
│            ↓                                                    │
│  4. SKILL REGISTRATION                                          │
│     ┌─────────────┐                                             │
│     │ Add to      │ → Update robot profile                      │
│     │ Profile     │ → Broadcast to fleet                        │
│     └──────┬──────┘                                             │
│            ↓                                                    │
│  5. AI NOTIFICATION                                             │
│     ┌─────────────┐                                             │
│     │ Notify      │ → AI agent learns new capability            │
│     │ AI Agents   │ → Update tool definitions                   │
│     └─────────────┘                                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3.3 AI-Modifiable Aspects

**What AI Can Modify:**

| Aspect | Modification | Safety Guard |
|--------|--------------|--------------|
| **Parameters** | Tune speed, tolerance, thresholds | Clamp to safe bounds |
| **Patterns** | Add NL triggers for skills | Validate against existing |
| **Priority** | Reorder skill preference | Human confirmation for critical |
| **Composition** | Combine skills ("pick_and_place") | Validate feasibility |
| **Learning** | Success rate, optimal params | Statistical validation |

**What AI Cannot Modify (Safety Critical):**

| Aspect | Reason |
|--------|--------|
| **Hardware limits** | Physical constraints (max speed, joint limits) |
| **Safety policies** | Emergency stop, collision avoidance |
| **Required topics** | Core ROS infrastructure |
| **Skill existence** | Cannot create skills out of thin air |

**Example: AI-Modified Skill**

```python
# Base skill (system defined)
skill = RobotSkill(
    skill_id="navigate",
    name="Navigate to location",
    parameters={
        "max_speed": 0.5,        # System default
        "goal_tolerance": 0.3,    # System default
    }
)

# AI optimization (learned from experience)
# AI learns: "For office environment, max_speed=0.3 is safer"
skill.parameters["max_speed"] = 0.3  # AI-modified, within safe bounds
skill.learned_patterns.append("go to {location} carefully")
skill.success_rate = 0.94  # Updated after 100 executions
```

---

## 4. Multi-ROS Coordination

### 4.1 The Challenge

```
Different ROS Versions = Different Interfaces:

ROS1 Navigation:
  - Action: /move_base
  - Msg: move_base_msgs/MoveBaseAction
  - Feedback: move_base_msgs/MoveBaseFeedback

ROS2 Navigation:
  - Action: /navigate_to_pose
  - Msg: nav2_msgs/NavigateToPose
  - Feedback: nav2_msgs/NavigateToPose_Feedback

AI Agent Should See:
  - Unified: "navigate" skill
  - Abstracted from ROS version differences
```

### 4.2 Solution: Skill Abstraction Layer

```python
class MultiROSSkillAdapter:
    """Adapt ROS1/ROS2 skills to unified interface."""
    
    SKILL_MAPPINGS = {
        "navigate": {
            "ros1": {
                "action": "/move_base",
                "msg_type": "move_base_msgs/MoveBaseAction",
                "goal_converter": ros1_nav_goal_converter,
            },
            "ros2": {
                "action": "/navigate_to_pose",
                "msg_type": "nav2_msgs/NavigateToPose",
                "goal_converter": ros2_nav_goal_converter,
            }
        },
        "pick": {
            "ros1": {
                "action": "/pickup",
                "msg_type": "manipulation_msgs/PickupAction",
            },
            "ros2": {
                "action": "/pick_object",
                "msg_type": "moveit_msgs/Pickup",
            }
        }
    }
    
    def execute_skill(self, skill_name: str, params: Dict, robot: Robot) -> Result:
        """Execute skill on any ROS version."""
        ros_version = robot.profile.ros_version
        mapping = self.SKILL_MAPPINGS[skill_name][ros_version]
        
        # Convert parameters to ROS-specific format
        goal = mapping["goal_converter"](params)
        
        # Execute via appropriate connector
        if ros_version == "ros1":
            return self.ros1_connector.send_goal(mapping["action"], goal)
        else:
            return self.ros2_connector.send_goal(mapping["action"], goal)
```

### 4.3 Dynamic ROS Version Detection

```python
class ROSVersionDetector:
    """Automatically detect ROS version of remote robot."""
    
    def detect(self, host: str, port: int) -> str:
        """Detect ROS version via protocol handshake."""
        # Try ROS2 first (DDS discovery)
        if self._probe_ros2(host, port):
            return "ros2"
        
        # Fall back to ROS1 (XML-RPC)
        if self._probe_ros1(host, port):
            return "ros1"
        
        raise UnknownROSVersion(host)
```

---

## 5. Implementation Architecture

### 5.1 New Components for v0.6.1

```
skills/ (new package)
├── __init__.py
├── skill_model.py          # RobotSkill, RobotProfile dataclasses
├── skill_discovery.py      # Dynamic discovery from topology
├── skill_registry.py       # Central skill registry
├── skill_adapter.py        # Multi-ROS adaptation
├── skill_optimizer.py      # AI parameter optimization
└── skill_validator.py      # Safety validation

integration with existing:
├── discovery.py            → Uses skill_discovery
├── context.py              → Uses skill_registry for NL context
├── fleet/orchestrator.py   → Uses skill_registry for coordination
└── nl_interpreter.py       → Uses skill patterns for NL matching
```

### 5.2 Skill Discovery Integration

```python
class SkillDiscoveryIntegration:
    """Integrate skill discovery with existing discovery.py."""
    
    def __init__(self, topology: ROSTopology):
        self.topology = topology
        self.skill_patterns = self._load_skill_patterns()
    
    def discover_skills(self) -> List[RobotSkill]:
        """Discover skills from current topology."""
        skills = []
        
        for pattern in self.skill_patterns:
            if self._matches_pattern(pattern, self.topology):
                skill = self._create_skill_from_pattern(pattern)
                skills.append(skill)
        
        return skills
    
    def _load_skill_patterns(self) -> List[SkillPattern]:
        """Load skill definition patterns."""
        return [
            SkillPattern(
                skill_id="navigate",
                required_topics=["/cmd_vel", "/odom"],
                required_actions=["/navigate_to_pose"],
                optional_topics=["/amcl_pose", "/scan"],
            ),
            SkillPattern(
                skill_id="manipulate",
                required_topics=["/joint_states"],
                required_actions=["/move_action"],
                optional_topics=["/gripper_state"],
            ),
            # ... more patterns
        ]
```

### 5.3 AI Modification Interface

```python
class SkillModificationInterface:
    """Controlled interface for AI to modify skills."""
    
    def __init__(self, skill_registry: SkillRegistry, safety_validator: SafetyValidator):
        self.registry = skill_registry
        self.validator = safety_validator
    
    def propose_parameter_change(self, skill_id: str, param: str, value: Any) -> ProposalResult:
        """AI proposes a parameter change."""
        skill = self.registry.get_skill(skill_id)
        
        # Validate change
        validation = self.validator.validate_parameter_change(skill, param, value)
        
        if not validation.safe:
            return ProposalResult(
                approved=False,
                reason=validation.reason,
                suggestion=validation.suggestion,
            )
        
        # Apply change
        skill.parameters[param] = value
        skill.last_modified_by = "ai"
        skill.modification_history.append(ModificationRecord(param, value))
        
        return ProposalResult(approved=True)
    
    def propose_new_skill_composition(self, base_skills: List[str], new_skill_name: str) -> ProposalResult:
        """AI proposes combining skills (e.g., pick + place = pick_and_place)."""
        # Validate composition is feasible
        # Check for conflicts
        # Require human confirmation for new skill types
        pass
```

---

## 6. Safety and Governance

### 6.1 Safety Constraints

```python
class SkillSafetyValidator:
    """Validate skill modifications for safety."""
    
    SAFETY_LIMITS = {
        "max_speed": (0.0, 2.0),           # m/s
        "max_acceleration": (0.0, 1.0),    # m/s^2
        "max_force": (0.0, 100.0),         # N
        "goal_tolerance": (0.01, 1.0),     # m
    }
    
    def validate_parameter_change(self, skill: RobotSkill, param: str, value: Any) -> Validation:
        """Validate parameter change is safe."""
        
        # Check if parameter has safety limits
        if param in self.SAFETY_LIMITS:
            min_val, max_val = self.SAFETY_LIMITS[param]
            if not (min_val <= value <= max_val):
                return Validation(
                    safe=False,
                    reason=f"{param}={value} outside safe bounds [{min_val}, {max_val}]",
                    suggestion=max(min_val, min(value, max_val)),
                )
        
        # Check for dangerous combinations
        if self._is_dangerous_combination(skill, param, value):
            return Validation(
                safe=False,
                reason="Parameter combination creates safety risk",
                requires_human_confirmation=True,
            )
        
        return Validation(safe=True)
```

### 6.2 Governance Model

| Modification | Auto-Approve | Human Confirm | Logged | Rollback |
|--------------|--------------|---------------|--------|----------|
| Parameter tune (within bounds) | ✅ | ❌ | ✅ | ✅ |
| Add NL pattern | ✅ | ❌ | ✅ | ✅ |
| New skill composition | ❌ | ✅ | ✅ | ✅ |
| Change safety limits | ❌ | ✅ Required | ✅ | ✅ |
| Disable skill | ❌ | ✅ | ✅ | ✅ |
| Delete skill | ❌ | ✅ Required | ✅ | ✅ |

---

## 7. Benefits Summary

### 7.1 Without Dynamic Skills

```
Problems:
- ❌ AI must know ROS version of each robot
- ❌ Manual configuration for each robot type
- ❌ Cannot adapt to runtime changes (sensor added/removed)
- ❌ No learning from experience
- ❌ Fragile to topology changes
```

### 7.2 With Dynamic Skills

```
Benefits:
- ✅ AI sees unified capability interface
- ✅ Automatic adaptation to robot hardware
- ✅ Runtime discovery of new capabilities
- ✅ Learning and optimization over time
- ✅ Robust to topology changes
- ✅ Multi-ROS coordination transparent
- ✅ Self-documenting (skills describe what robot can do)
```

---

## 8. Recommendation

### 8.1 Yes, Implement Dynamic Skill System

**Rationale:**
1. **Essential for Multi-ROS:** Cannot practically coordinate ROS1 + ROS2 fleets without abstraction
2. **Enables AI Adaptation:** Static systems cannot learn or optimize
3. **Runtime Resilience:** Robots are dynamic — system must adapt
4. **Developer Experience:** Simplifies AI agent development

### 8.2 But With Careful Architecture

**Critical Design Principles:**

1. **Safety First:** AI cannot modify safety-critical parameters
2. **Human Oversight:** New skill types require confirmation
3. **Audit Trail:** All modifications logged
4. **Rollback:** Can revert to known-good configurations
5. **Validation:** All changes validated before application

### 8.3 Implementation Priority

| Phase | Component | Priority | Effort |
|-------|-----------|----------|--------|
| 1 | Skill model & discovery | P0 | Medium |
| 2 | Multi-ROS adapter | P0 | High |
| 3 | AI modification interface | P1 | Medium |
| 4 | Skill optimization | P2 | Medium |
| 5 | Advanced composition | P2 | High |

---

## 9. Conclusion

**The Answer: Yes, with safeguards.**

A dynamic, auto-discoverable, AI-modifiable skill system is **essential** for:
- Multi-ROS fleet coordination
- Runtime adaptation
- AI learning and optimization
- Robust operation

However, it must be implemented with:
- Strong safety validation
- Human oversight for critical changes
- Comprehensive audit logging
- Rollback capabilities

This system becomes the foundation for intelligent, adaptive robot fleets in v0.6.1.

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Recommendation Complete, Implementation Planned for v0.6.1
