# ROS System Topology Retrieval for NL Context

## Overview

This document analyzes how the Agent ROS Bridge retrieves and maintains ROS system topology (topics, services, actions, nodes, parameters) as contextual information for natural language understanding and execution.

---

## 1. What is ROS System Topology?

### 1.1 Topology Components

```
ROS System Topology
├── Computation Graph
│   ├── Nodes (processes)
│   ├── Topics (data streams)
│   ├── Services (RPC endpoints)
│   ├── Actions (long-running tasks)
│   └── Parameters (configuration)
│
├── Physical Topology
│   ├── Robot hardware (sensors, actuators)
│   ├── Network topology (DDS domains)
│   └── TF tree (coordinate frames)
│
├── Semantic Layer
│   ├── Message types (schemas)
│   ├── Interfaces (API contracts)
│   └── Capabilities (what the robot can do)
│
└── Runtime State
    ├── Active publishers/subscribers
    ├── Service availability
    ├── Node health
    └── Current parameter values
```

### 1.2 Why Topology Matters for NL

**Example:** User says "Show me what you see"

Without topology context:
- ❌ Don't know which camera topics exist
- ❌ Don't know message types (Image, CompressedImage, PointCloud)
- ❌ Can't generate correct subscription code
- ❌ Fail at runtime

With topology context:
- ✅ Discover `/camera/image_raw` (sensor_msgs/Image)
- ✅ Also find `/camera/depth` (sensor_msgs/PointCloud2)
- ✅ Generate appropriate code
- ✅ Execute successfully

---

## 2. Current Implementation Analysis

### 2.1 Discovery System (`discovery.py`)

**Current Capabilities:**

```python
class ToolDiscovery:
    def discover_all(self) -> List[ROSAction]:
        """Discover all available ROS tools."""
        tools = []
        tools.extend(self._discover_topics())      # ✅ Implemented
        tools.extend(self._discover_services())    # ✅ Implemented
        tools.extend(self._discover_actions())     # 🟡 Partial
        return tools
```

**ROS2 Discovery (via rclpy):**

```python
def _discover_ros2_topics(self, robot) -> List[ROSAction]:
    """Uses rclpy introspection APIs."""
    topic_names_and_types = robot.ros_node.get_topic_names_and_types()
    # Returns: [("/cmd_vel", ["geometry_msgs/Twist"]), ...]
    
def _discover_ros2_services(self, robot) -> List[ROSAction]:
    """Uses rclpy introspection APIs."""
    service_names_and_types = robot.ros_node.get_service_names_and_types()
    # Returns: [("/global_localization", ["std_srvs/Trigger"]), ...]
```

**Strengths:**
- ✅ Real-time discovery via ROS2 APIs
- ✅ Works with both ROS1 and ROS2
- ✅ Caches discovered tools
- ✅ Exports to AI formats (MCP, OpenAI)

**Limitations:**
- 🟡 No node relationship tracking
- 🟡 No parameter discovery
- 🟡 No TF tree introspection
- 🟡 Static snapshot (no change detection)
- 🟡 No semantic understanding (just names/types)

### 2.2 ROS2 Connector (`ros2_connector.py`)

**Introspection Methods:**

```python
class ROS2Robot:
    def _cmd_get_topics(self) -> List[Dict]:
        """Get available topics."""
        return self.ros_node.get_topic_names_and_types()
    
    def _cmd_get_nodes(self) -> List[str]:
        """Get running nodes."""
        return self.ros_node.get_node_names()
    
    def _get_topic_message_type(self, topic: str) -> Optional[str]:
        """Auto-detect message type for a topic."""
        # Used for dynamic message creation
```

**Runtime Integration:**
- ✅ Auto-detects message types for publishing
- ✅ Creates publishers/subscribers dynamically
- ✅ Works with 50+ message types via registry

---

## 3. Topology Retrieval Architecture

### 3.1 Multi-Layer Topology Model

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 4: Semantic Topology (AI-Optimized)                  │
│  - "Camera" = /camera/image_raw (Image) + /camera/info      │
│  - "Navigation" = /cmd_vel + /odom + /amcl_pose             │
│  - "Manipulation" = /joint_states + /gripper_cmd            │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│  LAYER 3: Functional Topology (Robot Capabilities)          │
│  - Navigation capability → /navigate_to_pose action         │
│  - Perception capability → /detect_objects service          │
│  - Manipulation capability → /move_group action             │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│  LAYER 2: ROS Graph Topology (Computation Graph)            │
│  - Nodes: /turtlebot3_diff_drive, /amcl, /nav2_controller   │
│  - Topics: /cmd_vel, /odom, /scan, /map                     │
│  - Services: /global_localization, /clear_costmap           │
│  - Actions: /navigate_to_pose, /follow_waypoints            │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: Physical Topology (Hardware & Network)            │
│  - TF Tree: map → odom → base_link → camera_link            │
│  - DDS: Domain ID, partitions, QoS profiles                 │
│  - Hardware: /dev/video0, /dev/ttyACM0 (serial)             │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Topology Retrieval Methods

#### Method 1: ROS2 Introspection APIs (Primary)

```python
class ROSTopologyRetriever:
    """Retrieve topology using ROS2 native APIs."""
    
    def get_full_topology(self, node: rclpy.Node) -> ROSTopology:
        """Get complete system topology."""
        return ROSTopology(
            topics=self._get_topics(node),
            services=self._get_services(node),
            actions=self._get_actions(node),
            nodes=self._get_nodes(node),
            parameters=self._get_parameters(node),
        )
    
    def _get_topics(self, node) -> List[TopicInfo]:
        """Get topic graph."""
        topics = []
        names_and_types = node.get_topic_names_and_types()
        
        for name, types in names_and_types:
            # Get publishers/subscribers for each topic
            publishers = node.get_publishers_info_by_topic(name)
            subscribers = node.get_subscriptions_info_by_topic(name)
            
            topics.append(TopicInfo(
                name=name,
                msg_type=types[0] if types else "unknown",
                publishers=[p.node_name for p in publishers],
                subscribers=[s.node_name for s in subscribers],
                qos=self._analyze_qos(publishers, subscribers),
            ))
        
        return topics
    
    def _get_services(self, node) -> List[ServiceInfo]:
        """Get service graph."""
        services = []
        names_and_types = node.get_service_names_and_types()
        
        for name, types in names_and_types:
            services.append(ServiceInfo(
                name=name,
                srv_type=types[0] if types else "unknown",
                # Note: ROS2 doesn't expose service clients easily
            ))
        
        return services
```

#### Method 2: DDS Discovery (Network Level)

```python
class DDSTopologyRetriever:
    """Retrieve topology via DDS discovery (lower level)."""
    
    def __init__(self, domain_id: int = 0):
        self.domain_id = domain_id
    
    def get_participants(self) -> List[DDSParticipant]:
        """Get DDS participants (roughly = ROS nodes)."""
        # Use DDS API or ros2 doctor
        participants = self._query_dds_discovery()
        return participants
    
    def get_topic_qos(self, topic_name: str) -> QoSProfile:
        """Get actual QoS in use on the network."""
        # Critical for compatibility
        pass
```

#### Method 3: Static Configuration (Fallback)

```python
class StaticTopologyLoader:
    """Load topology from configuration files."""
    
    def load_from_config(self, config_path: str) -> ROSTopology:
        """Load expected topology from YAML."""
        # Used when:
        # - Runtime discovery fails
        # - Need "known good" baseline
        # - Security (don't discover unknown nodes)
        pass
```

### 3.3 Topology Change Detection

**The Problem:** ROS topology is dynamic. Nodes start/stop. Topics appear/disappear.

**Solution:** Continuous monitoring with change detection

```python
class TopologyMonitor:
    """Monitor topology changes over time."""
    
    def __init__(self, retriever: ROSTopologyRetriever):
        self.retriever = retriever
        self.current_topology: Optional[ROSTopology] = None
        self.change_callbacks: List[Callable] = []
    
    async def start_monitoring(self, interval: float = 5.0):
        """Start periodic topology checks."""
        while True:
            new_topology = self.retriever.get_full_topology()
            
            if self.current_topology:
                changes = self._detect_changes(
                    self.current_topology, 
                    new_topology
                )
                
                if changes:
                    self._notify_change(changes)
            
            self.current_topology = new_topology
            await asyncio.sleep(interval)
    
    def _detect_changes(self, old: ROSTopology, new: ROSTopology) -> TopologyChanges:
        """Detect what changed between snapshots."""
        return TopologyChanges(
            topics_added=[t for t in new.topics if t not in old.topics],
            topics_removed=[t for t in old.topics if t not in new.topics],
            nodes_added=[n for n in new.nodes if n not in old.nodes],
            nodes_removed=[n for n in old.nodes if n not in new.nodes],
            # ... etc
        )
```

---

## 4. Context Integration for NL

### 4.1 From Topology to NL Context

**Raw Topology:**
```
Topic: /camera/image_raw
Type: sensor_msgs/Image
Publishers: [/camera_driver]
Subscribers: [/image_view, /object_detector]
```

**NL Context:**
```python
{
    "camera": {
        "available": True,
        "topic": "/camera/image_raw",
        "type": "Image",
        "resolution": "640x480",  # from topic introspection
        "frame_rate": 30,         # from topic introspection
        "capabilities": ["see", "detect_objects", "recognize_faces"],
        "current_viewers": ["image_view", "object_detector"]
    }
}
```

### 4.2 Semantic Enrichment

**Adding Meaning to Raw Topology:**

```python
class TopologySemanticEnricher:
    """Enrich raw topology with semantic meaning."""
    
    def enrich(self, topology: ROSTopology) -> SemanticTopology:
        """Add semantic layer to raw topology."""
        
        # Pattern matching for common robot components
        components = {
            "camera": self._find_camera_topics(topology),
            "lidar": self._find_lidar_topics(topology),
            "odometry": self._find_odom_topics(topology),
            "navigation": self._find_nav_topics(topology),
            "manipulation": self._find_manip_topics(topology),
        }
        
        # Infer capabilities from components
        capabilities = self._infer_capabilities(components)
        
        # Build natural language descriptions
        nl_descriptions = self._generate_nl_descriptions(components)
        
        return SemanticTopology(
            raw=topology,
            components=components,
            capabilities=capabilities,
            nl_descriptions=nl_descriptions,
        )
    
    def _find_camera_topics(self, topology: ROSTopology) -> List[CameraInfo]:
        """Find camera-related topics by pattern matching."""
        cameras = []
        
        for topic in topology.topics:
            # Pattern: contains "camera", "image", "rgb"
            if any(pattern in topic.name for pattern in 
                   ["camera", "image", "rgb", "color"]):
                
                if "Image" in topic.msg_type:
                    cameras.append(CameraInfo(
                        name=self._extract_camera_name(topic.name),
                        image_topic=topic.name,
                        info_topic=topic.name.replace("image", "camera_info"),
                    ))
        
        return cameras
    
    def _infer_capabilities(self, components: Dict) -> List[Capability]:
        """Infer what the robot can do from components."""
        capabilities = []
        
        if components["camera"] and components["lidar"]:
            capabilities.append(Capability(
                name="navigate",
                description="Navigate autonomously using camera and lidar",
                required_topics=["/cmd_vel", "/odom"],
                required_actions=["/navigate_to_pose"],
            ))
        
        if components["manipulation"]:
            capabilities.append(Capability(
                name="pick_and_place",
                description="Pick up and place objects",
                required_topics=["/joint_states"],
                required_actions=["/move_action"],
            ))
        
        return capabilities
```

### 4.3 Context for NL Understanding

**Example Integration:**

```python
class NLContextResolver:
    """Resolve NL ambiguities using topology context."""
    
    def __init__(self, semantic_topology: SemanticTopology):
        self.topology = semantic_topology
    
    def resolve_reference(self, nl_phrase: str) -> Optional[ROSTopic]:
        """Resolve "the camera" to specific topic."""
        
        # Direct match
        if nl_phrase == "camera":
            if self.topology.components["camera"]:
                return self.topology.components["camera"][0].image_topic
        
        # Fuzzy match
        if "camera" in nl_phrase or "see" in nl_phrase:
            # Find best matching camera
            return self._find_best_camera_match(nl_phrase)
        
        # Context-dependent
        if nl_phrase == "it":
            return self._resolve_anaphora(nl_phrase)
        
        return None
    
    def validate_command(self, intent: Intent, entities: List[Entity]) -> Validation:
        """Check if command is possible given topology."""
        
        # Check if required topics exist
        required_topics = self._get_required_topics(intent)
        missing = [t for t in required_topics 
                   if t not in self.topology.raw.topic_names]
        
        if missing:
            return Validation(
                valid=False,
                reason=f"Missing required topics: {missing}",
                suggestion="Ensure navigation stack is running",
            )
        
        # Check if required capabilities exist
        required_capability = self._get_required_capability(intent)
        if required_capability not in self.topology.capabilities:
            return Validation(
                valid=False,
                reason=f"Robot doesn't have {required_capability} capability",
                suggestion="Check hardware configuration",
            )
        
        return Validation(valid=True)
```

---

## 5. Implementation for v0.6.1

### 5.1 Architecture Changes

```
current:                    v0.6.1:
discovery.py                topology/
├── ToolDiscovery           ├── TopologyRetriever (new)
└── ROSAction               ├── TopologyMonitor (new)
                            ├── TopologySemanticEnricher (new)
                            └── TopologyContext (integration)
```

### 5.2 New Components

#### Component 1: TopologyRetriever

```python
class TopologyRetriever:
    """Unified topology retrieval for ROS1/ROS2."""
    
    def __init__(self, ros_version: str, node: Node):
        self.ros_version = ros_version
        self.node = node
    
    def get_topology(self) -> ROSTopology:
        """Get complete topology snapshot."""
        return ROSTopology(
            topics=self._get_topics(),
            services=self._get_services(),
            actions=self._get_actions(),
            nodes=self._get_nodes(),
            parameters=self._get_parameters(),
            tf_tree=self._get_tf_tree(),
        )
```

#### Component 2: TopologyContext

```python
class TopologyContext:
    """Topology-aware context for NL understanding."""
    
    def __init__(self, retriever: TopologyRetriever):
        self.retriever = retriever
        self.semantic = None
        self._refresh_topology()
    
    def _refresh_topology(self):
        """Update topology and semantic enrichment."""
        raw = self.retriever.get_topology()
        self.semantic = TopologySemanticEnricher().enrich(raw)
    
    def resolve(self, nl_reference: str) -> Optional[ROSTopic]:
        """Resolve NL reference to ROS entity."""
        return self.semantic.resolve_reference(nl_reference)
    
    def validate(self, command: Command) -> Validation:
        """Validate command against available topology."""
        return self.semantic.validate_command(command)
```

### 5.3 Integration Points

**Integration with Existing Modules:**

| Existing Module | Integration | Purpose |
|-----------------|-------------|---------|
| `discovery.py` | Extend with semantic layer | Export enriched topology to AI |
| `context.py` | Add topology context | NL reference resolution |
| `nl_interpreter.py` | Use topology for validation | Check command feasibility |
| `ros2_connector.py` | Provide node for introspection | Raw topology retrieval |
| `fleet/orchestrator.py` | Aggregate fleet topology | Multi-robot context |

### 5.4 TDD Test Specification

```python
# tests/unit/test_topology_retrieval_tdd.py

def test_topic_discovery_returns_all_topics():
    """Should discover all active topics."""
    retriever = TopologyRetriever(node)
    topology = retriever.get_topology()
    assert len(topology.topics) > 0

def test_semantic_enrichment_finds_camera():
    """Should identify camera from topic patterns."""
    raw = ROSTopology(topics=[TopicInfo("/camera/image_raw", "sensor_msgs/Image")])
    enriched = TopologySemanticEnricher().enrich(raw)
    assert "camera" in enriched.components

def test_nl_reference_resolution():
    """Should resolve 'the camera' to topic."""
    context = TopologyContext(retriever)
    topic = context.resolve("the camera")
    assert topic == "/camera/image_raw"

def test_command_validation_missing_topic():
    """Should reject command if required topic missing."""
    context = TopologyContext(retriever)
    validation = context.validate(navigate_command)
    assert not validation.valid
    assert "Missing required topics" in validation.reason

def test_topology_change_detection():
    """Should detect when new node appears."""
    monitor = TopologyMonitor(retriever)
    changes = monitor._detect_changes(old_topology, new_topology)
    assert len(changes.nodes_added) == 1
```

---

## 6. Usage Examples

### 6.1 NL Understanding with Topology

```python
# Without topology context
user: "Show me what you see"
system: ❌ "Error: No camera topic specified"

# With topology context
user: "Show me what you see"
system: ✅ Discovers /camera/image_raw
       ✅ Generates subscription code
       ✅ Displays image stream
```

### 6.2 Command Validation

```python
# Without topology validation
user: "Navigate to the kitchen"
system: ❌ Generates Nav2 code
       ❌ Fails at runtime (nav2 not running)

# With topology validation
user: "Navigate to the kitchen"
system: ✅ Checks: /navigate_to_pose action exists? YES
       ✅ Checks: /amcl_pose topic publishing? YES
       ✅ Generates code
       ✅ Executes successfully
```

### 6.3 Ambiguity Resolution

```python
# Ambiguous command
user: "Use the camera"

# Without topology: "Which camera?"
# With topology:
system: ✅ Found 2 cameras:
       - /front_camera (active, 30fps)
       - /rear_camera (standby)
       ✅ Selects front_camera (active)
       ✅ "Using front camera"
```

---

## 7. Summary

### 7.1 Key Insights

1. **Topology is Essential for NL**
   - Raw topic names are meaningless to NL
   - Semantic enrichment bridges gap
   - Enables validation before execution

2. **Dynamic Discovery Required**
   - ROS topology changes at runtime
   - Continuous monitoring needed
   - Change detection for adaptation

3. **Multi-Layer Approach**
   - Layer 1: Physical (DDS, hardware)
   - Layer 2: ROS graph (nodes, topics)
   - Layer 3: Functional (capabilities)
   - Layer 4: Semantic (NL-meaningful)

4. **Integration Strategy**
   - Extend existing `discovery.py`
   - Integrate with `context.py`
   - Use `ros2_connector.py` for raw access
   - Enrich for NL understanding

### 7.2 Implementation Priority

| Priority | Component | Effort | Impact |
|----------|-----------|--------|--------|
| P0 | TopologyRetriever | Medium | Critical |
| P0 | SemanticEnricher | High | Critical |
| P1 | TopologyMonitor | Medium | High |
| P1 | TopologyContext | Medium | High |
| P2 | DDS-level discovery | High | Medium |
| P2 | Static config loader | Low | Low |

### 7.3 Success Criteria

- ✅ Discover 100% of active ROS entities
- ✅ Enrich with >90% accuracy
- ✅ Resolve NL references >85% accuracy
- ✅ Validate commands before execution
- ✅ Detect topology changes <5s latency

---

**Document Version:** 1.0  
**Last Updated:** 2026-03-06  
**Status:** Architecture Complete, Implementation Planned for v0.6.1
