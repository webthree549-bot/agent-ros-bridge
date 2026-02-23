"""Agent ROS Bridge - Core bridge for multi-agent ROS control.

A secure, multi-agent-capable bridge for ROS1/2 that enables AI agents
to control robots via multiple transport protocols.
"""

import asyncio
import logging
from typing import Any, Callable, Dict, List, Optional, Set
from dataclasses import dataclass, field
from datetime import datetime

logger = logging.getLogger(__name__)


@dataclass
class AgentSession:
    """Represents a connected agent session."""
    session_id: str
    agent_id: str
    connected_at: datetime
    permissions: Set[str] = field(default_factory=set)
    metadata: Dict[str, Any] = field(default_factory=dict)


class ActionRegistry:
    """Registry for ROS actions with async handlers."""
    
    def __init__(self):
        self._actions: Dict[str, Callable] = {}
        self._schemas: Dict[str, Dict[str, Any]] = {}
    
    def register(self, name: str, handler: Callable, schema: Optional[Dict] = None):
        """Register an action handler."""
        self._actions[name] = handler
        self._schemas[name] = schema or {"type": "object", "properties": {}}
        logger.info(f"Registered action: {name}")
    
    def get(self, name: str) -> Optional[Callable]:
        """Get action handler by name."""
        return self._actions.get(name)
    
    def list_actions(self) -> List[str]:
        """List all registered action names."""
        return list(self._actions.keys())
    
    def get_schema(self, name: str) -> Dict[str, Any]:
        """Get JSON schema for action parameters."""
        return self._schemas.get(name, {"type": "object"})
    
    def action(self, name: str, schema: Optional[Dict] = None):
        """Decorator to register an action."""
        def decorator(func: Callable):
            self.register(name, func, schema)
            return func
        return decorator


class TransportManager:
    """Manages multiple transport protocols (WebSocket, gRPC)."""
    
    def __init__(self):
        self._transports: List[Any] = []
        self._running = False
    
    def register(self, transport: Any):
        """Register a transport."""
        self._transports.append(transport)
        logger.info(f"Registered transport: {type(transport).__name__}")
    
    async def start(self):
        """Start all transports."""
        self._running = True
        tasks = []
        for transport in self._transports:
            if hasattr(transport, 'start'):
                tasks.append(asyncio.create_task(transport.start()))
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def stop(self):
        """Stop all transports."""
        self._running = False
        for transport in self._transports:
            if hasattr(transport, 'stop'):
                await transport.stop()
    
    def get_transports(self) -> List[Any]:
        """Get all registered transports."""
        return self._transports.copy()


class ConnectorManager:
    """Manages ROS1/2 connectors."""
    
    def __init__(self):
        self._connectors: List[Any] = []
        self._primary_connector: Optional[Any] = None
    
    def register(self, connector: Any, primary: bool = False):
        """Register a ROS connector."""
        self._connectors.append(connector)
        if primary or not self._primary_connector:
            self._primary_connector = connector
        logger.info(f"Registered connector: {type(connector).__name__}")
    
    async def start(self):
        """Start all connectors."""
        for connector in self._connectors:
            if hasattr(connector, 'start'):
                await connector.start()
    
    async def stop(self):
        """Stop all connectors."""
        for connector in self._connectors:
            if hasattr(connector, 'stop'):
                await connector.stop()
    
    def get_primary(self) -> Optional[Any]:
        """Get the primary connector."""
        return self._primary_connector
    
    def get_connectors(self) -> List[Any]:
        """Get all connectors."""
        return self._connectors.copy()


class TopicManager:
    """Manages ROS topic subscriptions and caching."""
    
    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = {}
        self._topic_cache: Dict[str, Any] = {}
        self._subscriptions: Dict[str, Any] = {}
    
    def register_topic(self, topic_name: str, msg_type: Any, callback: Optional[Callable] = None):
        """Register a topic for subscription."""
        if topic_name not in self._subscribers:
            self._subscribers[topic_name] = []
        if callback:
            self._subscribers[topic_name].append(callback)
    
    def update_topic_data(self, topic_name: str, data: Any):
        """Update cached topic data and notify subscribers."""
        self._topic_cache[topic_name] = {
            "data": data,
            "timestamp": datetime.utcnow().isoformat()
        }
        for callback in self._subscribers.get(topic_name, []):
            try:
                if asyncio.iscoroutinefunction(callback):
                    asyncio.create_task(callback(data))
                else:
                    callback(data)
            except Exception as e:
                logger.error(f"Error in topic callback for {topic_name}: {e}")
    
    def get_topic_data(self, topic_name: str) -> Optional[Dict]:
        """Get cached data for a topic."""
        return self._topic_cache.get(topic_name)
    
    def list_topics(self) -> List[str]:
        """List all registered topics."""
        return list(self._subscribers.keys())


class ROSBridge:
    """Main bridge class for multi-agent ROS control.
    
    Orchestrates transports, connectors, actions, and topics to enable
    secure, multi-agent robot control.
    """
    
    def __init__(self, ros_version: int = 2, config: Optional[Dict] = None):
        """Initialize ROS Bridge.
        
        Args:
            ros_version: ROS version (1 or 2)
            config: Optional configuration dictionary
        """
        self.ros_version = ros_version
        self.config = config or {}
        
        # Core components
        self.action_registry = ActionRegistry()
        self.transport_manager = TransportManager()
        self.connector_manager = ConnectorManager()
        self.topic_manager = TopicManager()
        
        # Session management
        self._sessions: Dict[str, AgentSession] = {}
        self._running = False
        
        logger.info(f"ROSBridge initialized (ROS{ros_version})")
    
    def action(self, name: str, schema: Optional[Dict] = None):
        """Decorator to register an action."""
        return self.action_registry.action(name, schema)
    
    async def call_action(self, name: str, **kwargs) -> Dict[str, Any]:
        """Execute a registered action.
        
        Args:
            name: Action name
            **kwargs: Action parameters
            
        Returns:
            Action result dictionary
        """
        handler = self.action_registry.get(name)
        if not handler:
            raise ValueError(f"Action '{name}' not found")
        
        try:
            if asyncio.iscoroutinefunction(handler):
                result = await handler(**kwargs)
            else:
                result = handler(**kwargs)
            
            return {
                "success": True,
                "action": name,
                "result": result,
                "timestamp": datetime.utcnow().isoformat()
            }
        except Exception as e:
            logger.error(f"Action '{name}' failed: {e}")
            return {
                "success": False,
                "action": name,
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat()
            }
    
    def get_registered_actions(self) -> List[str]:
        """List all registered action names."""
        return self.action_registry.list_actions()
    
    def get_available_topics(self) -> List[str]:
        """List all available ROS topics."""
        # Combine from topic manager and connector
        topics = set(self.topic_manager.list_topics())
        
        # Add topics from connector if available
        connector = self.connector_manager.get_primary()
        if connector and hasattr(connector, 'get_topics'):
            try:
                connector_topics = connector.get_topics()
                topics.update(connector_topics)
            except Exception as e:
                logger.warning(f"Could not get topics from connector: {e}")
        
        return sorted(list(topics))
    
    async def get_topic_data(self, topic: str) -> Dict[str, Any]:
        """Get current data for a topic."""
        # Check cache first
        cached = self.topic_manager.get_topic_data(topic)
        if cached:
            return cached
        
        # Try to fetch from connector
        connector = self.connector_manager.get_primary()
        if connector and hasattr(connector, 'get_topic_data'):
            try:
                data = await connector.get_topic_data(topic)
                return {"data": data, "timestamp": datetime.utcnow().isoformat()}
            except Exception as e:
                logger.error(f"Failed to get topic data for {topic}: {e}")
        
        return {"error": f"Topic '{topic}' not available"}
    
    def create_session(self, agent_id: str, permissions: Optional[Set[str]] = None) -> AgentSession:
        """Create a new agent session."""
        import uuid
        session = AgentSession(
            session_id=str(uuid.uuid4()),
            agent_id=agent_id,
            connected_at=datetime.utcnow(),
            permissions=permissions or {"read", "execute"}
        )
        self._sessions[session.session_id] = session
        logger.info(f"Created session {session.session_id} for agent {agent_id}")
        return session
    
    def get_session(self, session_id: str) -> Optional[AgentSession]:
        """Get session by ID."""
        return self._sessions.get(session_id)
    
    def close_session(self, session_id: str):
        """Close a session."""
        if session_id in self._sessions:
            del self._sessions[session_id]
            logger.info(f"Closed session {session_id}")
    
    async def start(self):
        """Start the bridge."""
        logger.info("Starting ROS Bridge...")
        self._running = True
        
        # Start connectors first
        await self.connector_manager.start()
        
        # Then start transports
        await self.transport_manager.start()
        
        logger.info("ROS Bridge started")
        
        # Keep running
        while self._running:
            await asyncio.sleep(1)
    
    async def stop(self):
        """Stop the bridge."""
        logger.info("Stopping ROS Bridge...")
        self._running = False
        
        await self.transport_manager.stop()
        await self.connector_manager.stop()
        
        logger.info("ROS Bridge stopped")


try:
    from .config import BridgeConfig, load_config
except ImportError:
    BridgeConfig = None
    load_config = None

try:
    from .openclaw import OpenClawIntegration, OpenClawSkill, connect_to_openclaw
except ImportError:
    OpenClawIntegration = None
    OpenClawSkill = None
    connect_to_openclaw = None

try:
    from .memory import AgentMemory, create_memory
except ImportError:
    AgentMemory = None
    create_memory = None

try:
    from .discovery import ToolDiscovery, DiscoveredTool, SafetyLevel, discover_tools
except ImportError:
    ToolDiscovery = None
    DiscoveredTool = None
    SafetyLevel = None
    discover_tools = None

try:
    from .safety import (
        Confirmation, ConfirmationRequest, ConfirmationStatus,
        ActionSafety, SafetyLevel as SafetyLevelSafety,
        ConfirmationRejected, ConfirmationTimeout, ConfirmationCancelled,
        safety_level, confirm_dangerous
    )
except ImportError:
    Confirmation = None
    ConfirmationRequest = None
    ConfirmationStatus = None
    ActionSafety = None
    SafetyLevelSafety = None
    ConfirmationRejected = None
    ConfirmationTimeout = None
    ConfirmationCancelled = None
    safety_level = None
    confirm_dangerous = None

try:
    from .metrics import MetricsCollector, HealthChecker, create_metrics, start_metrics_server
except ImportError:
    MetricsCollector = None
    HealthChecker = None
    create_metrics = None
    start_metrics_server = None

try:
    from .tracing import TracingManager, TraceConfig, create_tracing
except ImportError:
    TracingManager = None
    TraceConfig = None
    create_tracing = None

try:
    from .dashboard import DashboardServer
except ImportError:
    DashboardServer = None

try:
    from .langchain import ROSBridgeTool, ROSAgent, create_ros_tool
except ImportError:
    ROSBridgeTool = None
    ROSAgent = None
    create_ros_tool = None

try:
    from .autogpt import AutoGPTPlugin, AutoGPTBridge
except ImportError:
    AutoGPTPlugin = None
    AutoGPTBridge = None

try:
    from .actions import ROS2ActionClient, Navigation2Client, ActionGoal, ActionResult, ActionStatus
except ImportError:
    ROS2ActionClient = None
    Navigation2Client = None
    ActionGoal = None
    ActionResult = None
    ActionStatus = None

__all__ = [
    # Core
    "ROSBridge",
    "AgentSession",
    "ActionRegistry",
    "TransportManager",
    "ConnectorManager",
    "TopicManager",
    # Config
    "BridgeConfig",
    "load_config",
    # OpenClaw
    "OpenClawIntegration",
    "OpenClawSkill",
    "connect_to_openclaw",
    # Phase 2: Memory
    "AgentMemory",
    "create_memory",
    # Phase 2: Discovery
    "ToolDiscovery",
    "DiscoveredTool",
    "SafetyLevel",
    "discover_tools",
    # Phase 2: Safety
    "Confirmation",
    "ConfirmationRequest",
    "ConfirmationStatus",
    "ActionSafety",
    "ConfirmationRejected",
    "ConfirmationTimeout",
    "ConfirmationCancelled",
    "safety_level",
    "confirm_dangerous",
    # Phase 3: Observability
    "MetricsCollector",
    "HealthChecker",
    "create_metrics",
    "start_metrics_server",
    "TracingManager",
    "TraceConfig",
    "create_tracing",
    "DashboardServer",
    # Phase 4: Ecosystem
    "ROSBridgeTool",
    "ROSAgent",
    "create_ros_tool",
    "AutoGPTPlugin",
    "AutoGPTBridge",
    "ROS2ActionClient",
    "Navigation2Client",
    "ActionGoal",
    "ActionResult",
    "ActionStatus",
]
