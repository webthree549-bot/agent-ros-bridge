#!/usr/bin/env python3
"""OpenClaw Gateway - Universal Robot Gateway.

The next-generation architecture for robot-AI connectivity.
Multi-protocol, multi-robot, cloud-native.

v0.5.0: Integrated AI Agent Support
- Agent Memory (SQLite/Redis)
- Safety Manager (confirmation, emergency stop)
- Tool Discovery (MCP/OpenAI export)
- LangChain/AutoGPT/MCP integrations
"""

from __future__ import annotations

import asyncio
import logging
import uuid
from abc import ABC, abstractmethod
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any, AsyncIterator, Callable

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")

# v0.5.0: Import AI integrations
try:
    from ..integrations.discovery import ToolDiscovery
    from ..integrations.memory import AgentMemory
    from ..integrations.safety import SafetyLevel, SafetyManager

    INTEGRATIONS_AVAILABLE = True
except ImportError:
    INTEGRATIONS_AVAILABLE = False
    logger.warning("AI integrations not available")


# =============================================================================
# Core Data Models
# =============================================================================


class QoS(Enum):
    """Quality of Service levels."""

    BEST_EFFORT = auto()  # Fire and forget
    AT_LEAST_ONCE = auto()  # Retry until ack
    EXACTLY_ONCE = auto()  # Deduplication guaranteed


@dataclass
class Identity:
    """Authenticated identity."""

    id: str
    name: str
    roles: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Header:
    """Message header."""

    message_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = field(default_factory=datetime.utcnow)
    source: str = ""
    target: str = ""
    correlation_id: str | None = None


@dataclass
class Command:
    """Robot command."""

    action: str
    parameters: dict[str, Any] = field(default_factory=dict)
    timeout_ms: int = 5000
    priority: int = 5  # 1-10, lower = higher priority


@dataclass
class Telemetry:
    """Sensor/telemetry data."""

    topic: str
    data: Any
    quality: float = 1.0  # Data quality 0-1


@dataclass
class Event:
    """System event."""

    event_type: str
    severity: str = "info"  # debug, info, warning, error, critical
    data: dict[str, Any] = field(default_factory=dict)


@dataclass
class Message:
    """Unified message format."""

    header: Header = field(default_factory=Header)
    command: Command | None = None
    telemetry: Telemetry | None = None
    event: Event | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
    identity: Identity | None = None


# =============================================================================
# Transport Layer Abstraction
# =============================================================================


class Transport(ABC):
    """Abstract transport layer."""

    def __init__(self, name: str, config: dict[str, Any]):
        self.name = name
        self.config = config
        self.running = False
        self.message_handler: Callable[[Message, Identity], asyncio.Future] | None = None

    @abstractmethod
    async def start(self) -> bool:
        """Start the transport."""
        pass

    @abstractmethod
    async def stop(self) -> None:
        """Stop the transport."""
        pass

    @abstractmethod
    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific recipient."""
        pass

    @abstractmethod
    async def broadcast(self, message: Message) -> list[str]:
        """Broadcast to all connected clients."""
        pass

    def on_message(self, handler: Callable[[Message, Identity], asyncio.Future]):
        """Register message handler."""
        self.message_handler = handler


class TransportManager:
    """Manages multiple transport endpoints."""

    def __init__(self):
        self.transports: dict[str, Transport] = {}
        self._message_handler: Callable | None = None

    def register(self, transport: Transport) -> None:
        """Register a transport."""
        self.transports[transport.name] = transport
        transport.on_message(self._route_message)
        logger.info(f"Registered transport: {transport.name}")

    def _route_message(self, message: Message, identity: Identity):
        """Route incoming message to handler — schedules the coroutine on the running loop."""
        if self._message_handler:
            asyncio.ensure_future(self._message_handler(message, identity))

    def on_message(self, handler: Callable[[Message, Identity], asyncio.Future]):
        """Set global message handler."""
        self._message_handler = handler

    async def start_all(self) -> None:
        """Start all registered transports."""
        await asyncio.gather(*[t.start() for t in self.transports.values()])

    async def stop_all(self) -> None:
        """Stop all transports."""
        await asyncio.gather(*[t.stop() for t in self.transports.values()])

    async def send(self, transport_name: str, message: Message, recipient: str) -> bool:
        """Send via specific transport."""
        if transport_name not in self.transports:
            raise ValueError(f"Unknown transport: {transport_name}")
        return await self.transports[transport_name].send(message, recipient)


# =============================================================================
# Connector Layer Abstraction
# =============================================================================


class Robot(ABC):
    """Abstract robot representation."""

    def __init__(self, robot_id: str, name: str, connector_type: str):
        self.robot_id = robot_id
        self.name = name
        self.connector_type = connector_type
        self.connected = False
        self.capabilities: set[str] = set()
        self.metadata: dict[str, Any] = {}
        self._subscribers: dict[str, list[Callable]] = {}

    @abstractmethod
    async def connect(self) -> bool:
        """Connect to robot."""
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from robot."""
        pass

    @abstractmethod
    async def execute(self, command: Command) -> Any:
        """Execute command on robot."""
        pass

    @abstractmethod
    async def subscribe(self, topic: str) -> AsyncIterator[Telemetry]:
        """Subscribe to telemetry topic."""
        pass

    def _notify_subscribers(self, topic: str, data: Telemetry):
        """Notify subscribers of new data."""
        for callback in self._subscribers.get(topic, []):
            try:
                callback(data)
            except Exception as e:
                logger.error(f"Subscriber error: {e}")


class Connector(ABC):
    """Abstract robot connector."""

    connector_type: str = "abstract"

    @abstractmethod
    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to robot at URI."""
        pass

    @abstractmethod
    async def discover(self) -> list[RobotEndpoint]:
        """Discover available robots."""
        pass


@dataclass
class RobotEndpoint:
    """Discovered robot endpoint."""

    uri: str
    name: str
    connector_type: str
    capabilities: list[str]
    metadata: dict[str, Any]


class ConnectorRegistry:
    """Registry of robot connectors."""

    def __init__(self):
        self.connectors: dict[str, Connector] = {}

    def register(self, connector: Connector) -> None:
        """Register a connector."""
        self.connectors[connector.connector_type] = connector
        logger.info(f"Registered connector: {connector.connector_type}")

    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to robot using appropriate connector."""
        # Parse URI to determine connector type
        # e.g., ros2://192.168.1.100:7411
        scheme = uri.split("://")[0]

        if scheme not in self.connectors:
            raise ValueError(f"No connector for scheme: {scheme}")

        return await self.connectors[scheme].connect(uri, **kwargs)

    async def discover_all(self) -> list[RobotEndpoint]:
        """Discover robots using all connectors."""
        results = []
        for connector in self.connectors.values():
            try:
                robots = await connector.discover()
                results.extend(robots)
            except Exception as e:
                logger.warning(f"Discovery failed for {connector.connector_type}: {e}")
        return results


# =============================================================================
# Orchestration Layer
# =============================================================================


class RobotFleet:
    """Manages a fleet of robots."""

    def __init__(self, name: str):
        self.name = name
        self.robots: dict[str, Robot] = {}
        self.groups: dict[str, list[str]] = {}

    def add_robot(self, robot: Robot) -> None:
        """Add robot to fleet."""
        self.robots[robot.robot_id] = robot
        logger.info(f"Added robot {robot.name} to fleet {self.name}")

    def remove_robot(self, robot_id: str) -> None:
        """Remove robot from fleet."""
        if robot_id in self.robots:
            del self.robots[robot_id]
            logger.info(f"Removed robot {robot_id} from fleet {self.name}")

    def get_robot(self, robot_id: str) -> Robot | None:
        """Get robot by ID."""
        return self.robots.get(robot_id)

    def select(self, predicate: Callable[[Robot], bool]) -> list[Robot]:
        """Select robots matching predicate."""
        return [r for r in self.robots.values() if predicate(r)]

    async def broadcast(
        self, command: Command, selector: Callable[[Robot], bool] | None = None
    ) -> dict[str, Any]:
        """Broadcast command to selected robots."""
        targets = self.select(selector) if selector else list(self.robots.values())

        async def exec_with_id(robot: Robot):
            try:
                result = await asyncio.wait_for(
                    robot.execute(command), timeout=command.timeout_ms / 1000
                )
                return robot.robot_id, {"status": "success", "result": result}
            except asyncio.TimeoutError:
                return robot.robot_id, {"status": "timeout"}
            except Exception as e:
                return robot.robot_id, {"status": "error", "error": str(e)}

        results = await asyncio.gather(*[exec_with_id(r) for r in targets])
        return dict(results)


class Plugin(ABC):
    """Abstract plugin."""

    name: str = "abstract_plugin"
    version: str = "0.0.1"

    @abstractmethod
    async def initialize(self, gateway: Bridge) -> None:
        """Initialize plugin with gateway reference."""
        pass

    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown plugin."""
        pass

    async def handle_message(self, message: Message, identity: Identity) -> Message | None:
        """Handle message - return response or None."""
        return None


class PluginManager:
    """Manages plugins with hot reload."""

    def __init__(self):
        self.plugins: dict[str, Plugin] = {}
        self.gateway: Bridge | None = None

    def set_gateway(self, gateway: Bridge) -> None:
        """Set gateway reference."""
        self.gateway = gateway

    async def load_plugin(self, plugin: Plugin) -> None:
        """Load and initialize a plugin."""
        if self.gateway is None:
            raise RuntimeError("Gateway not set")

        self.plugins[plugin.name] = plugin
        await plugin.initialize(self.gateway)
        logger.info(f"Loaded plugin: {plugin.name} v{plugin.version}")

    async def unload_plugin(self, name: str) -> None:
        """Unload a plugin."""
        if name in self.plugins:
            await self.plugins[name].shutdown()
            del self.plugins[name]
            logger.info(f"Unloaded plugin: {name}")

    async def handle_message(self, message: Message, identity: Identity) -> Message | None:
        """Route message through plugins."""
        for plugin in self.plugins.values():
            try:
                response = await plugin.handle_message(message, identity)
                if response is not None:
                    return response
            except Exception as e:
                logger.error(f"Plugin {plugin.name} error: {e}")
        return None


# =============================================================================
# Main Gateway
# =============================================================================


class Bridge:
    """Universal robot gateway - main entry point.

    v0.5.0: Added AI agent integrations
    """

    def __init__(self, config: dict[str, Any] | None = None):
        self.config = config or {}
        self.transport_manager = TransportManager()
        self.connector_registry = ConnectorRegistry()
        self.fleets: dict[str, RobotFleet] = {}
        self.plugin_manager = PluginManager()
        self.plugin_manager.set_gateway(self)
        self.running = False

        # v0.5.0: Initialize AI integrations
        self.memory: AgentMemory | None = None
        self.safety: SafetyManager | None = None
        self.tools: ToolDiscovery | None = None

        if INTEGRATIONS_AVAILABLE:
            self._init_integrations()

        # Set up message routing
        self.transport_manager.on_message(self._handle_message)

    def _init_integrations(self):
        """Initialize v0.5.0 AI integrations."""
        try:
            # Agent Memory (SQLite default, Redis optional)
            memory_backend = self.config.get("memory_backend", "sqlite")
            memory_path = self.config.get("memory_path", ":memory:")
            self.memory = AgentMemory(backend=memory_backend, path=memory_path)
            logger.info(f"AgentMemory initialized ({memory_backend})")

            # Safety Manager
            self.safety = SafetyManager()
            # Register default safety policies
            self.safety.register_policy(
                "emergency_stop", SafetyLevel.DANGEROUS, "Emergency stop - requires confirmation"
            )
            self.safety.register_policy(
                "move_arm", SafetyLevel.DANGEROUS, "Arm movement - requires confirmation"
            )
            logger.info("SafetyManager initialized")

            # Tool Discovery
            self.tools = ToolDiscovery(self)
            logger.info("ToolDiscovery initialized")

        except Exception as e:
            logger.error(f"Failed to initialize integrations: {e}")
            # Continue without integrations - core functionality still works

    async def _handle_message(self, message: Message, identity: Identity) -> None:
        """Handle incoming message from transport."""
        # Try plugins first
        response = await self.plugin_manager.handle_message(message, identity)

        if response:
            # Send response back
            await self.transport_manager.send(
                identity.metadata.get("transport", "websocket"), response, identity.id
            )
        else:
            # Handle core commands
            response = await self._handle_core_command(message, identity)
            if response:
                await self.transport_manager.send(
                    identity.metadata.get("transport", "websocket"), response, identity.id
                )

    async def _handle_core_command(self, message: Message, identity: Identity) -> Message | None:
        """Handle core gateway commands.

        v0.5.0: Integrated safety checks and memory logging
        """
        if not message.command:
            return None

        cmd = message.command

        # v0.5.0: Safety check for dangerous actions
        if self.safety and self.safety.requires_confirmation(cmd.action):
            request = await self.safety.request_confirmation(
                cmd.action, f"Execute {cmd.action}?", timeout=30
            )
            approved = await self.safety.wait_for_confirmation(request.id, timeout=30)

            if not approved:
                return Message(
                    header=Header(correlation_id=message.header.message_id),
                    event=Event(
                        event_type="action_rejected",
                        severity="warning",
                        data={"action": cmd.action, "reason": "not_confirmed"},
                    ),
                )

        # v0.5.0: Log to memory
        if self.memory:
            await self.memory.append(
                f"agent:{identity.id}:actions",
                {
                    "action": cmd.action,
                    "parameters": cmd.parameters,
                    "timestamp": datetime.utcnow().isoformat(),
                },
            )

        if cmd.action == "discover":
            # Discover robots
            endpoints = await self.connector_registry.discover_all()

            # v0.5.0: Also discover tools if available
            tools = []
            if self.tools:
                tools = self.tools.to_mcp_tools()

            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="/discovery/results",
                    data={
                        "robots": [
                            {"uri": e.uri, "name": e.name, "type": e.connector_type}
                            for e in endpoints
                        ],
                        "tools": tools,
                    },
                ),
            )

        elif cmd.action == "fleet.list":
            # List fleets
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(topic="/fleet/list", data=list(self.fleets.keys())),
            )

        elif cmd.action == "fleet.robots":
            # List robots in fleet
            fleet_name = cmd.parameters.get("fleet")
            fleet = self.fleets.get(fleet_name)
            if fleet:
                return Message(
                    header=Header(correlation_id=message.header.message_id),
                    telemetry=Telemetry(
                        topic=f"/fleet/{fleet_name}/robots",
                        data=[{"id": r.robot_id, "name": r.name} for r in fleet.robots.values()],
                    ),
                )

        elif cmd.action == "robot.execute":
            # Execute command on specific robot
            robot_id = cmd.parameters.get("robot_id")
            for fleet in self.fleets.values():
                robot = fleet.get_robot(robot_id)
                if robot:
                    result = await robot.execute(cmd)
                    return Message(
                        header=Header(correlation_id=message.header.message_id),
                        telemetry=Telemetry(topic=f"/robot/{robot_id}/result", data=result),
                    )

        return None

    def create_fleet(self, name: str) -> RobotFleet:
        """Create a new robot fleet."""
        fleet = RobotFleet(name)
        self.fleets[name] = fleet
        return fleet

    async def connect_robot(self, uri: str, fleet_name: str | None = None, **kwargs) -> Robot:
        """Connect to a robot and optionally add to fleet."""
        robot = await self.connector_registry.connect(uri, **kwargs)

        if fleet_name and fleet_name in self.fleets:
            self.fleets[fleet_name].add_robot(robot)

        return robot

    async def start(self) -> None:
        """Start the gateway."""
        logger.info("Starting Agent ROS Bridge...")
        await self.transport_manager.start_all()
        self.running = True
        logger.info("Bridge started")

    async def stop(self) -> None:
        """Stop the gateway."""
        logger.info("Stopping Agent ROS Bridge...")
        self.running = False
        await self.transport_manager.stop_all()

        # Disconnect all robots
        for fleet in self.fleets.values():
            for robot in fleet.robots.values():
                await robot.disconnect()

        # Unload plugins
        for name in list(self.plugin_manager.plugins.keys()):
            await self.plugin_manager.unload_plugin(name)

        logger.info("Bridge stopped")

    # v0.5.0: AI Agent Integration Methods

    def get_langchain_tool(self, actions: list[str] | None = None):
        """Get LangChain tool for this bridge.

        Example:
            from langchain.agents import initialize_agent

            tool = bridge.get_langchain_tool(["navigate", "move_arm"])
            agent = initialize_agent([tool], llm, agent="zero-shot-react-description")
        """
        if not INTEGRATIONS_AVAILABLE:
            raise ImportError("Integrations not available")

        from ..integrations.langchain_adapter import ROSBridgeTool

        return ROSBridgeTool(self, actions=actions)

    def get_autogpt_adapter(self):
        """Get AutoGPT adapter for this bridge.

        Example:
            adapter = bridge.get_autogpt_adapter()
            commands = adapter.get_commands()
        """
        if not INTEGRATIONS_AVAILABLE:
            raise ImportError("Integrations not available")

        from ..integrations.autogpt_adapter import AutoGPTAdapter

        return AutoGPTAdapter(self)

    def get_mcp_server(self, mode: str = "stdio"):
        """Get MCP server transport for this bridge.

        Example:
            mcp = bridge.get_mcp_server(mode="stdio")
            await mcp.start()
        """
        if not INTEGRATIONS_AVAILABLE:
            raise ImportError("Integrations not available")

        from ..integrations.mcp_transport import MCPServerTransport

        return MCPServerTransport(self, mode=mode)

    def get_dashboard(self, port: int = 8080):
        """Get dashboard server for this bridge.

        Example:
            dashboard = bridge.get_dashboard(port=8080)
            await dashboard.start()
        """
        if not INTEGRATIONS_AVAILABLE:
            raise ImportError("Integrations not available")

        from ..integrations.dashboard_server import DashboardServer

        return DashboardServer(self, port=port)

    async def execute_action(self, action: str, parameters: dict[str, Any]) -> dict[str, Any]:
        """Execute an action via the bridge (used by AI integrations).

        This is the main entry point for AI agents to control robots.

        Args:
            action: Action name (e.g., "navigate", "move_arm")
            parameters: Action parameters

        Returns:
            Action result
        """
        # Create command
        cmd = Command(action=action, parameters=parameters)

        # Create dummy identity for AI agent
        identity = Identity(id="ai_agent", name="AI Agent", roles=["operator"])

        # Create message
        msg = Message(command=cmd)

        # Handle via core command handler
        response = await self._handle_core_command(msg, identity)

        if response and response.telemetry:
            return response.telemetry.data

        return {"status": "error", "message": "No response from bridge"}

    def get_actions(self) -> list[str]:
        """Get list of available actions (used by discovery)."""
        # Return common actions - actual implementation would discover from robots
        return ["discover", "fleet.list", "fleet.robots", "robot.execute", "emergency_stop"]

    def emergency_stop(self):
        """Trigger emergency stop."""
        if self.safety:
            self.safety.trigger_emergency_stop("Manual emergency stop triggered")
            logger.critical("EMERGENCY STOP triggered")
        else:
            logger.warning("Safety manager not available")

    @asynccontextmanager
    async def run(self):
        """Context manager for gateway lifecycle."""
        await self.start()
        try:
            yield self
        finally:
            await self.stop()


# =============================================================================
# Example Usage
# =============================================================================


async def example():
    """Example gateway usage."""
    # Create gateway
    gateway = Bridge()

    # Register transports (simplified - actual implementations needed)
    # gateway.transport_manager.register(WebSocketTransport("websocket", {"port": 8765}))
    # gateway.transport_manager.register(GRPCTransport("grpc", {"port": 50051}))

    # Register connectors (simplified - actual implementations needed)
    # gateway.connector_registry.register(ROS2Connector())
    # gateway.connector_registry.register(MQTTConnector())

    # Load plugins
    # await gateway.plugin_manager.load_plugin(GreenhousePlugin())

    # Create fleet
    gateway.create_fleet("warehouse")

    # Start gateway
    async with gateway.run():
        logger.info("Gateway running. Press Ctrl+C to stop.")
        while gateway.running:
            await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(example())
