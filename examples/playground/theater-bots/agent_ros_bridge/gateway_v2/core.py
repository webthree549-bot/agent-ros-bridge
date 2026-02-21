#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenClaw Gateway - Universal Robot Gateway

The next-generation architecture for robot-AI connectivity.
Multi-protocol, multi-robot, cloud-native.
"""

from __future__ import annotations
import asyncio
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable, AsyncIterator, Set
from enum import Enum, auto
from contextlib import asynccontextmanager
import uuid
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")


# =============================================================================
# Core Data Models
# =============================================================================

class QoS(Enum):
    """Quality of Service levels"""
    BEST_EFFORT = auto()      # Fire and forget
    AT_LEAST_ONCE = auto()    # Retry until ack
    EXACTLY_ONCE = auto()     # Deduplication guaranteed


@dataclass
class Identity:
    """Authenticated identity"""
    id: str
    name: str
    roles: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass 
class Header:
    """Message header"""
    message_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = field(default_factory=datetime.utcnow)
    source: str = ""
    target: str = ""
    correlation_id: Optional[str] = None


@dataclass
class Command:
    """Robot command"""
    action: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    timeout_ms: int = 5000
    priority: int = 5  # 1-10, lower = higher priority


@dataclass
class Telemetry:
    """Sensor/telemetry data"""
    topic: str
    data: Any
    quality: float = 1.0  # Data quality 0-1


@dataclass
class Event:
    """System event"""
    event_type: str
    severity: str = "info"  # debug, info, warning, error, critical
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Message:
    """Unified message format"""
    header: Header = field(default_factory=Header)
    command: Optional[Command] = None
    telemetry: Optional[Telemetry] = None
    event: Optional[Event] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


# =============================================================================
# Transport Layer Abstraction
# =============================================================================

class Transport(ABC):
    """Abstract transport layer"""
    
    def __init__(self, name: str, config: Dict[str, Any]):
        self.name = name
        self.config = config
        self.running = False
        self.message_handler: Optional[Callable[[Message, Identity], asyncio.Future]] = None
    
    @abstractmethod
    async def start(self) -> bool:
        """Start the transport"""
        pass
    
    @abstractmethod
    async def stop(self) -> None:
        """Stop the transport"""
        pass
    
    @abstractmethod
    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific recipient"""
        pass
    
    @abstractmethod
    async def broadcast(self, message: Message) -> List[str]:
        """Broadcast to all connected clients"""
        pass
    
    def on_message(self, handler: Callable[[Message, Identity], asyncio.Future]):
        """Register message handler"""
        self.message_handler = handler


class TransportManager:
    """Manages multiple transport endpoints"""
    
    def __init__(self):
        self.transports: Dict[str, Transport] = {}
        self._message_handler: Optional[Callable] = None
    
    def register(self, transport: Transport) -> None:
        """Register a transport"""
        self.transports[transport.name] = transport
        transport.on_message(self._route_message)
        logger.info(f"Registered transport: {transport.name}")
    
    def _route_message(self, message: Message, identity: Identity):
        """Route incoming message to handler"""
        if self._message_handler:
            return self._message_handler(message, identity)
    
    def on_message(self, handler: Callable[[Message, Identity], asyncio.Future]):
        """Set global message handler"""
        self._message_handler = handler
    
    async def start_all(self) -> None:
        """Start all registered transports"""
        await asyncio.gather(*[
            t.start() for t in self.transports.values()
        ])
    
    async def stop_all(self) -> None:
        """Stop all transports"""
        await asyncio.gather(*[
            t.stop() for t in self.transports.values()
        ])
    
    async def send(self, transport_name: str, message: Message, recipient: str) -> bool:
        """Send via specific transport"""
        if transport_name not in self.transports:
            raise ValueError(f"Unknown transport: {transport_name}")
        return await self.transports[transport_name].send(message, recipient)


# =============================================================================
# Connector Layer Abstraction
# =============================================================================

class Robot(ABC):
    """Abstract robot representation"""
    
    def __init__(self, robot_id: str, name: str, connector_type: str):
        self.robot_id = robot_id
        self.name = name
        self.connector_type = connector_type
        self.connected = False
        self.capabilities: Set[str] = set()
        self.metadata: Dict[str, Any] = {}
        self._subscribers: Dict[str, List[Callable]] = {}
    
    @abstractmethod
    async def connect(self) -> bool:
        """Connect to robot"""
        pass
    
    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from robot"""
        pass
    
    @abstractmethod
    async def execute(self, command: Command) -> Any:
        """Execute command on robot"""
        pass
    
    @abstractmethod
    async def subscribe(self, topic: str) -> AsyncIterator[Telemetry]:
        """Subscribe to telemetry topic"""
        pass
    
    def _notify_subscribers(self, topic: str, data: Telemetry):
        """Notify subscribers of new data"""
        for callback in self._subscribers.get(topic, []):
            try:
                callback(data)
            except Exception as e:
                logger.error(f"Subscriber error: {e}")


class Connector(ABC):
    """Abstract robot connector"""
    
    connector_type: str = "abstract"
    
    @abstractmethod
    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to robot at URI"""
        pass
    
    @abstractmethod
    async def discover(self) -> List[RobotEndpoint]:
        """Discover available robots"""
        pass


@dataclass
class RobotEndpoint:
    """Discovered robot endpoint"""
    uri: str
    name: str
    connector_type: str
    capabilities: List[str]
    metadata: Dict[str, Any]


class ConnectorRegistry:
    """Registry of robot connectors"""
    
    def __init__(self):
        self.connectors: Dict[str, Connector] = {}
    
    def register(self, connector: Connector) -> None:
        """Register a connector"""
        self.connectors[connector.connector_type] = connector
        logger.info(f"Registered connector: {connector.connector_type}")
    
    async def connect(self, uri: str, **kwargs) -> Robot:
        """Connect to robot using appropriate connector"""
        # Parse URI to determine connector type
        # e.g., ros2://192.168.1.100:7411
        scheme = uri.split("://")[0]
        
        if scheme not in self.connectors:
            raise ValueError(f"No connector for scheme: {scheme}")
        
        return await self.connectors[scheme].connect(uri, **kwargs)
    
    async def discover_all(self) -> List[RobotEndpoint]:
        """Discover robots using all connectors"""
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
    """Manages a fleet of robots"""
    
    def __init__(self, name: str):
        self.name = name
        self.robots: Dict[str, Robot] = {}
        self.groups: Dict[str, List[str]] = {}
    
    def add_robot(self, robot: Robot) -> None:
        """Add robot to fleet"""
        self.robots[robot.robot_id] = robot
        logger.info(f"Added robot {robot.name} to fleet {self.name}")
    
    def remove_robot(self, robot_id: str) -> None:
        """Remove robot from fleet"""
        if robot_id in self.robots:
            del self.robots[robot_id]
            logger.info(f"Removed robot {robot_id} from fleet {self.name}")
    
    def get_robot(self, robot_id: str) -> Optional[Robot]:
        """Get robot by ID"""
        return self.robots.get(robot_id)
    
    def select(self, predicate: Callable[[Robot], bool]) -> List[Robot]:
        """Select robots matching predicate"""
        return [r for r in self.robots.values() if predicate(r)]
    
    async def broadcast(self, command: Command, 
                       selector: Optional[Callable[[Robot], bool]] = None) -> Dict[str, Any]:
        """Broadcast command to selected robots"""
        targets = self.select(selector) if selector else list(self.robots.values())
        
        async def exec_with_id(robot: Robot):
            try:
                result = await asyncio.wait_for(
                    robot.execute(command),
                    timeout=command.timeout_ms / 1000
                )
                return robot.robot_id, {"status": "success", "result": result}
            except asyncio.TimeoutError:
                return robot.robot_id, {"status": "timeout"}
            except Exception as e:
                return robot.robot_id, {"status": "error", "error": str(e)}
        
        results = await asyncio.gather(*[exec_with_id(r) for r in targets])
        return dict(results)


class Plugin(ABC):
    """Abstract plugin"""
    
    name: str = "abstract_plugin"
    version: str = "0.0.1"
    
    @abstractmethod
    async def initialize(self, gateway: Bridge) -> None:
        """Initialize plugin with gateway reference"""
        pass
    
    @abstractmethod
    async def shutdown(self) -> None:
        """Shutdown plugin"""
        pass
    
    async def handle_message(self, message: Message, identity: Identity) -> Optional[Message]:
        """Handle message - return response or None"""
        return None


class PluginManager:
    """Manages plugins with hot reload"""
    
    def __init__(self):
        self.plugins: Dict[str, Plugin] = {}
        self.gateway: Optional[Bridge] = None
    
    def set_gateway(self, gateway: Bridge) -> None:
        """Set gateway reference"""
        self.gateway = gateway
    
    async def load_plugin(self, plugin: Plugin) -> None:
        """Load and initialize a plugin"""
        if self.gateway is None:
            raise RuntimeError("Gateway not set")
        
        self.plugins[plugin.name] = plugin
        await plugin.initialize(self.gateway)
        logger.info(f"Loaded plugin: {plugin.name} v{plugin.version}")
    
    async def unload_plugin(self, name: str) -> None:
        """Unload a plugin"""
        if name in self.plugins:
            await self.plugins[name].shutdown()
            del self.plugins[name]
            logger.info(f"Unloaded plugin: {name}")
    
    async def handle_message(self, message: Message, identity: Identity) -> Optional[Message]:
        """Route message through plugins"""
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
    """Universal robot gateway - main entry point"""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}
        self.transport_manager = TransportManager()
        self.connector_registry = ConnectorRegistry()
        self.fleets: Dict[str, RobotFleet] = {}
        self.plugin_manager = PluginManager()
        self.plugin_manager.set_gateway(self)
        self.running = False
        
        # Set up message routing
        self.transport_manager.on_message(self._handle_message)
    
    async def _handle_message(self, message: Message, identity: Identity) -> None:
        """Handle incoming message from transport"""
        # Try plugins first
        response = await self.plugin_manager.handle_message(message, identity)
        
        if response:
            # Send response back
            await self.transport_manager.send(
                identity.metadata.get("transport", "websocket"),
                response,
                identity.id
            )
        else:
            # Handle core commands
            response = await self._handle_core_command(message, identity)
            if response:
                await self.transport_manager.send(
                    identity.metadata.get("transport", "websocket"),
                    response,
                    identity.id
                )
    
    async def _handle_core_command(self, message: Message, identity: Identity) -> Optional[Message]:
        """Handle core gateway commands"""
        if not message.command:
            return None
        
        cmd = message.command
        
        if cmd.action == "discover":
            # Discover robots
            endpoints = await self.connector_registry.discover_all()
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="/discovery/results",
                    data=[{"uri": e.uri, "name": e.name, "type": e.connector_type} 
                          for e in endpoints]
                )
            )
        
        elif cmd.action == "fleet.list":
            # List fleets
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="/fleet/list",
                    data=list(self.fleets.keys())
                )
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
                        data=[{"id": r.robot_id, "name": r.name} 
                              for r in fleet.robots.values()]
                    )
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
                        telemetry=Telemetry(
                            topic=f"/robot/{robot_id}/result",
                            data=result
                        )
                    )
        
        return None
    
    def create_fleet(self, name: str) -> RobotFleet:
        """Create a new robot fleet"""
        fleet = RobotFleet(name)
        self.fleets[name] = fleet
        return fleet
    
    async def connect_robot(self, uri: str, fleet_name: Optional[str] = None, 
                           **kwargs) -> Robot:
        """Connect to a robot and optionally add to fleet"""
        robot = await self.connector_registry.connect(uri, **kwargs)
        
        if fleet_name and fleet_name in self.fleets:
            self.fleets[fleet_name].add_robot(robot)
        
        return robot
    
    async def start(self) -> None:
        """Start the gateway"""
        logger.info("Starting OpenClaw Gateway...")
        await self.transport_manager.start_all()
        self.running = True
        logger.info("Gateway started")
    
    async def stop(self) -> None:
        """Stop the gateway"""
        logger.info("Stopping OpenClaw Gateway...")
        self.running = False
        await self.transport_manager.stop_all()
        
        # Disconnect all robots
        for fleet in self.fleets.values():
            for robot in fleet.robots.values():
                await robot.disconnect()
        
        # Unload plugins
        for name in list(self.plugin_manager.plugins.keys()):
            await self.plugin_manager.unload_plugin(name)
        
        logger.info("Gateway stopped")
    
    @asynccontextmanager
    async def run(self):
        """Context manager for gateway lifecycle"""
        await self.start()
        try:
            yield self
        finally:
            await self.stop()


# =============================================================================
# Example Usage
# =============================================================================

async def example():
    """Example gateway usage"""
    
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
    warehouse = gateway.create_fleet("warehouse")
    
    # Start gateway
    async with gateway.run():
        logger.info("Gateway running. Press Ctrl+C to stop.")
        while gateway.running:
            await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(example())
