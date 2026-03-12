"""Tests for gateway_v2/core.py - Part 2: Robot, Connector, Plugin classes.

Following TDD to improve coverage.
"""
import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from dataclasses import dataclass

from agent_ros_bridge.gateway_v2.core import (
    Robot,
    Connector,
    RobotEndpoint,
    ConnectorRegistry,
    Plugin,
    PluginManager,
    Bridge,
)
from agent_ros_bridge.gateway_v2.core import (
    Command,
    Telemetry,
    Header,
    Identity,
)


# Concrete implementations for testing
class MockRobot(Robot):
    """Mock robot implementation for testing."""
    
    def __init__(self, robot_id: str, name: str, connector_type: str):
        super().__init__(robot_id, name, connector_type)
        self.commands_executed = []
        self.subscriptions = []
    
    async def connect(self) -> bool:
        self.connected = True
        return True
    
    async def disconnect(self) -> None:
        self.connected = False
    
    async def execute(self, command: Command) -> dict:
        self.commands_executed.append(command)
        return {"status": "success", "command": command.action}
    
    async def subscribe(self, topic: str):
        self.subscriptions.append(topic)
        # Yield mock telemetry
        yield Telemetry(topic=topic, data={"test": "data"})


class MockConnector(Connector):
    """Mock connector implementation for testing."""
    
    connector_type = "mock"
    
    async def connect(self, uri: str, **kwargs) -> Robot:
        return MockRobot("test_id", "Test Robot", "mock")
    
    async def discover(self) -> list:
        return [
            RobotEndpoint(
                uri="mock://robot1",
                name="Robot 1",
                connector_type="mock",
                capabilities=["move"],
                metadata={}
            )
        ]


class TestRobot:
    """Test Robot base class."""
    
    def test_robot_creation(self):
        """Robot can be created."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        assert robot.robot_id == "r1"
        assert robot.name == "Test Robot"
        assert robot.connector_type == "ros2"
        assert robot.connected is False
        assert robot.capabilities == set()
        assert robot.metadata == {}
    
    @pytest.mark.asyncio
    async def test_robot_connect(self):
        """Robot can connect."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        result = await robot.connect()
        
        assert result is True
        assert robot.connected is True
    
    @pytest.mark.asyncio
    async def test_robot_disconnect(self):
        """Robot can disconnect."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        await robot.connect()
        
        await robot.disconnect()
        
        assert robot.connected is False
    
    @pytest.mark.asyncio
    async def test_robot_execute_command(self):
        """Robot can execute commands."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        cmd = Command(action="move", parameters={"x": 1.0})
        
        result = await robot.execute(cmd)
        
        assert result["status"] == "success"
        assert len(robot.commands_executed) == 1
        assert robot.commands_executed[0].action == "move"
    
    @pytest.mark.asyncio
    async def test_robot_subscribe(self):
        """Robot can subscribe to topics."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        async for telemetry in robot.subscribe("/odom"):
            assert telemetry.topic == "/odom"
            break  # Only check first yield
        
        assert "/odom" in robot.subscriptions
    
    def test_robot_notify_subscribers(self):
        """Robot can notify subscribers."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        # Add subscriber
        received = []
        def callback(data):
            received.append(data)
        
        robot._subscribers["/odom"] = [callback]
        
        # Notify
        telemetry = Telemetry(topic="/odom", data={"x": 0})
        robot._notify_subscribers("/odom", telemetry)
        
        assert len(received) == 1
        assert received[0] == telemetry
    
    def test_robot_add_capability(self):
        """Robot can have capabilities."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        robot.capabilities.add("navigation")
        robot.capabilities.add("manipulation")
        
        assert "navigation" in robot.capabilities
        assert "manipulation" in robot.capabilities
    
    def test_robot_metadata(self):
        """Robot can have metadata."""
        robot = MockRobot("r1", "Test Robot", "ros2")
        
        robot.metadata["battery_level"] = 85
        robot.metadata["location"] = "warehouse"
        
        assert robot.metadata["battery_level"] == 85


class TestConnector:
    """Test Connector base class."""
    
    @pytest.mark.asyncio
    async def test_connector_connect(self):
        """Connector can connect to robot."""
        connector = MockConnector()
        
        robot = await connector.connect("mock://test")
        
        assert isinstance(robot, Robot)
        assert robot.robot_id == "test_id"
    
    @pytest.mark.asyncio
    async def test_connector_discover(self):
        """Connector can discover robots."""
        connector = MockConnector()
        
        endpoints = await connector.discover()
        
        assert len(endpoints) == 1
        assert endpoints[0].uri == "mock://robot1"
        assert endpoints[0].connector_type == "mock"


class TestConnectorRegistry:
    """Test ConnectorRegistry."""
    
    def test_registry_creation(self):
        """Registry can be created."""
        registry = ConnectorRegistry()
        
        assert registry.connectors == {}
    
    def test_registry_register_connector(self):
        """Connector can be registered."""
        registry = ConnectorRegistry()
        connector = MockConnector()
        
        registry.register(connector)
        
        assert "mock" in registry.connectors
        assert registry.connectors["mock"] == connector
    
    @pytest.mark.asyncio
    async def test_registry_connect(self):
        """Registry can connect via appropriate connector."""
        registry = ConnectorRegistry()
        connector = MockConnector()
        registry.register(connector)
        
        robot = await registry.connect("mock://test_robot")
        
        assert isinstance(robot, Robot)
    
    @pytest.mark.asyncio
    async def test_registry_connect_unknown_scheme(self):
        """Unknown scheme raises error."""
        registry = ConnectorRegistry()
        
        with pytest.raises(ValueError) as exc_info:
            await registry.connect("unknown://test")
        
        assert "No connector for scheme" in str(exc_info.value)


class MockPlugin(Plugin):
    """Mock plugin for testing."""
    name = "mock_plugin"
    version = "1.0.0"
    
    def __init__(self):
        self.initialized = False
        self.shutdown_called = False
    
    async def initialize(self, gateway: Bridge) -> None:
        self.initialized = True
    
    async def shutdown(self) -> None:
        self.shutdown_called = True


class TestPlugin:
    """Test Plugin base class."""
    
    def test_plugin_attributes(self):
        """Plugin has default attributes."""
        plugin = MockPlugin()
        
        assert plugin.name == "mock_plugin"
        assert plugin.version == "1.0.0"
    
    @pytest.mark.asyncio
    async def test_plugin_lifecycle(self):
        """Plugin has lifecycle methods."""
        plugin = MockPlugin()
        bridge = Bridge()
        
        await plugin.initialize(bridge)
        assert plugin.initialized is True
        
        await plugin.shutdown()
        assert plugin.shutdown_called is True
    
    @pytest.mark.asyncio
    async def test_plugin_handle_message_default(self):
        """Plugin handle_message returns None by default."""
        plugin = MockPlugin()
        
        result = await plugin.handle_message(Message(), Identity(id="u1", name="Test"))
        
        assert result is None


class TestPluginManager:
    """Test PluginManager."""

    def test_manager_creation(self):
        """Manager can be created."""
        manager = PluginManager()

        assert manager.plugins == {}

    @pytest.mark.asyncio
    async def test_manager_load_plugin(self):
        """Plugin can be loaded."""
        manager = PluginManager()
        manager.set_gateway(Bridge())
        plugin = MockPlugin()

        await manager.load_plugin(plugin)

        assert "mock_plugin" in manager.plugins
        assert plugin.initialized is True

    @pytest.mark.asyncio
    async def test_manager_unload_plugin(self):
        """Plugin can be unloaded."""
        manager = PluginManager()
        manager.set_gateway(Bridge())
        plugin = MockPlugin()
        await manager.load_plugin(plugin)

        await manager.unload_plugin("mock_plugin")

        assert "mock_plugin" not in manager.plugins
        assert plugin.shutdown_called is True

    def test_manager_set_gateway(self):
        """Gateway can be set."""
        manager = PluginManager()
        bridge = Bridge()

        manager.set_gateway(bridge)

        assert manager.gateway == bridge

    @pytest.mark.asyncio
    async def test_manager_load_without_gateway_fails(self):
        """Loading plugin without gateway raises error."""
        manager = PluginManager()
        plugin = MockPlugin()

        with pytest.raises(RuntimeError) as exc_info:
            await manager.load_plugin(plugin)

        assert "Gateway not set" in str(exc_info.value)


class TestBridgeAdvanced:
    """Advanced Bridge tests."""
    
    @pytest.fixture
    def bridge(self):
        """Create bridge for testing."""
        return Bridge()
    
    def test_bridge_has_transport_manager(self, bridge):
        """Bridge has transport manager."""
        assert bridge.transport_manager is not None
        assert hasattr(bridge.transport_manager, 'transports')
    
    def test_bridge_has_connector_registry(self, bridge):
        """Bridge has connector registry."""
        assert bridge.connector_registry is not None
        assert hasattr(bridge.connector_registry, 'connectors')
    
    def test_bridge_has_plugin_manager(self, bridge):
        """Bridge has plugin manager."""
        assert bridge.plugin_manager is not None
        assert hasattr(bridge.plugin_manager, 'plugins')
    
    def test_bridge_has_fleets(self, bridge):
        """Bridge has fleets."""
        assert hasattr(bridge, 'fleets')
    
    @pytest.mark.asyncio
    async def test_bridge_lifecycle(self, bridge):
        """Bridge can start and stop."""
        # Start
        result = await bridge.start()
        assert result is True or result is None  # Depends on implementation
        
        # Stop
        await bridge.stop()
        # Should complete without error
    
    def test_bridge_register_transport(self, bridge):
        """Bridge can register transport."""
        from agent_ros_bridge.gateway_v2.transports import WebSocketTransport

        transport = WebSocketTransport({"port": 9999})
        bridge.transport_manager.register(transport)

        assert "websocket" in bridge.transport_manager.transports


class TestMessageFlow:
    """Test message flow through system."""
    
    @pytest.mark.asyncio
    async def test_message_routing(self):
        """Messages can be routed through transport manager."""
        from agent_ros_bridge.gateway_v2.core import TransportManager, Message, Identity
        
        manager = TransportManager()
        
        # Set up message handler
        received_messages = []
        async def handler(msg, identity):
            received_messages.append((msg, identity))
        
        manager.on_message(handler)
        
        # Create mock transport that calls handler
        transport = Mock()
        transport.name = "test"
        transport.on_message = lambda h: None
        
        manager.register(transport)
        
        # Route a message
        msg = Message()
        identity = Identity(id="user1", name="Test User")
        
        # Manually trigger routing
        manager._route_message(msg, identity)
        
        # Wait for async handler
        await asyncio.sleep(0.1)
        
        assert len(received_messages) == 1
        assert received_messages[0][0] == msg
        assert received_messages[0][1] == identity


class TestRobotEndpoint:
    """Test RobotEndpoint dataclass."""
    
    def test_endpoint_creation(self):
        """Endpoint can be created."""
        endpoint = RobotEndpoint(
            uri="ros2://192.168.1.1",
            name="Warehouse Robot",
            connector_type="ros2",
            capabilities=["navigate", "pick"],
            metadata={"floor": 1}
        )
        
        assert endpoint.uri == "ros2://192.168.1.1"
        assert endpoint.name == "Warehouse Robot"
        assert "navigate" in endpoint.capabilities
        assert endpoint.metadata["floor"] == 1
