"""Tests for gateway_v2/core.py - Critical path coverage.

Following TDD principles to improve coverage from 43% to 80%+.
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock, MagicMock
from dataclasses import dataclass

from agent_ros_bridge.gateway_v2.core import (
    Header,
    Command,
    Telemetry,
    Event,
    Identity,
    Message,
    QoS,
    Robot,
    RobotFleet,
    Transport,
    TransportManager,
    Connector,
    ConnectorRegistry,
    Plugin,
    PluginManager,
    Bridge,
)


class TestHeader:
    """Test Header dataclass."""
    
    def test_header_default_creation(self):
        """Header can be created with defaults."""
        header = Header()
        
        assert header.message_id is not None
        assert len(header.message_id) == 36  # UUID
        assert header.timestamp is not None
        assert header.source == ""
        assert header.target == ""
        assert header.correlation_id is None
    
    def test_header_custom_values(self):
        """Header can be created with custom values."""
        header = Header(
            message_id="test-id",
            source="robot1",
            target="agent1",
            correlation_id="corr-123"
        )
        
        assert header.message_id == "test-id"
        assert header.source == "robot1"
        assert header.target == "agent1"
        assert header.correlation_id == "corr-123"


class TestCommand:
    """Test Command dataclass."""
    
    def test_command_creation(self):
        """Command can be created."""
        cmd = Command(action="move")
        
        assert cmd.action == "move"
        assert cmd.parameters == {}
        assert cmd.timeout_ms == 5000
        assert cmd.priority == 5
    
    def test_command_with_parameters(self):
        """Command can have parameters."""
        cmd = Command(
            action="navigate",
            parameters={"x": 1.0, "y": 2.0},
            timeout_ms=10000,
            priority=1
        )
        
        assert cmd.parameters == {"x": 1.0, "y": 2.0}
        assert cmd.timeout_ms == 10000
        assert cmd.priority == 1


class TestTelemetry:
    """Test Telemetry dataclass."""
    
    def test_telemetry_creation(self):
        """Telemetry can be created."""
        tel = Telemetry(topic="/odom", data={"x": 0})
        
        assert tel.topic == "/odom"
        assert tel.data == {"x": 0}
        assert tel.quality == 1.0
    
    def test_telemetry_with_quality(self):
        """Telemetry can have quality score."""
        tel = Telemetry(topic="/scan", data=[], quality=0.8)
        assert tel.quality == 0.8


class TestEvent:
    """Test Event dataclass."""
    
    def test_event_creation(self):
        """Event can be created."""
        evt = Event(type="connected")
        
        assert evt.type == "connected"
        assert evt.data == {}
        assert evt.severity == "info"
    
    def test_event_with_data(self):
        """Event can have data and severity."""
        evt = Event(
            type="error",
            data={"code": 500},
            severity="error"
        )
        
        assert evt.severity == "error"
        assert evt.data == {"code": 500}


class TestIdentity:
    """Test Identity dataclass."""
    
    def test_identity_creation(self):
        """Identity can be created."""
        ident = Identity(id="user1")
        
        assert ident.id == "user1"
        assert ident.type == "anonymous"
        assert ident.roles == []
        assert ident.metadata == {}
    
    def test_identity_with_roles(self):
        """Identity can have roles."""
        ident = Identity(
            id="admin1",
            type="user",
            roles=["admin", "operator"],
            metadata={"team": "robotics"}
        )
        
        assert ident.roles == ["admin", "operator"]
        assert ident.metadata == {"team": "robotics"}


class TestMessage:
    """Test Message dataclass."""
    
    def test_message_default_creation(self):
        """Message can be created with defaults."""
        msg = Message()
        
        assert msg.header is not None
        assert msg.command is None
        assert msg.telemetry is None
        assert msg.event is None
        assert msg.metadata == {}
        assert msg.identity is None
    
    def test_message_with_command(self):
        """Message can contain command."""
        cmd = Command(action="move")
        msg = Message(command=cmd)
        
        assert msg.command == cmd
    
    def test_message_with_telemetry(self):
        """Message can contain telemetry."""
        tel = Telemetry(topic="/odom", data={})
        msg = Message(telemetry=tel)
        
        assert msg.telemetry == tel


class TestQoS:
    """Test QoS enum."""
    
    def test_qos_values(self):
        """QoS has correct values."""
        assert QoS.BEST_EFFORT.value == 0
        assert QoS.RELIABLE.value == 1
        assert QoS.EXACTLY_ONCE.value == 2


class TestRobot:
    """Test Robot class."""
    
    def test_robot_creation(self):
        """Robot can be created."""
        robot = Robot(id="robot1", name="TurtleBot")
        
        assert robot.id == "robot1"
        assert robot.name == "TurtleBot"
        assert robot.status == "offline"
        assert robot.capabilities == []
    
    def test_robot_update_status(self):
        """Robot status can be updated."""
        robot = Robot(id="robot1", name="Test")
        
        robot.status = "online"
        assert robot.status == "online"
    
    def test_robot_is_online(self):
        """Robot online check works."""
        robot = Robot(id="robot1", name="Test")
        
        assert not robot.is_online
        
        robot.status = "online"
        assert robot.is_online


class TestRobotFleet:
    """Test RobotFleet class."""
    
    def test_fleet_creation(self):
        """Fleet can be created."""
        fleet = RobotFleet()
        
        assert fleet.robots == {}
    
    def test_fleet_add_robot(self):
        """Robot can be added to fleet."""
        fleet = RobotFleet()
        robot = Robot(id="r1", name="Robot1")
        
        fleet.add_robot(robot)
        
        assert "r1" in fleet.robots
        assert fleet.robots["r1"] == robot
    
    def test_fleet_get_robot(self):
        """Robot can be retrieved from fleet."""
        fleet = RobotFleet()
        robot = Robot(id="r1", name="Robot1")
        fleet.add_robot(robot)
        
        retrieved = fleet.get_robot("r1")
        
        assert retrieved == robot
    
    def test_fleet_get_nonexistent_robot(self):
        """Getting nonexistent robot returns None."""
        fleet = RobotFleet()
        
        retrieved = fleet.get_robot("nonexistent")
        
        assert retrieved is None
    
    def test_fleet_remove_robot(self):
        """Robot can be removed from fleet."""
        fleet = RobotFleet()
        robot = Robot(id="r1", name="Robot1")
        fleet.add_robot(robot)
        
        fleet.remove_robot("r1")
        
        assert "r1" not in fleet.robots
    
    def test_fleet_get_online_robots(self):
        """Can get list of online robots."""
        fleet = RobotFleet()
        
        r1 = Robot(id="r1", name="Robot1")
        r1.status = "online"
        
        r2 = Robot(id="r2", name="Robot2")
        r2.status = "offline"
        
        fleet.add_robot(r1)
        fleet.add_robot(r2)
        
        online = fleet.get_online_robots()
        
        assert len(online) == 1
        assert online[0].id == "r1"


class TestTransportManager:
    """Test TransportManager class."""
    
    def test_manager_creation(self):
        """Manager can be created."""
        manager = TransportManager()
        
        assert manager.transports == {}
    
    def test_manager_register_transport(self):
        """Transport can be registered."""
        manager = TransportManager()
        transport = Mock()
        transport.name = "test"
        
        manager.register(transport)
        
        assert "test" in manager.transports
    
    def test_manager_get_transport(self):
        """Transport can be retrieved."""
        manager = TransportManager()
        transport = Mock()
        transport.name = "test"
        manager.register(transport)
        
        retrieved = manager.get("test")
        
        assert retrieved == transport
    
    def test_manager_get_nonexistent(self):
        """Getting nonexistent transport returns None."""
        manager = TransportManager()
        
        retrieved = manager.get("nonexistent")
        
        assert retrieved is None
    
    @pytest.mark.asyncio
    async def test_manager_start_all(self):
        """All transports can be started."""
        manager = TransportManager()
        transport = AsyncMock()
        transport.name = "test"
        transport.start = AsyncMock(return_value=True)
        manager.register(transport)
        
        results = await manager.start_all()
        
        assert results["test"] is True
        transport.start.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_manager_stop_all(self):
        """All transports can be stopped."""
        manager = TransportManager()
        transport = AsyncMock()
        transport.name = "test"
        transport.stop = AsyncMock()
        manager.register(transport)
        
        await manager.stop_all()
        
        transport.stop.assert_called_once()


class TestConnectorRegistry:
    """Test ConnectorRegistry class."""
    
    def test_registry_creation(self):
        """Registry can be created."""
        registry = ConnectorRegistry()
        
        assert registry.connectors == {}
    
    def test_registry_register_connector(self):
        """Connector can be registered."""
        registry = ConnectorRegistry()
        connector = Mock()
        connector.name = "ros2"
        
        registry.register(connector)
        
        assert "ros2" in registry.connectors
    
    def test_registry_get_connector(self):
        """Connector can be retrieved."""
        registry = ConnectorRegistry()
        connector = Mock()
        connector.name = "ros2"
        registry.register(connector)
        
        retrieved = registry.get("ros2")
        
        assert retrieved == connector


class TestPluginManager:
    """Test PluginManager class."""
    
    def test_manager_creation(self):
        """Manager can be created."""
        manager = PluginManager()
        
        assert manager.plugins == {}
    
    def test_manager_load_plugin(self):
        """Plugin can be loaded."""
        manager = PluginManager()
        plugin = Mock()
        plugin.name = "test_plugin"
        
        manager.load(plugin)
        
        assert "test_plugin" in manager.plugins


class TestBridge:
    """Test Bridge class."""
    
    def test_bridge_creation(self):
        """Bridge can be created."""
        bridge = Bridge()
        
        assert bridge.transport_manager is not None
        assert bridge.connector_registry is not None
        assert bridge.plugin_manager is not None
        assert bridge.fleet is not None
        assert bridge.running is False
    
    @pytest.mark.asyncio
    async def test_bridge_start_stop(self):
        """Bridge can start and stop."""
        bridge = Bridge()
        
        # Start
        result = await bridge.start()
        assert result is True
        assert bridge.running is True
        
        # Stop
        await bridge.stop()
        assert bridge.running is False
    
    def test_bridge_get_robot(self):
        """Bridge can get robot from fleet."""
        bridge = Bridge()
        robot = Robot(id="r1", name="Test")
        bridge.fleet.add_robot(robot)
        
        retrieved = bridge.get_robot("r1")
        
        assert retrieved == robot
    
    def test_bridge_send_command(self):
        """Bridge can send command to robot."""
        bridge = Bridge()
        
        # This would need more setup with mocks
        # For now, just test the method exists
        assert hasattr(bridge, 'send_command')
