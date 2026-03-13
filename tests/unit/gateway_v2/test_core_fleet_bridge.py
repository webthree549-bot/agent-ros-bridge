"""Additional unit tests for gateway_v2/core.py - RobotFleet and Bridge classes.

Following TDD patterns to improve coverage.
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from dataclasses import dataclass

from agent_ros_bridge.gateway_v2.core import (
    Robot,
    Connector,
    RobotEndpoint,
    ConnectorRegistry,
    Plugin,
    PluginManager,
    Bridge,
    RobotFleet,
    Transport,
    TransportManager,
    Message,
    Command,
    Telemetry,
    Header,
    Identity,
    Event,
    QoS,
)


class MockRobot(Robot):
    """Mock robot implementation for testing."""

    def __init__(self, robot_id: str, name: str, connector_type: str):
        super().__init__(robot_id, name, connector_type)
        self.commands_executed = []
        self.subscriptions = []
        self._should_fail = False

    async def connect(self) -> bool:
        if self._should_fail:
            return False
        self.connected = True
        return True

    async def disconnect(self) -> None:
        self.connected = False

    async def execute(self, command: Command) -> dict:
        if self._should_fail:
            raise Exception("Execution failed")
        self.commands_executed.append(command)
        return {"status": "success", "command": command.action}

    async def subscribe(self, topic: str):
        self.subscriptions.append(topic)
        yield Telemetry(topic=topic, data={"test": "data"})


class TestRobotFleet:
    """Test RobotFleet class."""

    def test_fleet_creation(self):
        """Fleet can be created with name."""
        fleet = RobotFleet("warehouse")
        assert fleet.name == "warehouse"
        assert fleet.robots == {}
        assert fleet.groups == {}

    def test_fleet_add_robot(self):
        """Robot can be added to fleet."""
        fleet = RobotFleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")

        fleet.add_robot(robot)

        assert "r1" in fleet.robots
        assert fleet.robots["r1"] == robot

    def test_fleet_remove_robot(self):
        """Robot can be removed from fleet."""
        fleet = RobotFleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        fleet.add_robot(robot)

        fleet.remove_robot("r1")

        assert "r1" not in fleet.robots

    def test_fleet_get_robot(self):
        """Robot can be retrieved by ID."""
        fleet = RobotFleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        fleet.add_robot(robot)

        result = fleet.get_robot("r1")

        assert result == robot

    def test_fleet_get_robot_not_found(self):
        """Getting non-existent robot returns None."""
        fleet = RobotFleet("warehouse")

        result = fleet.get_robot("nonexistent")

        assert result is None

    def test_fleet_select_with_predicate(self):
        """Robots can be selected with predicate."""
        fleet = RobotFleet("warehouse")
        robot1 = MockRobot("r1", "Robot 1", "ros2")
        robot1.capabilities.add("navigation")
        robot2 = MockRobot("r2", "Robot 2", "ros2")
        robot2.capabilities.add("manipulation")

        fleet.add_robot(robot1)
        fleet.add_robot(robot2)

        # Select only robots with navigation capability
        result = fleet.select(lambda r: "navigation" in r.capabilities)

        assert len(result) == 1
        assert result[0] == robot1

    def test_fleet_select_all(self):
        """Select all robots when no predicate."""
        fleet = RobotFleet("warehouse")
        robot1 = MockRobot("r1", "Robot 1", "ros2")
        robot2 = MockRobot("r2", "Robot 2", "ros2")

        fleet.add_robot(robot1)
        fleet.add_robot(robot2)

        result = fleet.select(lambda r: True)

        assert len(result) == 2

    @pytest.mark.asyncio
    async def test_fleet_broadcast_command(self):
        """Command can be broadcast to all robots."""
        fleet = RobotFleet("warehouse")
        robot1 = MockRobot("r1", "Robot 1", "ros2")
        robot2 = MockRobot("r2", "Robot 2", "ros2")

        fleet.add_robot(robot1)
        fleet.add_robot(robot2)

        cmd = Command(action="move", parameters={"x": 1.0})
        results = await fleet.broadcast(cmd)

        assert len(results) == 2
        assert results["r1"]["status"] == "success"
        assert results["r2"]["status"] == "success"

    @pytest.mark.asyncio
    async def test_fleet_broadcast_with_selector(self):
        """Command can be broadcast to selected robots only."""
        fleet = RobotFleet("warehouse")
        robot1 = MockRobot("r1", "Robot 1", "ros2")
        robot1.capabilities.add("navigation")
        robot2 = MockRobot("r2", "Robot 2", "ros2")
        robot2.capabilities.add("manipulation")

        fleet.add_robot(robot1)
        fleet.add_robot(robot2)

        cmd = Command(action="move", parameters={"x": 1.0})
        results = await fleet.broadcast(cmd, selector=lambda r: "navigation" in r.capabilities)

        assert len(results) == 1
        assert "r1" in results

    @pytest.mark.asyncio
    async def test_fleet_broadcast_timeout(self):
        """Broadcast handles robot timeout."""
        fleet = RobotFleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        robot._should_fail = True  # Will cause timeout

        fleet.add_robot(robot)

        cmd = Command(action="move", parameters={"x": 1.0}, timeout_ms=100)
        results = await fleet.broadcast(cmd)

        assert results["r1"]["status"] == "error"

    @pytest.mark.asyncio
    async def test_fleet_broadcast_error(self):
        """Broadcast handles robot execution error."""
        fleet = RobotFleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")

        # Mock execute to raise exception
        async def failing_execute(cmd):
            raise Exception("Execution failed")

        robot.execute = failing_execute

        fleet.add_robot(robot)

        cmd = Command(action="move", parameters={"x": 1.0})
        results = await fleet.broadcast(cmd)

        assert results["r1"]["status"] == "error"
        assert "Execution failed" in results["r1"]["error"]


class TestTransport:
    """Test Transport base class."""

    class ConcreteTransport(Transport):
        """Concrete implementation for testing."""

        async def start(self) -> bool:
            self.running = True
            return True

        async def stop(self) -> None:
            self.running = False

        async def send(self, message: Message, recipient: str) -> bool:
            return True

        async def broadcast(self, message: Message) -> list[str]:
            return ["client1", "client2"]

    def test_transport_creation(self):
        """Transport can be created."""
        transport = self.ConcreteTransport("test", {"port": 8080})

        assert transport.name == "test"
        assert transport.config == {"port": 8080}
        assert transport.running is False
        assert transport.message_handler is None

    def test_transport_on_message(self):
        """Message handler can be registered."""
        transport = self.ConcreteTransport("test", {})

        def handler(msg, identity):
            pass

        transport.on_message(handler)

        assert transport.message_handler == handler

    @pytest.mark.asyncio
    async def test_transport_lifecycle(self):
        """Transport can start and stop."""
        transport = self.ConcreteTransport("test", {})

        result = await transport.start()
        assert result is True
        assert transport.running is True

        await transport.stop()
        assert transport.running is False


class TestTransportManager:
    """Test TransportManager class."""

    def test_manager_creation(self):
        """Manager can be created."""
        manager = TransportManager()

        assert manager.transports == {}
        assert manager._message_handler is None

    def test_manager_register_transport(self):
        """Transport can be registered."""
        manager = TransportManager()
        transport = Mock()
        transport.name = "test"
        transport.on_message = Mock()

        manager.register(transport)

        assert "test" in manager.transports
        assert manager.transports["test"] == transport

    def test_manager_on_message(self):
        """Global message handler can be set."""
        manager = TransportManager()

        def handler(msg, identity):
            pass

        manager.on_message(handler)

        assert manager._message_handler == handler

    @pytest.mark.asyncio
    async def test_manager_start_all(self):
        """All transports can be started."""
        manager = TransportManager()
        transport1 = Mock()
        transport1.name = "t1"
        transport1.on_message = Mock()
        transport1.start = AsyncMock()
        transport2 = Mock()
        transport2.name = "t2"
        transport2.on_message = Mock()
        transport2.start = AsyncMock()

        manager.register(transport1)
        manager.register(transport2)

        await manager.start_all()

        transport1.start.assert_called_once()
        transport2.start.assert_called_once()

    @pytest.mark.asyncio
    async def test_manager_stop_all(self):
        """All transports can be stopped."""
        manager = TransportManager()
        transport1 = Mock()
        transport1.name = "t1"
        transport1.on_message = Mock()
        transport1.stop = AsyncMock()
        transport2 = Mock()
        transport2.name = "t2"
        transport2.on_message = Mock()
        transport2.stop = AsyncMock()

        manager.register(transport1)
        manager.register(transport2)

        await manager.stop_all()

        transport1.stop.assert_called_once()
        transport2.stop.assert_called_once()

    @pytest.mark.asyncio
    async def test_manager_send(self):
        """Message can be sent via specific transport."""
        manager = TransportManager()
        transport = Mock()
        transport.name = "websocket"
        transport.on_message = Mock()
        transport.send = AsyncMock(return_value=True)

        manager.register(transport)

        msg = Message()
        result = await manager.send("websocket", msg, "client1")

        assert result is True
        transport.send.assert_called_once_with(msg, "client1")

    @pytest.mark.asyncio
    async def test_manager_send_unknown_transport(self):
        """Sending to unknown transport raises error."""
        manager = TransportManager()

        msg = Message()
        with pytest.raises(ValueError, match="Unknown transport"):
            await manager.send("unknown", msg, "client1")

    @pytest.mark.asyncio
    async def test_manager_route_message(self):
        """Messages are routed to handler."""
        manager = TransportManager()

        received = []

        async def handler(msg, identity):
            received.append((msg, identity))

        manager.on_message(handler)

        transport = Mock()
        transport.name = "test"
        transport.on_message = Mock()
        manager.register(transport)

        msg = Message()
        identity = Identity(id="u1", name="Test")

        # Manually trigger routing
        manager._route_message(msg, identity)

        # Wait for async handler
        await asyncio.sleep(0.1)

        assert len(received) == 1


class TestBridgeAdvanced:
    """Advanced tests for Bridge class."""

    @pytest.fixture
    def bridge(self):
        """Create bridge for testing."""
        return Bridge()

    def test_bridge_creation(self, bridge):
        """Bridge can be created."""
        assert bridge.config == {}
        assert bridge.running is False
        assert bridge.transport_manager is not None
        assert bridge.connector_registry is not None
        assert bridge.plugin_manager is not None
        assert bridge.fleets == {}

    def test_bridge_creation_with_config(self):
        """Bridge can be created with config."""
        config = {"memory_backend": "sqlite", "memory_path": ":memory:"}
        bridge = Bridge(config=config)

        assert bridge.config == config

    def test_bridge_create_fleet(self, bridge):
        """Fleet can be created via bridge."""
        fleet = bridge.create_fleet("warehouse")

        assert fleet.name == "warehouse"
        assert "warehouse" in bridge.fleets
        assert bridge.fleets["warehouse"] == fleet

    @pytest.mark.asyncio
    async def test_bridge_connect_robot(self, bridge):
        """Robot can be connected via bridge."""
        # Mock connector registry
        mock_robot = MockRobot("r1", "Robot 1", "ros2")
        bridge.connector_registry.connect = AsyncMock(return_value=mock_robot)

        robot = await bridge.connect_robot("ros2://192.168.1.1")

        assert robot == mock_robot
        bridge.connector_registry.connect.assert_called_once_with("ros2://192.168.1.1")

    @pytest.mark.asyncio
    async def test_bridge_connect_robot_to_fleet(self, bridge):
        """Robot can be connected and added to fleet."""
        mock_robot = MockRobot("r1", "Robot 1", "ros2")
        bridge.connector_registry.connect = AsyncMock(return_value=mock_robot)
        bridge.create_fleet("warehouse")

        robot = await bridge.connect_robot("ros2://192.168.1.1", fleet_name="warehouse")

        assert robot == mock_robot
        assert bridge.fleets["warehouse"].get_robot("r1") == mock_robot

    @pytest.mark.asyncio
    async def test_bridge_start_stop(self, bridge):
        """Bridge can start and stop."""
        bridge.transport_manager.start_all = AsyncMock()
        bridge.transport_manager.stop_all = AsyncMock()

        await bridge.start()
        assert bridge.running is True
        bridge.transport_manager.start_all.assert_called_once()

        await bridge.stop()
        assert bridge.running is False
        bridge.transport_manager.stop_all.assert_called_once()

    @pytest.mark.asyncio
    async def test_bridge_stop_disconnects_robots(self, bridge):
        """Stopping bridge disconnects all robots."""
        fleet = bridge.create_fleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        await robot.connect()
        fleet.add_robot(robot)

        bridge.transport_manager.stop_all = AsyncMock()
        bridge.plugin_manager.unload_plugin = AsyncMock()

        await bridge.stop()

        assert robot.connected is False

    @pytest.mark.asyncio
    async def test_bridge_stop_unloads_plugins(self, bridge):
        """Stopping bridge unloads all plugins."""
        bridge.plugin_manager.plugins = {"plugin1": Mock()}
        bridge.plugin_manager.unload_plugin = AsyncMock()
        bridge.transport_manager.stop_all = AsyncMock()

        await bridge.stop()

        bridge.plugin_manager.unload_plugin.assert_called_once_with("plugin1")

    def test_bridge_get_actions(self, bridge):
        """Bridge returns available actions."""
        actions = bridge.get_actions()

        assert isinstance(actions, list)
        assert "discover" in actions
        assert "fleet.list" in actions
        assert "robot.execute" in actions

    @pytest.mark.asyncio
    async def test_bridge_execute_action(self, bridge):
        """Action can be executed via bridge."""
        # Create a fleet with a robot
        fleet = bridge.create_fleet("default")
        robot = MockRobot("r1", "Test Robot", "ros2")
        fleet.add_robot(robot)

        result = await bridge.execute_action("robot.execute", {"robot_id": "r1"})

        assert "status" in result

    def test_bridge_emergency_stop(self, bridge):
        """Emergency stop can be triggered."""
        # Should not raise error even without safety manager
        bridge.emergency_stop()

    @pytest.mark.asyncio
    async def test_bridge_handle_core_command_discover(self, bridge):
        """Bridge handles discover command."""
        bridge.connector_registry.discover_all = AsyncMock(return_value=[])

        msg = Message(
            command=Command(action="discover", parameters={}),
            identity=Identity(id="u1", name="Test"),
        )

        response = await bridge._handle_core_command(msg, Identity(id="u1", name="Test"))

        assert response is not None
        assert response.telemetry is not None
        assert response.telemetry.topic == "/discovery/results"

    @pytest.mark.asyncio
    async def test_bridge_handle_core_command_fleet_list(self, bridge):
        """Bridge handles fleet.list command."""
        bridge.create_fleet("fleet1")
        bridge.create_fleet("fleet2")

        msg = Message(
            command=Command(action="fleet.list", parameters={}),
            identity=Identity(id="u1", name="Test"),
        )

        response = await bridge._handle_core_command(msg, Identity(id="u1", name="Test"))

        assert response is not None
        assert response.telemetry is not None
        assert "fleet1" in response.telemetry.data
        assert "fleet2" in response.telemetry.data

    @pytest.mark.asyncio
    async def test_bridge_handle_core_command_fleet_robots(self, bridge):
        """Bridge handles fleet.robots command."""
        fleet = bridge.create_fleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        fleet.add_robot(robot)

        msg = Message(
            command=Command(action="fleet.robots", parameters={"fleet": "warehouse"}),
            identity=Identity(id="u1", name="Test"),
        )

        response = await bridge._handle_core_command(msg, Identity(id="u1", name="Test"))

        assert response is not None
        assert response.telemetry is not None
        assert len(response.telemetry.data) == 1

    @pytest.mark.asyncio
    async def test_bridge_handle_core_command_robot_execute(self, bridge):
        """Bridge handles robot.execute command."""
        fleet = bridge.create_fleet("warehouse")
        robot = MockRobot("r1", "Robot 1", "ros2")
        fleet.add_robot(robot)

        msg = Message(
            command=Command(
                action="robot.execute", parameters={"robot_id": "r1", "action": "move"}
            ),
            identity=Identity(id="u1", name="Test"),
        )

        response = await bridge._handle_core_command(msg, Identity(id="u1", name="Test"))

        assert response is not None
        assert response.telemetry is not None

    @pytest.mark.asyncio
    async def test_bridge_handle_core_command_unknown(self, bridge):
        """Bridge returns None for unknown commands."""
        msg = Message(
            command=Command(action="unknown_command", parameters={}),
            identity=Identity(id="u1", name="Test"),
        )

        response = await bridge._handle_core_command(msg, Identity(id="u1", name="Test"))

        assert response is None

    @pytest.mark.asyncio
    async def test_bridge_handle_message_with_plugin_response(self, bridge):
        """Bridge handles message with plugin response."""
        bridge.plugin_manager.handle_message = AsyncMock(
            return_value=Message(telemetry=Telemetry(topic="/test", data={}))
        )
        bridge.transport_manager.send = AsyncMock()

        msg = Message(command=Command(action="test", parameters={}))
        identity = Identity(id="u1", name="Test", metadata={"transport": "websocket"})

        await bridge._handle_message(msg, identity)

        bridge.transport_manager.send.assert_called_once()

    @pytest.mark.asyncio
    async def test_bridge_run_context_manager(self, bridge):
        """Bridge can be used as async context manager."""
        bridge.start = AsyncMock()
        bridge.stop = AsyncMock()

        async with bridge.run() as b:
            assert b == bridge
            bridge.start.assert_called_once()

        bridge.stop.assert_called_once()


class TestTelemetry:
    """Test Telemetry data class."""

    def test_telemetry_creation(self):
        """Telemetry can be created."""
        telemetry = Telemetry(topic="/odom", data={"x": 1.0})

        assert telemetry.topic == "/odom"
        assert telemetry.data == {"x": 1.0}
        assert telemetry.quality == 1.0

    def test_telemetry_with_quality(self):
        """Telemetry can have quality score."""
        telemetry = Telemetry(topic="/odom", data={"x": 1.0}, quality=0.85)

        assert telemetry.quality == 0.85


class TestEvent:
    """Test Event data class."""

    def test_event_creation(self):
        """Event can be created."""
        event = Event(event_type="robot_connected")

        assert event.event_type == "robot_connected"
        assert event.severity == "info"
        assert event.data == {}

    def test_event_with_severity(self):
        """Event can have severity."""
        event = Event(event_type="error", severity="error", data={"code": 500})

        assert event.severity == "error"
        assert event.data["code"] == 500


class TestMessage:
    """Test Message data class."""

    def test_message_creation(self):
        """Message can be created."""
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
        telemetry = Telemetry(topic="/odom", data={})
        msg = Message(telemetry=telemetry)

        assert msg.telemetry == telemetry

    def test_message_with_event(self):
        """Message can contain event."""
        event = Event(event_type="connected")
        msg = Message(event=event)

        assert msg.event == event

    def test_message_with_identity(self):
        """Message can contain identity."""
        identity = Identity(id="u1", name="Test")
        msg = Message(identity=identity)

        assert msg.identity == identity


class TestQoSEnum:
    """Test QoS enum."""

    def test_qos_values(self):
        """QoS has expected values."""
        assert QoS.BEST_EFFORT is not None
        assert QoS.AT_LEAST_ONCE is not None
        assert QoS.EXACTLY_ONCE is not None

    def test_qos_comparison(self):
        """QoS values can be compared."""
        assert QoS.BEST_EFFORT != QoS.AT_LEAST_ONCE
        assert QoS.AT_LEAST_ONCE != QoS.EXACTLY_ONCE
