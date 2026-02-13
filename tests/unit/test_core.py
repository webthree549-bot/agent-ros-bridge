"""Unit tests for core Bridge functionality"""

import pytest
import asyncio
from unittest.mock import Mock, patch

from agent_ros_bridge.gateway_v2.core import (
    Bridge,
    Transport,
    Connector,
    Robot,
    RobotFleet,
    Message,
    Header,
    Command,
    Identity,
)


class TestBridge:
    """Test Bridge class"""
    
    @pytest.fixture
    async def bridge(self):
        """Create a test bridge instance"""
        bridge = Bridge()
        yield bridge
        await bridge.stop()
    
    def test_bridge_creation(self):
        """Test bridge can be created"""
        bridge = Bridge()
        assert bridge is not None
        assert bridge.running is False
    
    def test_create_fleet(self):
        """Test fleet creation"""
        bridge = Bridge()
        fleet = bridge.create_fleet("test_fleet")
        assert fleet is not None
        assert fleet.name == "test_fleet"
        assert "test_fleet" in bridge.fleets


class TestMessage:
    """Test Message class"""
    
    def test_message_creation(self):
        """Test message can be created"""
        msg = Message(
            header=Header(),
            command=Command(action="test")
        )
        assert msg.header is not None
        assert msg.command.action == "test"
    
    def test_message_defaults(self):
        """Test message has correct defaults"""
        msg = Message()
        assert msg.header is not None
        assert msg.command is None
        assert msg.telemetry is None


class TestRobotFleet:
    """Test RobotFleet class"""
    
    def test_fleet_creation(self):
        """Test fleet can be created"""
        fleet = RobotFleet("test_fleet")
        assert fleet.name == "test_fleet"
        assert len(fleet.robots) == 0
    
    def test_add_robot(self):
        """Test adding robot to fleet"""
        fleet = RobotFleet("test_fleet")
        mock_robot = Mock(spec=Robot)
        mock_robot.robot_id = "robot_1"
        
        fleet.add_robot(mock_robot)
        assert "robot_1" in fleet.robots
    
    def test_get_robot(self):
        """Test getting robot from fleet"""
        fleet = RobotFleet("test_fleet")
        mock_robot = Mock(spec=Robot)
        mock_robot.robot_id = "robot_1"
        
        fleet.add_robot(mock_robot)
        retrieved = fleet.get_robot("robot_1")
        assert retrieved == mock_robot
    
    def test_get_nonexistent_robot(self):
        """Test getting non-existent robot returns None"""
        fleet = RobotFleet("test_fleet")
        retrieved = fleet.get_robot("nonexistent")
        assert retrieved is None


class TestIdentity:
    """Test Identity class"""
    
    def test_identity_creation(self):
        """Test identity can be created"""
        identity = Identity(
            id="test_id",
            name="test_name",
            roles=["admin"]
        )
        assert identity.id == "test_id"
        assert identity.name == "test_name"
        assert "admin" in identity.roles
