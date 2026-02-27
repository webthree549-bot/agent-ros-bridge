"""
Unit tests for core message data classes and routing logic.
Tests Message, Header, Command, Telemetry, Event, Identity, and QoS.
"""

import uuid
import json
from datetime import datetime, timedelta
from dataclasses import asdict

import pytest

from agent_ros_bridge.gateway_v2.core import (
    Message,
    Header,
    Command,
    Telemetry,
    Event,
    Identity,
    QoS,
)


class TestQoSEnum:
    """Test Quality of Service enum"""
    
    def test_qos_values_exist(self):
        """Test that all QoS levels are defined"""
        assert QoS.BEST_EFFORT is not None
        assert QoS.AT_LEAST_ONCE is not None
        assert QoS.EXACTLY_ONCE is not None
    
    def test_qos_auto_values(self):
        """Test that QoS enum auto-generates unique values"""
        assert QoS.BEST_EFFORT != QoS.AT_LEAST_ONCE
        assert QoS.AT_LEAST_ONCE != QoS.EXACTLY_ONCE
        assert QoS.BEST_EFFORT != QoS.EXACTLY_ONCE


class TestIdentity:
    """Test Identity data class"""
    
    def test_identity_creation(self):
        """Test basic identity creation"""
        identity = Identity(
            id="user_123",
            name="Test User",
            roles=["operator", "viewer"]
        )
        
        assert identity.id == "user_123"
        assert identity.name == "Test User"
        assert "operator" in identity.roles
        assert "viewer" in identity.roles
    
    def test_identity_defaults(self):
        """Test identity default values"""
        identity = Identity(id="minimal_user", name="Minimal")
        
        assert identity.roles == []
        assert identity.metadata == {}
    
    def test_identity_with_metadata(self):
        """Test identity with metadata"""
        identity = Identity(
            id="user_456",
            name="Advanced User",
            roles=["admin"],
            metadata={
                "department": "robotics",
                "level": 5,
                "features": ["emergency_stop", "config_edit"]
            }
        )
        
        assert identity.metadata["department"] == "robotics"
        assert identity.metadata["level"] == 5
        assert "emergency_stop" in identity.metadata["features"]
    
    def test_identity_has_role(self):
        """Test helper method for checking roles"""
        identity = Identity(
            id="user_789",
            name="Role Tester",
            roles=["admin", "operator"]
        )
        
        # Would implement: assert identity.has_role("admin") is True
        # For now, just verify roles list
        assert "admin" in identity.roles
        assert "viewer" not in identity.roles


class TestHeader:
    """Test Header data class"""
    
    def test_header_default_values(self):
        """Test header default values"""
        header = Header()
        
        # message_id should be auto-generated UUID
        assert header.message_id is not None
        assert len(header.message_id) == 36  # UUID string length
        
        # timestamp should be recent
        assert header.timestamp is not None
        assert (datetime.utcnow() - header.timestamp) < timedelta(seconds=1)
        
        # Other fields should be empty
        assert header.source == ""
        assert header.target == ""
        assert header.correlation_id is None
    
    def test_header_custom_values(self):
        """Test header with custom values"""
        test_id = str(uuid.uuid4())
        test_time = datetime(2024, 1, 15, 10, 30, 0)
        
        header = Header(
            message_id=test_id,
            timestamp=test_time,
            source="robot_1",
            target="control_center",
            correlation_id="corr_123"
        )
        
        assert header.message_id == test_id
        assert header.timestamp == test_time
        assert header.source == "robot_1"
        assert header.target == "control_center"
        assert header.correlation_id == "corr_123"
    
    def test_header_unique_message_ids(self):
        """Test that each header gets a unique message_id"""
        headers = [Header() for _ in range(100)]
        ids = [h.message_id for h in headers]
        
        assert len(set(ids)) == 100  # All unique
    
    def test_header_timestamp_immutability(self):
        """Test that timestamp is captured at creation time"""
        before = datetime.utcnow()
        header = Header()
        after = datetime.utcnow()
        
        assert before <= header.timestamp <= after


class TestCommand:
    """Test Command data class"""
    
    def test_command_creation(self):
        """Test basic command creation"""
        cmd = Command(
            action="move",
            parameters={"x": 10, "y": 20, "speed": 1.5}
        )
        
        assert cmd.action == "move"
        assert cmd.parameters["x"] == 10
        assert cmd.parameters["y"] == 20
    
    def test_command_defaults(self):
        """Test command default values"""
        cmd = Command(action="test")
        
        assert cmd.parameters == {}
        assert cmd.timeout_ms == 5000
        assert cmd.priority == 5
    
    def test_command_priority_range(self):
        """Test command priority levels"""
        # Lower number = higher priority
        emergency = Command(action="stop", priority=1)
        normal = Command(action="move", priority=5)
        background = Command(action="log", priority=10)
        
        assert emergency.priority < normal.priority < background.priority
    
    def test_command_timeout(self):
        """Test command timeout configuration"""
        quick_cmd = Command(action="ping", timeout_ms=100)
        slow_cmd = Command(action="navigate", timeout_ms=60000)
        
        assert quick_cmd.timeout_ms == 100
        assert slow_cmd.timeout_ms == 60000
    
    def test_command_complex_parameters(self):
        """Test command with nested parameters"""
        cmd = Command(
            action="navigate_to_pose",
            parameters={
                "pose": {
                    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                },
                "obstacle_avoidance": True,
                "max_speed": 0.5
            }
        )
        
        assert cmd.parameters["pose"]["position"]["x"] == 1.0
        assert cmd.parameters["obstacle_avoidance"] is True


class TestTelemetry:
    """Test Telemetry data class"""
    
    def test_telemetry_creation(self):
        """Test basic telemetry creation"""
        telemetry = Telemetry(
            topic="/robot/odom",
            data={"x": 1.5, "y": 2.5, "theta": 0.5}
        )
        
        assert telemetry.topic == "/robot/odom"
        assert telemetry.data["x"] == 1.5
    
    def test_telemetry_defaults(self):
        """Test telemetry default values"""
        telemetry = Telemetry(topic="/test", data={})
        
        assert telemetry.quality == 1.0  # Full quality by default
    
    def test_telemetry_quality_levels(self):
        """Test telemetry quality levels"""
        perfect = Telemetry(topic="/a", data={}, quality=1.0)
        good = Telemetry(topic="/b", data={}, quality=0.8)
        poor = Telemetry(topic="/c", data={}, quality=0.3)
        
        assert perfect.quality == 1.0
        assert good.quality == 0.8
        assert poor.quality == 0.3
    
    def test_telemetry_various_data_types(self):
        """Test telemetry with different data types"""
        # Numeric data
        numeric = Telemetry(topic="/numeric", data={"value": 42})
        assert numeric.data["value"] == 42
        
        # String data
        string = Telemetry(topic="/string", data={"status": "active"})
        assert string.data["status"] == "active"
        
        # Array data
        array = Telemetry(topic="/array", data={"points": [1, 2, 3]})
        assert len(array.data["points"]) == 3
        
        # Complex nested data
        complex_data = Telemetry(
            topic="/complex",
            data={
                "laser_scan": {
                    "ranges": [1.0, 1.1, 1.2],
                    "intensities": [100, 100, 100],
                    "angle_min": -1.57,
                    "angle_max": 1.57
                }
            }
        )
        assert len(complex_data.data["laser_scan"]["ranges"]) == 3


class TestEvent:
    """Test Event data class"""
    
    def test_event_creation(self):
        """Test basic event creation"""
        event = Event(
            event_type="robot_connected",
            data={"robot_id": "robot_1", "ip": "192.168.1.10"}
        )
        
        assert event.event_type == "robot_connected"
        assert event.data["robot_id"] == "robot_1"
    
    def test_event_defaults(self):
        """Test event default values"""
        event = Event(event_type="test_event")
        
        assert event.severity == "info"
        assert event.data == {}
    
    def test_event_severity_levels(self):
        """Test event severity levels"""
        severities = ["debug", "info", "warning", "error", "critical"]
        
        for sev in severities:
            event = Event(event_type="test", severity=sev)
            assert event.severity == sev
    
    def test_event_error_with_stacktrace(self):
        """Test error event with stacktrace"""
        event = Event(
            event_type="runtime_error",
            severity="error",
            data={
                "error": "Connection timeout",
                "stacktrace": [
                    "File '/app/connector.py', line 42",
                    "File '/app/transport.py', line 100"
                ],
                "retry_count": 3
            }
        )
        
        assert event.severity == "error"
        assert len(event.data["stacktrace"]) == 2


class TestMessage:
    """Test Message data class"""
    
    def test_message_default_values(self):
        """Test message default values"""
        msg = Message()
        
        assert msg.header is not None
        assert msg.command is None
        assert msg.telemetry is None
        assert msg.event is None
    
    def test_message_with_command(self):
        """Test message containing a command"""
        msg = Message(
            header=Header(message_id="cmd_123"),
            command=Command(
                action="move_forward",
                parameters={"distance": 1.0}
            )
        )
        
        assert msg.command.action == "move_forward"
        assert msg.header.message_id == "cmd_123"
    
    def test_message_with_telemetry(self):
        """Test message containing telemetry"""
        msg = Message(
            header=Header(message_id="tel_456"),
            telemetry=Telemetry(
                topic="/battery",
                data={"level": 85.5, "voltage": 12.4}
            )
        )
        
        assert msg.telemetry.topic == "/battery"
        assert msg.command is None  # No command in telemetry message
    
    def test_message_with_event(self):
        """Test message containing an event"""
        msg = Message(
            header=Header(message_id="evt_789"),
            event=Event(
                event_type="emergency_stop_triggered",
                severity="critical",
                data={"reason": "obstacle_detected"}
            )
        )
        
        assert msg.event.severity == "critical"
        assert msg.event.data["reason"] == "obstacle_detected"
    
    def test_message_correlation_id_propagation(self):
        """Test that correlation_id links request and response"""
        request_id = str(uuid.uuid4())
        
        # Create request
        request = Message(
            header=Header(message_id=request_id),
            command=Command(action="get_status")
        )
        
        # Create response with correlation to request
        response = Message(
            header=Header(
                message_id=str(uuid.uuid4()),
                correlation_id=request_id
            ),
            telemetry=Telemetry(topic="status", data={"ready": True})
        )
        
        assert response.header.correlation_id == request.header.message_id
    
    def test_message_complete_flow(self):
        """Test complete message flow from command to response"""
        # 1. Client sends command
        client_identity = Identity(id="client_1", name="Control Center", roles=["operator"])
        command_msg = Message(
            header=Header(
                message_id="req_001",
                source="client_1",
                target="robot_1"
            ),
            command=Command(
                action="navigate_to",
                parameters={"x": 10, "y": 5},
                priority=2
            )
        )
        
        # 2. Server processes and sends response
        response_msg = Message(
            header=Header(
                message_id="resp_001",
                source="robot_1",
                target="client_1",
                correlation_id=command_msg.header.message_id
            ),
            telemetry=Telemetry(
                topic="navigation_result",
                data={"status": "started", "eta": 30}
            )
        )
        
        # Verify the flow
        assert response_msg.header.correlation_id == command_msg.header.message_id
        assert response_msg.header.target == command_msg.header.source
        assert response_msg.header.source == command_msg.header.target


class TestMessageDataclassFeatures:
    """Test dataclass-specific features"""
    
    def test_message_asdict_conversion(self):
        """Test converting message to dictionary"""
        msg = Message(
            header=Header(message_id="test_123"),
            command=Command(action="test")
        )
        
        data = asdict(msg)
        
        assert data["header"]["message_id"] == "test_123"
        assert data["command"]["action"] == "test"
    
    def test_message_equality(self):
        """Test message equality comparison"""
        msg1 = Message(header=Header(message_id="same_id"))
        msg2 = Message(header=Header(message_id="same_id"))
        msg3 = Message(header=Header(message_id="different_id"))
        
        # Note: These will not be equal because Header() creates new timestamps
        # This test documents the current behavior
        assert msg1.header.message_id == msg2.header.message_id


class TestMessageValidationEdgeCases:
    """Test edge cases and validation scenarios"""
    
    def test_empty_command_parameters(self):
        """Test command with empty parameters"""
        cmd = Command(action="ping", parameters={})
        assert cmd.parameters == {}
    
    def test_telemetry_with_none_data(self):
        """Test telemetry handling of None data"""
        # This might be an edge case to handle
        telemetry = Telemetry(topic="/test", data=None)
        assert telemetry.data is None
        assert telemetry.quality == 1.0
    
    def test_event_with_empty_type(self):
        """Test event with empty event type"""
        event = Event(event_type="")
        assert event.event_type == ""
        assert event.severity == "info"
    
    def test_identity_with_empty_roles(self):
        """Test identity with no roles"""
        identity = Identity(id="public_user", name="Public")
        assert identity.roles == []
        # Should this user have any access? Documentation question.


class TestQoSWithMessages:
    """Test QoS integration with messages"""
    
    def test_command_with_qos(self):
        """Test that commands can have associated QoS"""
        # If we add QoS field to Command in future
        cmd = Command(action="critical_stop", priority=1)
        # High priority commands might imply EXACTLY_ONCE
        assert cmd.priority == 1
    
    def test_telemetry_with_qos_implication(self):
        """Test telemetry QoS implications"""
        # Best effort telemetry
        best_effort = Telemetry(topic="/debug", data={}, quality=0.5)
        # Critical telemetry
        critical = Telemetry(topic="/emergency", data={}, quality=1.0)
        
        assert critical.quality > best_effort.quality
