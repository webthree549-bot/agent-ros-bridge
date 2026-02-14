#!/usr/bin/env python3
"""Agent ROS Bridge - WebSocket to ROS2 (with mock fallback)"""

import asyncio
import logging
import os
from datetime import datetime

from agent_ros_bridge import Bridge, Message, Header, Command, Telemetry, Event, Identity
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_ros_bridge")


class MockROS2Connector:
    """Mock ROS2 connector for testing without ROS2 installed"""
    
    def __init__(self, config=None):
        self.name = "ros2_mock"
        self.config = config or {}
        self.running = False
        self._robot = MockRobot()
    
    async def start(self) -> bool:
        self.running = True
        logger.info("🎭 Mock ROS2 connector started")
        return True
    
    async def stop(self) -> None:
        self.running = False
    
    async def handle_message(self, message: Message, identity: Identity) -> Message:
        """Handle robot commands"""
        if not message.command:
            return Message(
                header=Header(),
                event=Event(event_type="error", severity="warning", 
                           data={"error": "No command in message"})
            )
        
        result = self._robot.handle_command(message.command)
        
        if "error" in result:
            return Message(
                header=Header(),
                event=Event(event_type="command_failed", severity="error", data=result)
            )
        
        return Message(
            header=Header(correlation_id=message.header.message_id),
            telemetry=Telemetry(topic="response", data=result)
        )


class MockRobot:
    """Simulated TurtleBot for testing"""
    
    def __init__(self):
        self.connected = True
        self.topics = [
            "/cmd_vel", "/odom", "/scan", "/joint_states", "/tf", "/battery_state"
        ]
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.battery = 85.0
        self.speed = 0.0
        
    def handle_command(self, cmd: Command) -> dict:
        action = cmd.action
        params = cmd.parameters
        
        if action == "list_robots":
            return {"robots": [{"id": "turtlebot_01", "name": "TurtleBot4 Mock", 
                               "type": "ros2_mock", "connected": True, "battery": self.battery}]}
        
        elif action == "get_topics":
            return {"topics": self.topics}
        
        elif action == "get_robot_state":
            return {
                "position": self.position,
                "battery": self.battery,
                "speed": self.speed,
                "timestamp": datetime.utcnow().isoformat()
            }
        
        elif action == "publish":
            topic = params.get("topic")
            data = params.get("data", {})
            
            if topic == "/cmd_vel":
                linear = data.get("linear", {})
                self.speed = linear.get("x", 0.0)
                logger.info(f"🚀 Robot moving: speed={self.speed:.2f} m/s")
                return {"published": True, "topic": topic, "speed": self.speed}
            
            return {"published": True, "topic": topic}
        
        elif action == "move":
            direction = params.get("direction", "forward")
            distance = params.get("distance", 0.0)
            logger.info(f"🤖 Move {direction} {distance}m")
            return {"moving": True, "direction": direction, "distance": distance}
        
        elif action == "rotate":
            angle = params.get("angle", 0.0)
            logger.info(f"🔄 Rotate {angle} degrees")
            return {"rotating": True, "angle": angle}
        
        else:
            return {"error": f"Unknown action: {action}"}


async def message_handler(message: Message, identity: Identity) -> Message:
    """Route messages to appropriate connector"""
    if not message.command:
        return Message(
            header=Header(),
            event=Event(event_type="error", severity="warning", 
                       data={"error": "No command in message"})
        )
    
    # For now, route all robot commands to the handler
    # In full implementation, this would route to ROS2Connector
    return Message(
        header=Header(correlation_id=message.header.message_id),
        telemetry=Telemetry(topic="response", data={"status": "received", "action": message.command.action})
    )


async def main():
    bridge = Bridge()
    
    # Check for ROS2
    try:
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
        ros2_available = True
    except ImportError:
        ros2_available = False
    
    # Check environment
    force_mock = os.environ.get("MOCK_MODE", "").lower() in ("true", "1", "yes")
    use_real_ros2 = ros2_available and not force_mock
    
    # Register WebSocket transport
    ws_transport = WebSocketTransport({'port': 8765})
    bridge.transport_manager.register(ws_transport)
    
    if use_real_ros2:
        from agent_ros_bridge.gateway_v2.connectors.ros2_connector import ROS2Connector
        connector = ROS2Connector({'auto_discover': True})
        mode = "ROS2"
        logger.info("✅ Using real ROS2 connector")
    else:
        connector = MockROS2Connector({'auto_discover': True})
        ws_transport.message_handler = connector.handle_message
        mode = "MOCK"
        if force_mock:
            logger.info("🎭 Using mock mode (MOCK_MODE=true)")
        else:
            logger.info("🎭 Using mock mode (ROS2 not available)")
    
    bridge.connector_registry.register(connector)
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print(f"🤖 Agent ROS Bridge Running [{mode} MODE]")
    print("=" * 60)
    print("WebSocket: ws://localhost:8765")
    print("")
    print("Test commands:")
    print('  {"command": {"action": "list_robots"}}')
    print('  {"command": {"action": "get_topics"}}')
    print('  {"command": {"action": "publish", "parameters": {"topic": "/cmd_vel", "data": {"linear": {"x": 0.5}}}}}}')
    print("")
    if mode == "MOCK":
        print("Note: Running in mock mode. No real ROS2 connection.")
        print("      Set MOCK_MODE=true to force mock, or install ROS2 for real mode.")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    # Keep running
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print('\n⏹️  Stopping bridge...')
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
