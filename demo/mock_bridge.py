#!/usr/bin/env python3
"""Mock Robot Bridge - Demo/Testing without ROS2

This is a standalone mock for testing the WebSocket API without
requiring ROS2 to be installed. For production use, run_bridge.py
"""

import asyncio
import logging
from datetime import datetime

from agent_ros_bridge import Bridge, Message, Header, Command, Telemetry, Event, Identity
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mock_bridge")


class MockRobot:
    """Simulated TurtleBot for demo purposes"""
    
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


async def handle_message(robot: MockRobot, message: Message, identity: Identity) -> Message:
    """Handle incoming messages"""
    if not message.command:
        return Message(
            header=Header(),
            event=Event(event_type="error", severity="warning", 
                       data={"error": "No command in message"})
        )
    
    logger.info(f"📩 Command from {identity.name}: {message.command.action}")
    result = robot.handle_command(message.command)
    
    if "error" in result:
        return Message(
            header=Header(),
            event=Event(event_type="command_failed", severity="error", data=result)
        )
    
    return Message(
        header=Header(correlation_id=message.header.message_id),
        telemetry=Telemetry(topic="response", data=result)
    )


async def main():
    bridge = Bridge()
    robot = MockRobot()
    
    # Create WebSocket transport with mock handler
    ws_transport = WebSocketTransport({'port': 8765})
    ws_transport.message_handler = lambda msg, ident: handle_message(robot, msg, ident)
    bridge.transport_manager.register(ws_transport)
    
    # Start bridge
    await bridge.start()
    
    print("=" * 60)
    print("🎭 MOCK ROBOT BRIDGE (Demo Mode)")
    print("=" * 60)
    print("WebSocket: ws://localhost:8765")
    print("")
    print("This is a SIMULATED robot for testing.")
    print("No ROS2 required. No real hardware connected.")
    print("")
    print("Test commands:")
    print('  {"command": {"action": "list_robots"}}')
    print('  {"command": {"action": "get_topics"}}')
    print('  {"command": {"action": "move", "parameters": {"direction": "forward", "distance": 1.0}}}')
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print('\n⏹️  Stopping...')
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
