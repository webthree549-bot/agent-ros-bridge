#!/usr/bin/env python3
"""Simulated Robot Bridge - Demo/Testing without ROS2.

This is a standalone simulated for testing the WebSocket API without
requiring ROS2 to be installed. For production use, run_bridge.py
"""

import asyncio
import logging
from datetime import datetime

from agent_ros_bridge import Bridge, Event, Header, Identity, Message, Plugin, Telemetry
from agent_ros_bridge.gateway_v2.transports.websocket import WebSocketTransport

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("simulated_bridge")


class SimulatedRobotPlugin(Plugin):
    """Plugin that simulates a robot for testing."""

    name = "simulated_robot"
    version = "1.0.0"

    def __init__(self):
        self.topics = ["/cmd_vel", "/odom", "/scan", "/joint_states", "/tf", "/battery_state"]
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.battery = 85.0
        self.speed = 0.0

    async def initialize(self, gateway) -> None:
        """Initialize plugin."""
        logger.info("Simulated robot plugin initialized")
        return None

    async def shutdown(self) -> None:
        """Shutdown plugin."""
        logger.info("Simulated robot plugin shutdown")

    async def handle_message(self, message: Message, identity: Identity) -> Message:
        """Handle incoming commands."""
        if not message.command:
            return None

        cmd = message.command
        logger.info(f"📩 Command from {identity.name}: {cmd.action}")

        if cmd.action == "list_robots":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="robots",
                    data={
                        "robots": [
                            {
                                "id": "turtlebot_01",
                                "name": "TurtleBot4 Simulated",
                                "type": "ros2_simulated",
                                "connected": True,
                                "battery": self.battery,
                            }
                        ]
                    },
                ),
            )

        elif cmd.action == "get_topics":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(topic="topics", data={"topics": self.topics}),
            )

        elif cmd.action == "get_robot_state":
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="robot_state",
                    data={
                        "position": self.position,
                        "battery": self.battery,
                        "speed": self.speed,
                        "timestamp": datetime.utcnow().isoformat(),
                    },
                ),
            )

        elif cmd.action == "publish":
            topic = cmd.parameters.get("topic")
            data = cmd.parameters.get("data", {})

            if topic == "/cmd_vel":
                linear = data.get("linear", {})
                self.speed = linear.get("x", 0.0)
                logger.info(f"🚀 Robot moving: speed={self.speed:.2f} m/s")
                return Message(
                    header=Header(correlation_id=message.header.message_id),
                    telemetry=Telemetry(
                        topic="result",
                        data={"published": True, "topic": topic, "speed": self.speed},
                    ),
                )

            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(topic="result", data={"published": True, "topic": topic}),
            )

        elif cmd.action == "move":
            direction = cmd.parameters.get("direction", "forward")
            distance = cmd.parameters.get("distance", 0.0)
            logger.info(f"🤖 Move {direction} {distance}m")
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(
                    topic="result",
                    data={"moving": True, "direction": direction, "distance": distance},
                ),
            )

        elif cmd.action == "rotate":
            angle = cmd.parameters.get("angle", 0.0)
            logger.info(f"🔄 Rotate {angle} degrees")
            return Message(
                header=Header(correlation_id=message.header.message_id),
                telemetry=Telemetry(topic="result", data={"rotating": True, "angle": angle}),
            )

        # Unknown command - return error
        return Message(
            header=Header(correlation_id=message.header.message_id),
            event=Event(
                event_type="unknown_command",
                severity="warning",
                data={"action": cmd.action, "error": "Unknown command"},
            ),
        )


async def main():
    bridge = Bridge()

    # Register WebSocket transport
    bridge.transport_manager.register(WebSocketTransport({"port": 8766}))

    # Register simulated robot plugin
    simulated_plugin = SimulatedRobotPlugin()
    await bridge.plugin_manager.load_plugin(simulated_plugin)

    # Start bridge
    await bridge.start()

    print("=" * 60)
    print("🎭 SIMULATED ROBOT BRIDGE (Demo Mode)")
    print("=" * 60)
    print("WebSocket: ws://localhost:8766")
    print("")
    print("This is a SIMULATED robot for testing.")
    print("No ROS2 required. No real hardware connected.")
    print("")
    print("Test commands:")
    print('  {"command": {"action": "list_robots"}}')
    print('  {"command": {"action": "get_topics"}}')
    print(
        '  {"command": {"action": "move", "parameters": {"direction": "forward", "distance": 1.0}}}'
    )
    print("")
    print("Press Ctrl+C to stop")
    print("=" * 60)

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️  Stopping...")
        await bridge.stop()


if __name__ == "__main__":
    asyncio.run(main())
