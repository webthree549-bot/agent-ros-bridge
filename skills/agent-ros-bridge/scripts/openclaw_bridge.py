"""OpenClaw Real-Time Bridge for Agent ROS Bridge.

Provides bidirectional real-time communication between OpenClaw, Agent ROS Bridge,
and the websocket-based web UI.

Usage:
    python scripts/openclaw_bridge.py [--port 8766]

Features:
    - Real-time telemetry streaming to OpenClaw
    - Command forwarding from OpenClaw to robots
    - Web UI synchronization
    - Event broadcasting to all connected clients
"""

import argparse
import asyncio
import json
import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("openclaw_bridge")


class OpenClawBridge:
    """Bridge between OpenClaw and Agent ROS Bridge.
    
    Handles real-time bidirectional communication:
    - OpenClaw → Robot commands
    - Robot telemetry → OpenClaw
    - Web UI ↔ OpenClaw synchronization
    """
    
    def __init__(self, websocket_port: int = 8766, agent_ros_port: int = 8765):
        self.websocket_port = websocket_port
        self.agent_ros_port = agent_ros_port
        self.openclaw_clients: set = set()
        self.web_clients: set = set()
        self.telemetry_cache: dict = {}
        self.running = False
        
    async def start(self):
        """Start the bridge server."""
        import websockets
        
        self.running = True
        logger.info(f"Starting OpenClaw Bridge on port {self.websocket_port}")
        
        # Start websocket server
        async with websockets.serve(
            self._handle_client,
            "localhost",
            self.websocket_port,
            subprotocols=["openclaw-bridge"]
        ):
            logger.info(f"Bridge listening on ws://localhost:{self.websocket_port}")
            
            # Start telemetry forwarder
            telemetry_task = asyncio.create_task(self._telemetry_loop())
            
            try:
                await asyncio.Future()  # Run forever
            except asyncio.CancelledError:
                logger.info("Bridge shutting down...")
            finally:
                telemetry_task.cancel()
                
    async def _handle_client(self, websocket, path):
        """Handle incoming websocket connections."""
        import websockets
        
        client_type = None
        
        try:
            # Wait for client identification
            message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            data = json.loads(message)
            
            client_type = data.get("client_type", "unknown")
            
            if client_type == "openclaw":
                self.openclaw_clients.add(websocket)
                logger.info("OpenClaw client connected")
                await self._handle_openclaw_client(websocket)
                
            elif client_type == "web_ui":
                self.web_clients.add(websocket)
                logger.info("Web UI client connected")
                await self._handle_web_client(websocket)
                
            else:
                logger.warning(f"Unknown client type: {client_type}")
                
        except asyncio.TimeoutError:
            logger.warning("Client identification timeout")
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {client_type}")
        except Exception as e:
            logger.error(f"Client handler error: {e}")
        finally:
            # Cleanup
            if websocket in self.openclaw_clients:
                self.openclaw_clients.remove(websocket)
            if websocket in self.web_clients:
                self.web_clients.remove(websocket)
                
    async def _handle_openclaw_client(self, websocket):
        """Handle OpenClaw agent connection."""
        import websockets
        
        try:
            async for message in websocket:
                data = json.loads(message)
                action = data.get("action")
                
                if action == "command":
                    # Forward command to Agent ROS Bridge
                    await self._forward_to_agent_ros(data)
                    
                elif action == "query":
                    # Query robot status
                    response = await self._query_agent_ros(data)
                    await websocket.send(json.dumps(response))
                    
                elif action == "subscribe":
                    # Subscribe to telemetry stream
                    await self._subscribe_telemetry(websocket, data)
                    
                else:
                    logger.warning(f"Unknown action from OpenClaw: {action}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("OpenClaw client disconnected")
            
    async def _handle_web_client(self, websocket):
        """Handle Web UI client connection."""
        import websockets
        
        try:
            # Send current state
            await self._broadcast_state(websocket)
            
            async for message in websocket:
                data = json.loads(message)
                action = data.get("action")
                
                if action == "execute":
                    # Execute robot command
                    result = await self._execute_robot_command(data)
                    await websocket.send(json.dumps({
                        "type": "execution_result",
                        "data": result
                    }))
                    
                elif action == "get_telemetry":
                    # Get current telemetry
                    telemetry = self._get_cached_telemetry()
                    await websocket.send(json.dumps({
                        "type": "telemetry",
                        "data": telemetry
                    }))
                    
                elif action == "emergency_stop":
                    # Emergency stop
                    await self._emergency_stop()
                    await self._broadcast_to_all({
                        "type": "emergency_stop",
                        "source": "web_ui",
                        "timestamp": datetime.now().isoformat()
                    })
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("Web UI client disconnected")
            
    async def _telemetry_loop(self):
        """Continuously fetch and broadcast telemetry."""
        while self.running:
            try:
                # Fetch telemetry from Agent ROS Bridge
                telemetry = await self._fetch_telemetry()
                
                if telemetry:
                    self.telemetry_cache = telemetry
                    
                    # Broadcast to all clients
                    message = {
                        "type": "telemetry_update",
                        "data": telemetry,
                        "timestamp": datetime.now().isoformat()
                    }
                    await self._broadcast_to_all(message)
                    
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
                
            await asyncio.sleep(0.5)  # 2Hz update rate
            
    async def _forward_to_agent_ros(self, command: dict):
        """Forward command to Agent ROS Bridge."""
        try:
            import websockets
            
            uri = f"ws://localhost:{self.agent_ros_port}"
            async with websockets.connect(uri) as ws:
                await ws.send(json.dumps(command))
                response = await ws.recv()
                
                # Broadcast result to OpenClaw and Web UI
                await self._broadcast_to_all({
                    "type": "command_result",
                    "command": command,
                    "response": json.loads(response),
                    "timestamp": datetime.now().isoformat()
                })
                
        except Exception as e:
            logger.error(f"Failed to forward to Agent ROS Bridge: {e}")
            
    async def _query_agent_ros(self, query: dict) -> dict:
        """Query Agent ROS Bridge."""
        try:
            import websockets
            
            uri = f"ws://localhost:{self.agent_ros_port}"
            async with websockets.connect(uri) as ws:
                await ws.send(json.dumps(query))
                response = await ws.recv()
                return json.loads(response)
                
        except Exception as e:
            return {"error": str(e)}
            
    async def _execute_robot_command(self, command: dict) -> dict:
        """Execute robot command via Agent ROS Bridge."""
        return await self._query_agent_ros(command)
        
    async def _emergency_stop(self):
        """Trigger emergency stop."""
        await self._forward_to_agent_ros({
            "action": "emergency_stop",
            "priority": "critical"
        })
        
    def _get_cached_telemetry(self) -> dict:
        """Get cached telemetry data."""
        return self.telemetry_cache
        
    async def _fetch_telemetry(self) -> dict | None:
        """Fetch fresh telemetry from Agent ROS Bridge."""
        try:
            response = await self._query_agent_ros({
                "action": "get_telemetry"
            })
            return response.get("data")
        except Exception:
            return None
            
    async def _subscribe_telemetry(self, websocket, data: dict):
        """Subscribe client to telemetry updates."""
        # Telemetry is already broadcast to all clients
        await websocket.send(json.dumps({
            "type": "subscription_confirmed",
            "channel": "telemetry"
        }))
        
    async def _broadcast_state(self, websocket):
        """Send current state to a client."""
        state = {
            "type": "state",
            "telemetry": self.telemetry_cache,
            "connected_clients": {
                "openclaw": len(self.openclaw_clients),
                "web_ui": len(self.web_clients)
            },
            "timestamp": datetime.now().isoformat()
        }
        await websocket.send(json.dumps(state))
        
    async def _broadcast_to_all(self, message: dict):
        """Broadcast message to all connected clients."""
        import websockets
        
        json_message = json.dumps(message)
        
        # Send to OpenClaw clients
        for client in list(self.openclaw_clients):
            try:
                await client.send(json_message)
            except websockets.exceptions.ConnectionClosed:
                self.openclaw_clients.discard(client)
                
        # Send to Web UI clients
        for client in list(self.web_clients):
            try:
                await client.send(json_message)
            except websockets.exceptions.ConnectionClosed:
                self.web_clients.discard(client)


class OpenClawSkillInterface:
    """Interface for OpenClaw to interact with Agent ROS Bridge.
    
    This class provides a synchronous interface for OpenClaw to:
    - Send commands to robots
    - Receive real-time telemetry
    - Query robot status
    """
    
    def __init__(self, bridge_port: int = 8766):
        self.bridge_port = bridge_port
        self._event_queue: asyncio.Queue = asyncio.Queue()
        self._connected = False
        
    async def connect(self):
        """Connect to the OpenClaw Bridge."""
        import websockets
        
        uri = f"ws://localhost:{self.bridge_port}"
        self.websocket = await websockets.connect(
            uri,
            subprotocols=["openclaw-bridge"]
        )
        
        # Identify as OpenClaw
        await self.websocket.send(json.dumps({
            "client_type": "openclaw"
        }))
        
        self._connected = True
        
        # Start event listener
        asyncio.create_task(self._event_listener())
        
    async def _event_listener(self):
        """Listen for events from the bridge."""
        import websockets
        
        try:
            async for message in self.websocket:
                data = json.loads(message)
                await self._event_queue.put(data)
        except websockets.exceptions.ConnectionClosed:
            self._connected = False
            
    async def send_command(self, command: str, **kwargs) -> dict:
        """Send a command to the robot.
        
        Args:
            command: Natural language command (e.g., "move forward")
            **kwargs: Additional parameters
            
        Returns:
            Command result
        """
        if not self._connected:
            raise RuntimeError("Not connected to bridge")
            
        message = {
            "action": "command",
            "command": command,
            "parameters": kwargs
        }
        
        await self.websocket.send(json.dumps(message))
        
        # Wait for result (with timeout)
        try:
            result = await asyncio.wait_for(
                self._wait_for_result(),
                timeout=30.0
            )
            return result
        except asyncio.TimeoutError:
            return {"error": "Command timeout", "command": command}
            
    async def _wait_for_result(self) -> dict:
        """Wait for command result."""
        while True:
            event = await self._event_queue.get()
            if event.get("type") == "command_result":
                return event
                
    async def query_status(self) -> dict:
        """Query robot status."""
        if not self._connected:
            raise RuntimeError("Not connected to bridge")
            
        message = {
            "action": "query",
            "query": "status"
        }
        
        await self.websocket.send(json.dumps(message))
        
        # Wait for response
        response = await asyncio.wait_for(
            self.websocket.recv(),
            timeout=5.0
        )
        return json.loads(response)
        
    async def get_telemetry(self) -> dict:
        """Get current telemetry data."""
        if not self._connected:
            raise RuntimeError("Not connected to bridge")
            
        message = {
            "action": "subscribe"
        }
        
        await self.websocket.send(json.dumps(message))
        
        # Return cached telemetry from events
        events = []
        while not self._event_queue.empty():
            events.append(await self._event_queue.get())
            
        for event in reversed(events):
            if event.get("type") == "telemetry_update":
                return event.get("data", {})
                
        return {}
        
    async def disconnect(self):
        """Disconnect from the bridge."""
        if self._connected:
            await self.websocket.close()
            self._connected = False


def main():
    parser = argparse.ArgumentParser(
        description="OpenClaw Bridge for Agent ROS Bridge"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8766,
        help="Bridge WebSocket port (default: 8766)"
    )
    parser.add_argument(
        "--agent-ros-port",
        type=int,
        default=8765,
        help="Agent ROS Bridge port (default: 8765)"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run a test connection"
    )
    
    args = parser.parse_args()
    
    if args.test:
        # Test mode
        async def test():
            interface = OpenClawSkillInterface(args.port)
            await interface.connect()
            print("Connected to bridge!")
            
            # Query status
            status = await interface.query_status()
            print(f"Robot status: {status}")
            
            await interface.disconnect()
            
        asyncio.run(test())
    else:
        # Start bridge server
        bridge = OpenClawBridge(args.port, args.agent_ros_port)
        
        try:
            asyncio.run(bridge.start())
        except KeyboardInterrupt:
            print("\nShutting down...")
            sys.exit(0)


if __name__ == "__main__":
    main()
