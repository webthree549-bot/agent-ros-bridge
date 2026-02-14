#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WebSocket Transport for OpenClaw Gateway

Bidirectional WebSocket transport for browser-based agents,
dashboards, and web applications.
"""

import asyncio
import json
import logging
from typing import Dict, Any, Optional, Callable
from datetime import datetime

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False

from agent_ros_bridge.gateway_v2.core import (
    Transport, Message, Identity, Header, Command, Telemetry, Event
)

try:
    from agent_ros_bridge.gateway_v2.auth import Authenticator, AuthConfig, RoleBasedAccessControl
    AUTH_AVAILABLE = True
except ImportError:
    AUTH_AVAILABLE = False

logger = logging.getLogger("transport.websocket")


class WebSocketTransport(Transport):
    """WebSocket transport implementation with authentication"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__("websocket", config)
        self.host = config.get("host", "0.0.0.0")
        self.port = config.get("port", 8765)
        self.tls_cert = config.get("tls_cert")
        self.tls_key = config.get("tls_key")
        self.server = None
        self.clients: Dict[str, WebSocketServerProtocol] = {}
        self.identities: Dict[str, Identity] = {}
        
        # Initialize authentication
        auth_config = config.get("auth", {})
        self.auth_enabled = auth_config.get("enabled", False)
        if AUTH_AVAILABLE and self.auth_enabled:
            self.authenticator = Authenticator(AuthConfig(
                enabled=True,
                jwt_secret=auth_config.get("jwt_secret"),
                api_keys=auth_config.get("api_keys", {})
            ))
            self.rbac = RoleBasedAccessControl()
            logger.info("WebSocket authentication enabled")
        else:
            self.authenticator = None
            self.rbac = None
    
    async def start(self) -> bool:
        """Start WebSocket server"""
        if not WEBSOCKETS_AVAILABLE:
            logger.error("websockets library not installed. Run: pip install websockets")
            return False
        
        import ssl
        
        ssl_context = None
        if self.tls_cert and self.tls_key:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(self.tls_cert, self.tls_key)
            logger.info(f"WebSocket TLS enabled on wss://{self.host}:{self.port}")
        else:
            logger.info(f"WebSocket starting on ws://{self.host}:{self.port}")
        
        self.server = await websockets.serve(
            self._handle_client,
            self.host,
            self.port,
            ssl=ssl_context
        )
        
        self.running = True
        logger.info(f"WebSocket transport started on {self.host}:{self.port}")
        return True
    
    async def stop(self) -> None:
        """Stop WebSocket server"""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        
        # Close all client connections
        for client_id, ws in list(self.clients.items()):
            await ws.close()
        
        self.clients.clear()
        self.identities.clear()
        self.running = False
        logger.info("WebSocket transport stopped")
    
    async def _handle_client(self, websocket: WebSocketServerProtocol):
        """Handle client connection with authentication"""
        client_id = str(id(websocket))
        
        # Check authentication if enabled
        auth_payload = None
        if self.authenticator:
            # Try to get token from path query string
            path = getattr(websocket, 'path', '/')
            token = self.authenticator.extract_token_from_query(path.split('?')[1] if '?' in path else '')
            
            # Try API key from headers if no token
            if not token:
                headers = dict(websocket.request_headers) if hasattr(websocket, 'request_headers') else {}
                api_key = self.authenticator.extract_api_key_from_headers(headers)
                if api_key:
                    auth_payload = self.authenticator.verify_api_key(api_key)
            
            # Verify token
            if token and not auth_payload:
                auth_payload = self.authenticator.verify_token(token)
            
            # Reject if authentication required but failed
            if not auth_payload:
                logger.warning(f"Authentication failed for client {client_id}")
                await websocket.close(code=4001, reason="Authentication required")
                return
        
        self.clients[client_id] = websocket
        
        # Create identity with auth info
        roles = auth_payload.get("roles", ["anonymous"]) if auth_payload else ["anonymous"]
        user_id = auth_payload.get("sub", "anonymous") if auth_payload else "anonymous"
        
        identity = Identity(
            id=client_id,
            name=user_id,
            roles=roles,
            metadata={
                "transport": "websocket",
                "remote_addr": str(websocket.remote_address),
                "path": getattr(websocket, 'path', '/'),
                "auth": auth_payload is not None
            }
        )
        self.identities[client_id] = identity
        
        logger.info(f"WebSocket client connected: {client_id} (user: {user_id}, roles: {roles})")
        
        try:
            async for message in websocket:
                try:
                    logger.debug(f"Raw message from {client_id}: {repr(message)}")
                    data = json.loads(message)
                    msg = self._json_to_message(data)
                    
                    # Check RBAC permissions
                    if self.rbac and msg.command:
                        action = msg.command.action
                        if not self.rbac.can_execute(identity.roles, action):
                            logger.warning(f"Permission denied: {identity.name} cannot {action}")
                            await websocket.send(json.dumps({
                                "error": "Permission denied",
                                "action": action,
                                "required_roles": list(self.rbac.permissions.keys())
                            }))
                            continue
                    
                    if self.message_handler:
                        response = await self.message_handler(msg, identity)
                        if response:
                            # Filter response based on roles
                            if self.rbac and response.telemetry:
                                filtered_data = self.rbac.filter_response(
                                    identity.roles, 
                                    response.telemetry.data
                                )
                                response.telemetry.data = filtered_data
                            await websocket.send(self._message_to_json(response))
                            
                except json.JSONDecodeError as e:
                    logger.warning(f"Invalid JSON from {client_id}: {e}, raw: {repr(message[:100])}")
                    await websocket.send(json.dumps({
                        "error": "Invalid JSON",
                        "type": "parse_error",
                        "details": str(e)
                    }))
                except Exception as e:
                    logger.error(f"Error handling message from {client_id}: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"WebSocket client disconnected: {client_id}")
        finally:
            del self.clients[client_id]
            del self.identities[client_id]
    
    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to specific client"""
        if recipient not in self.clients:
            logger.warning(f"Recipient not found: {recipient}")
            return False
        
        try:
            ws = self.clients[recipient]
            await ws.send(self._message_to_json(message))
            return True
        except Exception as e:
            logger.error(f"Failed to send to {recipient}: {e}")
            return False
    
    async def broadcast(self, message: Message) -> list:
        """Broadcast to all connected clients"""
        if not self.clients:
            return []
        
        json_msg = self._message_to_json(message)
        results = []
        
        for client_id, ws in list(self.clients.items()):
            try:
                await ws.send(json_msg)
                results.append(client_id)
            except Exception as e:
                logger.warning(f"Failed to broadcast to {client_id}: {e}")
        
        return results
    
    def _json_to_message(self, data: Dict[str, Any]) -> Message:
        """Convert JSON to Message"""
        header_data = data.get("header", {})
        header = Header(
            message_id=header_data.get("message_id", ""),
            timestamp=datetime.fromisoformat(header_data.get("timestamp", datetime.utcnow().isoformat())),
            source=header_data.get("source", ""),
            target=header_data.get("target", ""),
            correlation_id=header_data.get("correlation_id")
        )
        
        # Parse command
        command = None
        if "command" in data:
            cmd_data = data["command"]
            command = Command(
                action=cmd_data.get("action", ""),
                parameters=cmd_data.get("parameters", {}),
                timeout_ms=cmd_data.get("timeout_ms", 5000),
                priority=cmd_data.get("priority", 5)
            )
        
        # Parse telemetry
        telemetry = None
        if "telemetry" in data:
            telem_data = data["telemetry"]
            telemetry = Telemetry(
                topic=telem_data.get("topic", ""),
                data=telem_data.get("data"),
                quality=telem_data.get("quality", 1.0)
            )
        
        # Parse event
        event = None
        if "event" in data:
            event_data = data["event"]
            event = Event(
                event_type=event_data.get("type", ""),
                severity=event_data.get("severity", "info"),
                data=event_data.get("data", {})
            )
        
        return Message(
            header=header,
            command=command,
            telemetry=telemetry,
            event=event,
            metadata=data.get("metadata", {})
        )
    
    def _message_to_json(self, message: Message) -> str:
        """Convert Message to JSON"""
        data = {
            "header": {
                "message_id": message.header.message_id,
                "timestamp": message.header.timestamp.isoformat(),
                "source": message.header.source,
                "target": message.header.target,
            },
            "metadata": message.metadata
        }
        
        if message.header.correlation_id:
            data["header"]["correlation_id"] = message.header.correlation_id
        
        if message.command:
            data["command"] = {
                "action": message.command.action,
                "parameters": message.command.parameters,
                "timeout_ms": message.command.timeout_ms,
                "priority": message.command.priority
            }
        
        if message.telemetry:
            data["telemetry"] = {
                "topic": message.telemetry.topic,
                "data": message.telemetry.data,
                "quality": message.telemetry.quality
            }
        
        if message.event:
            data["event"] = {
                "type": message.event.event_type,
                "severity": message.event.severity,
                "data": message.event.data
            }
        
        return json.dumps(data)


# Example usage
async def example_server():
    """Example WebSocket server"""
    from agent_ros_bridge.gateway_v2.core import Bridge
    
    gateway = Bridge()
    
    # Create WebSocket transport
    ws_transport = WebSocketTransport({
        "host": "0.0.0.0",
        "port": 8765
    })
    
    gateway.transport_manager.register(ws_transport)
    
    async with gateway.run():
        logger.info("WebSocket server running. Connect with ws://localhost:8765")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    import asyncio
    asyncio.run(example_server())
