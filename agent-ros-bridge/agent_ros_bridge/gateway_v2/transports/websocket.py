"""WebSocket transport for Agent ROS Bridge.

Supports local and remote connections with optional TLS encryption.
"""

import asyncio
import json
import logging
import ssl
from typing import Any, Callable, Dict, Optional, Set

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False

logger = logging.getLogger(__name__)


class WebSocketTransport:
    """WebSocket transport for real-time agent communication.
    
    Supports both local and remote connections. For remote/distributed
    deployments, use TLS encryption and JWT authentication.
    
    Examples:
        # Local development (no TLS)
        WebSocketTransport({
            "host": "localhost",
            "port": 8765,
            "auth": {"enabled": False}
        })
        
        # Remote edge server (with TLS)
        WebSocketTransport({
            "host": "0.0.0.0",
            "port": 8765,
            "tls": {
                "cert": "/etc/ssl/certs/server.crt",
                "key": "/etc/ssl/private/server.key"
            },
            "auth": {
                "enabled": True,
                "jwt_secret": "${JWT_SECRET}"
            }
        })
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize WebSocket transport.
        
        Args:
            config: Configuration dict with:
                - host: Bind address (default: "0.0.0.0")
                - port: Port number (default: 8765)
                - tls: Optional TLS config with cert/key paths
                - auth: Auth config with enabled flag and jwt_secret
                - cors: Optional CORS origins for browser clients
                - ping_interval: Keepalive ping interval in seconds (default: 20)
                - ping_timeout: Ping timeout in seconds (default: 10)
        """
        self.config = config
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 8765)
        self.tls_config = config.get('tls')
        self.auth_config = config.get('auth', {})
        self.auth_enabled = self.auth_config.get('enabled', False)
        self.cors_origins = config.get('cors', [])
        self.ping_interval = config.get('ping_interval', 20)
        self.ping_timeout = config.get('ping_timeout', 10)
        
        self._server = None
        self._clients: Dict[WebSocketServerProtocol, Dict[str, Any]] = {}
        self._message_handlers: List[Callable] = []
        self._running = False
        self._ssl_context: Optional[ssl.SSLContext] = None
        
        if not WEBSOCKETS_AVAILABLE:
            logger.warning("websockets not installed. WebSocket transport disabled.")
    
    def _load_ssl_context(self) -> Optional[ssl.SSLContext]:
        """Load SSL context from TLS configuration.
        
        Returns:
            SSLContext if TLS is configured, None otherwise
        """
        if not self.tls_config:
            return None
        
        cert_path = self.tls_config.get('cert')
        key_path = self.tls_config.get('key')
        
        if not cert_path or not key_path:
            logger.warning("TLS config missing cert or key path")
            return None
        
        try:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(cert_path, key_path)
            logger.info(f"Loaded TLS certificate from {cert_path}")
            return ssl_context
        except Exception as e:
            logger.error(f"Failed to load TLS certificate: {e}")
            raise
    
    def on_message(self, handler: Callable):
        """Register a message handler.
        
        Handler signature: handler(websocket, data) -> None
        """
        self._message_handlers.append(handler)
        return handler
    
    def on_connect(self, handler: Callable):
        """Register a connection handler.
        
        Handler signature: handler(websocket, client_info) -> None
        """
        if not hasattr(self, '_connect_handlers'):
            self._connect_handlers = []
        self._connect_handlers.append(handler)
        return handler
    
    def on_disconnect(self, handler: Callable):
        """Register a disconnection handler.
        
        Handler signature: handler(websocket, client_info) -> None
        """
        if not hasattr(self, '_disconnect_handlers'):
            self._disconnect_handlers = []
        self._disconnect_handlers.append(handler)
        return handler
    
    async def _handle_client(self, websocket: WebSocketServerProtocol, path: str):
        """Handle a new WebSocket connection."""
        if not WEBSOCKETS_AVAILABLE:
            return
        
        client_info = {
            "remote_address": websocket.remote_address,
            "user_agent": websocket.request_headers.get('User-Agent', 'unknown'),
            "connected_at": asyncio.get_event_loop().time(),
            "authenticated": not self.auth_enabled
        }
        
        self._clients[websocket] = client_info
        logger.info(f"WebSocket client connected: {client_info['remote_address']}")
        
        # Call connect handlers
        for handler in getattr(self, '_connect_handlers', []):
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(websocket, client_info)
                else:
                    handler(websocket, client_info)
            except Exception as e:
                logger.error(f"Connect handler error: {e}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self._process_message(websocket, data, client_info)
                except json.JSONDecodeError:
                    await self._send_error(websocket, "Invalid JSON")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
                    await self._send_error(websocket, str(e))
        except websockets.exceptions.ConnectionClosed as e:
            logger.info(f"WebSocket client disconnected: {client_info['remote_address']} (code: {e.code})")
        finally:
            # Call disconnect handlers
            for handler in getattr(self, '_disconnect_handlers', []):
                try:
                    if asyncio.iscoroutinefunction(handler):
                        await handler(websocket, client_info)
                    else:
                        handler(websocket, client_info)
                except Exception as e:
                    logger.error(f"Disconnect handler error: {e}")
            
            del self._clients[websocket]
    
    async def _process_message(
        self, 
        websocket: WebSocketServerProtocol, 
        data: Dict,
        client_info: Dict[str, Any]
    ):
        """Process incoming message."""
        msg_type = data.get('type', 'unknown')
        
        # Handle authentication if enabled
        if self.auth_enabled and not client_info.get('authenticated'):
            if msg_type == 'auth':
                token = data.get('token')
                agent_id = data.get('agent_id', 'unknown')
                
                if self._verify_token(token, agent_id):
                    client_info['authenticated'] = True
                    client_info['agent_id'] = agent_id
                    await self._send(websocket, {
                        'type': 'auth',
                        'status': 'success',
                        'agent_id': agent_id
                    })
                    logger.info(f"Client authenticated: {agent_id}")
                else:
                    await self._send(websocket, {
                        'type': 'auth',
                        'status': 'failed'
                    })
                    await websocket.close(1008, "Authentication failed")
                return
            else:
                await self._send_error(websocket, 'Authentication required')
                return
        
        # Add client info to message for handlers
        data['_client_info'] = client_info
        
        # Dispatch to handlers
        for handler in self._message_handlers:
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(websocket, data)
                else:
                    handler(websocket, data)
            except Exception as e:
                logger.error(f"Handler error: {e}")
    
    def _verify_token(self, token: str, agent_id: str) -> bool:
        """Verify JWT token.
        
        In production, use a proper JWT library with:
        - Signature verification
        - Expiration checking
        - Issuer validation
        """
        import os
        import hmac
        import hashlib
        import base64
        
        if not token:
            return False
        
        # Get JWT secret from config or environment
        jwt_secret = self.auth_config.get('jwt_secret') or os.environ.get('JWT_SECRET')
        if not jwt_secret:
            logger.warning("JWT_SECRET not configured, allowing all tokens (INSECURE)")
            return True
        
        # Simple HMAC verification (replace with PyJWT in production)
        try:
            # Expected token format: base64(agent_id).base64(timestamp).signature
            parts = token.split('.')
            if len(parts) != 3:
                return False
            
            message = f"{parts[0]}.{parts[1]}"
            expected_sig = base64.urlsafe_b64encode(
                hmac.new(
                    jwt_secret.encode(),
                    message.encode(),
                    hashlib.sha256
                ).digest()
            ).decode().rstrip('=')
            
            return hmac.compare_digest(parts[2], expected_sig)
        except Exception as e:
            logger.error(f"Token verification error: {e}")
            return False
    
    async def _send(self, websocket: WebSocketServerProtocol, data: Dict):
        """Send data to a client."""
        if WEBSOCKETS_AVAILABLE:
            await websocket.send(json.dumps(data))
    
    async def _send_error(self, websocket: WebSocketServerProtocol, error: str):
        """Send error to a client."""
        await self._send(websocket, {'type': 'error', 'message': error})
    
    async def send_to_agent(self, agent_id: str, data: Dict) -> bool:
        """Send message to a specific agent by ID.
        
        Args:
            agent_id: Agent identifier
            data: Message data
            
        Returns:
            True if sent, False if agent not found
        """
        for websocket, client_info in self._clients.items():
            if client_info.get('agent_id') == agent_id:
                try:
                    await self._send(websocket, data)
                    return True
                except websockets.exceptions.ConnectionClosed:
                    pass
        return False
    
    async def broadcast(self, data: Dict, authenticated_only: bool = True):
        """Broadcast message to all connected clients.
        
        Args:
            data: Message to broadcast
            authenticated_only: Only send to authenticated clients
        """
        if not WEBSOCKETS_AVAILABLE:
            return
        
        message = json.dumps(data)
        disconnected = []
        
        for websocket, client_info in self._clients.items():
            if authenticated_only and not client_info.get('authenticated'):
                continue
            
            try:
                await websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.append(websocket)
        
        # Clean up disconnected clients
        for ws in disconnected:
            if ws in self._clients:
                del self._clients[ws]
    
    async def start(self):
        """Start WebSocket server."""
        if not WEBSOCKETS_AVAILABLE:
            logger.error("Cannot start WebSocket server: websockets not installed")
            return
        
        self._running = True
        
        # Load SSL context if TLS configured
        ssl_context = self._load_ssl_context()
        
        protocol = "wss" if ssl_context else "ws"
        logger.info(f"Starting WebSocket server on {protocol}://{self.host}:{self.port}")
        
        self._server = await websockets.serve(
            self._handle_client,
            self.host,
            self.port,
            ssl=ssl_context,
            ping_interval=self.ping_interval,
            ping_timeout=self.ping_timeout
        )
        
        logger.info(f"WebSocket server started on {protocol}://{self.host}:{self.port}")
        
        # Keep server running
        await self._server.wait_closed()
    
    async def stop(self):
        """Stop WebSocket server."""
        self._running = False
        
        # Close all client connections
        for websocket in list(self._clients.keys()):
            await websocket.close(1001, "Server shutting down")
        self._clients.clear()
        
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        
        logger.info("WebSocket server stopped")
    
    def get_clients(self) -> Dict[WebSocketServerProtocol, Dict[str, Any]]:
        """Get all connected clients with their info."""
        return self._clients.copy()
    
    def get_client_count(self) -> int:
        """Get number of connected clients."""
        return len(self._clients)
    
    def get_authenticated_count(self) -> int:
        """Get number of authenticated clients."""
        return sum(1 for info in self._clients.values() if info.get('authenticated'))


__all__ = ["WebSocketTransport"]
