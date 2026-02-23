"""gRPC transport for Agent ROS Bridge.

Supports local and remote connections with optional TLS encryption.
"""

import asyncio
import logging
import ssl
from typing import Any, Callable, Dict, List, Optional

try:
    import grpc
    from concurrent import futures
    GRPC_AVAILABLE = True
except ImportError:
    GRPC_AVAILABLE = False

logger = logging.getLogger(__name__)


class GRPCServer:
    """gRPC transport for high-performance agent communication.
    
    Supports both local and remote connections. For remote/distributed
    deployments, use TLS encryption and mutual authentication.
    
    Examples:
        # Local development (no TLS)
        GRPCServer({
            "host": "localhost",
            "port": 50051,
            "auth": {"enabled": False}
        })
        
        # Remote edge server (with TLS)
        GRPCServer({
            "host": "0.0.0.0",
            "port": 50051,
            "tls": {
                "cert": "/etc/ssl/certs/server.crt",
                "key": "/etc/ssl/private/server.key",
                "ca": "/etc/ssl/certs/ca.crt"  # Optional: for mutual TLS
            },
            "auth": {"enabled": True}
        })
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize gRPC server.
        
        Args:
            config: Configuration dict with:
                - host: Bind address (default: "0.0.0.0")
                - port: Port number (default: 50051)
                - tls: Optional TLS config with cert/key/ca paths
                - auth: Auth config with enabled flag
                - max_workers: Thread pool size (default: 10)
                - compression: gRPC compression (default: None)
        """
        self.config = config
        self.host = config.get('host', '0.0.0.0')
        self.port = config.get('port', 50051)
        self.tls_config = config.get('tls')
        self.auth_config = config.get('auth', {})
        self.auth_enabled = self.auth_config.get('enabled', False)
        self.max_workers = config.get('max_workers', 10)
        self.compression = config.get('compression')
        
        self._server = None
        self._running = False
        self._connected_agents: Dict[str, Dict[str, Any]] = {}
        
        if not GRPC_AVAILABLE:
            logger.warning("grpcio not installed. gRPC transport disabled.")
    
    def _create_server_credentials(self) -> Optional[grpc.ServerCredentials]:
        """Create gRPC server credentials from TLS config.
        
        Returns:
            ServerCredentials if TLS is configured, None for insecure
        """
        if not self.tls_config:
            return None
        
        cert_path = self.tls_config.get('cert')
        key_path = self.tls_config.get('key')
        ca_path = self.tls_config.get('ca')  # For mutual TLS
        
        if not cert_path or not key_path:
            logger.warning("TLS config missing cert or key path")
            return None
        
        try:
            with open(cert_path, 'rb') as f:
                cert_chain = f.read()
            with open(key_path, 'rb') as f:
                private_key = f.read()
            
            if ca_path:
                # Mutual TLS (client certificate verification)
                with open(ca_path, 'rb') as f:
                    ca_cert = f.read()
                
                credentials = grpc.ssl_server_credentials(
                    ((private_key, cert_chain),),
                    root_certificates=ca_cert,
                    require_client_auth=True
                )
                logger.info("Loaded mutual TLS credentials (client auth required)")
            else:
                # Server-only TLS
                credentials = grpc.ssl_server_credentials(((private_key, cert_chain),))
                logger.info(f"Loaded TLS credentials from {cert_path}")
            
            return credentials
            
        except Exception as e:
            logger.error(f"Failed to load TLS credentials: {e}")
            raise
    
    def register_service(self, servicer, add_servicer_func):
        """Register a gRPC servicer.
        
        Args:
            servicer: The servicer implementation
            add_servicer_func: The generated add_servicer_to_server function
        """
        if self._server:
            add_servicer_func(servicer, self._server)
            logger.info(f"Registered gRPC service: {type(servicer).__name__}")
        else:
            logger.warning("Cannot register service: server not created yet")
    
    def add_agent(self, agent_id: str, metadata: Dict[str, Any] = None):
        """Register a connected agent.
        
        Args:
            agent_id: Unique agent identifier
            metadata: Optional agent metadata
        """
        self._connected_agents[agent_id] = {
            "agent_id": agent_id,
            "connected_at": asyncio.get_event_loop().time(),
            "metadata": metadata or {}
        }
        logger.info(f"gRPC agent connected: {agent_id}")
    
    def remove_agent(self, agent_id: str):
        """Unregister a disconnected agent."""
        if agent_id in self._connected_agents:
            del self._connected_agents[agent_id]
            logger.info(f"gRPC agent disconnected: {agent_id}")
    
    def get_agent(self, agent_id: str) -> Optional[Dict[str, Any]]:
        """Get agent info by ID."""
        return self._connected_agents.get(agent_id)
    
    def list_agents(self) -> List[Dict[str, Any]]:
        """List all connected agents."""
        return list(self._connected_agents.values())
    
    async def start(self):
        """Start gRPC server."""
        if not GRPC_AVAILABLE:
            logger.error("Cannot start gRPC server: grpcio not installed")
            return
        
        self._running = True
        
        # Create server with thread pool
        self._server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers),
            compression=self.compression
        )
        
        # Add port with or without TLS
        address = f"{self.host}:{self.port}"
        credentials = self._create_server_credentials()
        
        if credentials:
            self._server.add_secure_port(address, credentials)
            logger.info(f"gRPC secure server starting on {address} (TLS enabled)")
        else:
            self._server.add_insecure_port(address)
            logger.info(f"gRPC insecure server starting on {address}")
            
            if self.host not in ('localhost', '127.0.0.1'):
                logger.warning(
                    "gRPC server binding to non-local address without TLS! "
                    "Use TLS for remote connections."
                )
        
        self._server.start()
        logger.info(f"gRPC server started on {address}")
        
        # Keep server running
        try:
            while self._running:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
    
    async def stop(self, grace_period: float = 5.0):
        """Stop gRPC server.
        
        Args:
            grace_period: Seconds to wait for graceful shutdown
        """
        self._running = False
        
        if self._server:
            self._server.stop(grace_period)
            logger.info(f"gRPC server stopped (grace period: {grace_period}s)")
        
        self._connected_agents.clear()
    
    def is_running(self) -> bool:
        """Check if server is running."""
        return self._running and self._server is not None


class GRPCClient:
    """gRPC client for connecting to remote bridges.
    
    Useful for OpenClaw or other agents connecting to a remote bridge.
    
    Example:
        # Connect to remote bridge with TLS
        client = GRPCClient({
            "host": "bridge.example.com",
            "port": 50051,
            "tls": {
                "enabled": True,
                "ca": "/path/to/ca.crt",  # Optional: verify server
                "cert": "/path/to/client.crt",  # Optional: for mutual TLS
                "key": "/path/to/client.key"
            }
        })
        await client.connect()
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize gRPC client.
        
        Args:
            config: Configuration with host, port, and optional TLS
        """
        self.config = config
        self.host = config.get('host', 'localhost')
        self.port = config.get('port', 50051)
        self.tls_config = config.get('tls', {})
        self.tls_enabled = self.tls_config.get('enabled', False)
        
        self._channel = None
        self._stub = None
        self._connected = False
    
    def _create_channel_credentials(self) -> Optional[grpc.ChannelCredentials]:
        """Create channel credentials for TLS connection."""
        if not self.tls_enabled:
            return None
        
        ca_cert = None
        client_cert = None
        client_key = None
        
        # Load CA certificate for server verification
        ca_path = self.tls_config.get('ca')
        if ca_path:
            with open(ca_path, 'rb') as f:
                ca_cert = f.read()
        
        # Load client certificate for mutual TLS
        cert_path = self.tls_config.get('cert')
        key_path = self.tls_config.get('key')
        if cert_path and key_path:
            with open(cert_path, 'rb') as f:
                client_cert = f.read()
            with open(key_path, 'rb') as f:
                client_key = f.read()
        
        if client_cert and client_key:
            # Mutual TLS
            credentials = grpc.ssl_channel_credentials(
                root_certificates=ca_cert,
                private_key=client_key,
                certificate_chain=client_cert
            )
        else:
            # Server-only TLS
            credentials = grpc.ssl_channel_credentials(root_certificates=ca_cert)
        
        return credentials
    
    async def connect(self):
        """Connect to gRPC server."""
        if not GRPC_AVAILABLE:
            raise RuntimeError("grpcio not installed")
        
        address = f"{self.host}:{self.port}"
        credentials = self._create_channel_credentials()
        
        if credentials:
            self._channel = grpc.secure_channel(address, credentials)
            logger.info(f"Connected to gRPC server at {address} (TLS)")
        else:
            self._channel = grpc.insecure_channel(address)
            logger.info(f"Connected to gRPC server at {address} (insecure)")
        
        self._connected = True
    
    async def disconnect(self):
        """Disconnect from server."""
        if self._channel:
            self._channel.close()
            self._channel = None
        self._connected = False
        logger.info("Disconnected from gRPC server")
    
    def get_channel(self) -> Optional[grpc.Channel]:
        """Get the gRPC channel."""
        return self._channel
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._connected and self._channel is not None


__all__ = ["GRPCServer", "GRPCClient"]
