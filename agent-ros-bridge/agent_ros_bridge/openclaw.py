"""OpenClaw privileged integration for Agent ROS Bridge.

This module provides enhanced features when running under OpenClaw:
- Native OpenClaw protocol support
- Cloud orchestration integration
- Enhanced telemetry and monitoring
- Automatic skill discovery

Usage:
    from agent_ros_bridge.openclaw import OpenClawIntegration
    
    integration = OpenClawIntegration(bridge)
    await integration.connect()
"""

import asyncio
import logging
import os
from typing import Any, Dict, List, Optional
from dataclasses import dataclass

try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class OpenClawConfig:
    """Configuration for OpenClaw integration."""
    endpoint: str
    api_key: str
    bridge_id: str
    heartbeat_interval: int = 30
    telemetry_enabled: bool = True


class OpenClawIntegration:
    """Privileged integration with OpenClaw platform.
    
    Provides enhanced features when running under OpenClaw:
    - Cloud-based orchestration
    - Fleet management
    - Centralized logging
    - Skill marketplace integration
    
    Example:
        integration = OpenClawIntegration(bridge, {
            "endpoint": "wss://api.openclaw.ai/v1/bridge",
            "api_key": "${OPENCLAW_API_KEY}",
            "bridge_id": "warehouse-bot-01"
        })
        await integration.connect()
    """
    
    # OpenClaw protocol version
    PROTOCOL_VERSION = "2024-1"
    
    def __init__(self, bridge, config: Optional[Dict[str, Any]] = None):
        """Initialize OpenClaw integration.
        
        Args:
            bridge: ROSBridge instance
            config: OpenClaw configuration (or loaded from env)
        """
        self.bridge = bridge
        self.config = self._load_config(config)
        self._session = None
        self._ws = None
        self._running = False
        self._heartbeat_task = None
        
        if not AIOHTTP_AVAILABLE:
            logger.warning("aiohttp not installed. OpenClaw integration disabled.")
    
    def _load_config(self, config: Optional[Dict]) -> OpenClawConfig:
        """Load configuration from dict or environment."""
        if config is None:
            config = {}
        
        # Environment variable overrides
        endpoint = config.get('endpoint') or os.environ.get(
            'OPENCLAW_ENDPOINT', 
            'wss://api.openclaw.ai/v1/bridge'
        )
        api_key = config.get('api_key') or os.environ.get('OPENCLAW_API_KEY')
        bridge_id = config.get('bridge_id') or os.environ.get(
            'OPENCLAW_BRIDGE_ID',
            'agent-ros-bridge'
        )
        
        if not api_key:
            raise ValueError(
                "OpenClaw API key required. Set OPENCLAW_API_KEY env var "
                "or pass api_key in config."
            )
        
        return OpenClawConfig(
            endpoint=endpoint,
            api_key=api_key,
            bridge_id=bridge_id,
            heartbeat_interval=config.get('heartbeat_interval', 30),
            telemetry_enabled=config.get('telemetry_enabled', True)
        )
    
    async def connect(self):
        """Connect to OpenClaw cloud platform."""
        if not AIOHTTP_AVAILABLE:
            raise RuntimeError("aiohttp required for OpenClaw integration")
        
        import aiohttp
        
        self._session = aiohttp.ClientSession(
            headers={
                'Authorization': f'Bearer {self.config.api_key}',
                'X-OpenClaw-Protocol': self.PROTOCOL_VERSION,
                'X-Bridge-ID': self.config.bridge_id
            }
        )
        
        # Connect WebSocket
        self._ws = await self._session.ws_connect(self.config.endpoint)
        self._running = True
        
        # Send registration
        await self._send_registration()
        
        # Start heartbeat
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
        
        # Start message handler
        asyncio.create_task(self._message_loop())
        
        logger.info(f"Connected to OpenClaw: {self.config.endpoint}")
    
    async def _send_registration(self):
        """Send bridge registration to OpenClaw."""
        await self._ws.send_json({
            'type': 'register',
            'bridge_id': self.config.bridge_id,
            'protocol_version': self.PROTOCOL_VERSION,
            'capabilities': self._get_capabilities(),
            'ros_version': self.bridge.ros_version,
            'actions': self.bridge.get_registered_actions(),
            'topics': self.bridge.get_available_topics()
        })
    
    def _get_capabilities(self) -> List[str]:
        """Get bridge capabilities for OpenClaw."""
        capabilities = ['ros', 'mcp', 'actions', 'topics']
        
        # Check transport capabilities
        for transport in self.bridge.transport_manager.get_transports():
            name = type(transport).__name__.lower()
            if 'websocket' in name:
                capabilities.append('websocket')
            elif 'grpc' in name:
                capabilities.append('grpc')
        
        # Check connector capabilities
        for connector in self.bridge.connector_manager.get_connectors():
            name = type(connector).__name__.lower()
            if 'ros2' in name:
                capabilities.append('ros2')
            elif 'ros1' in name:
                capabilities.append('ros1')
        
        return list(set(capabilities))
    
    async def _heartbeat_loop(self):
        """Send periodic heartbeats to OpenClaw."""
        while self._running:
            try:
                await asyncio.sleep(self.config.heartbeat_interval)
                
                if self._ws and not self._ws.closed:
                    await self._ws.send_json({
                        'type': 'heartbeat',
                        'timestamp': asyncio.get_event_loop().time(),
                        'stats': self._get_stats()
                    })
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
    
    def _get_stats(self) -> Dict[str, Any]:
        """Get bridge statistics for telemetry."""
        return {
            'connected_agents': len(self.bridge._sessions),
            'registered_actions': len(self.bridge.get_registered_actions()),
            'available_topics': len(self.bridge.get_available_topics()),
            'transports': len(self.bridge.transport_manager.get_transports()),
            'connectors': len(self.bridge.connector_manager.get_connectors())
        }
    
    async def _message_loop(self):
        """Handle incoming messages from OpenClaw."""
        async for msg in self._ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    data = msg.json()
                    await self._handle_message(data)
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
            elif msg.type == aiohttp.WSMsgType.ERROR:
                logger.error(f"WebSocket error: {self._ws.exception()}")
                break
        
        self._running = False
    
    async def _handle_message(self, data: Dict):
        """Handle message from OpenClaw."""
        msg_type = data.get('type')
        
        if msg_type == 'execute_action':
            # Execute action on behalf of OpenClaw
            action = data.get('action')
            params = data.get('params', {})
            request_id = data.get('request_id')
            
            try:
                result = await self.bridge.call_action(action, **params)
                await self._ws.send_json({
                    'type': 'action_result',
                    'request_id': request_id,
                    'result': result
                })
            except Exception as e:
                await self._ws.send_json({
                    'type': 'action_error',
                    'request_id': request_id,
                    'error': str(e)
                })
        
        elif msg_type == 'get_state':
            # Send current bridge state
            await self._ws.send_json({
                'type': 'state',
                'actions': self.bridge.get_registered_actions(),
                'topics': self.bridge.get_available_topics(),
                'sessions': len(self.bridge._sessions)
            })
        
        elif msg_type == 'update_config':
            # Update bridge configuration
            config = data.get('config', {})
            # Apply config updates
            logger.info(f"Configuration update received: {config}")
        
        else:
            logger.warning(f"Unknown message type: {msg_type}")
    
    async def disconnect(self):
        """Disconnect from OpenClaw."""
        self._running = False
        
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass
        
        if self._ws:
            await self._ws.close()
        
        if self._session:
            await self._session.close()
        
        logger.info("Disconnected from OpenClaw")
    
    async def report_telemetry(self, data: Dict[str, Any]):
        """Report telemetry data to OpenClaw.
        
        Args:
            data: Telemetry data to report
        """
        if not self.config.telemetry_enabled or not self._ws:
            return
        
        await self._ws.send_json({
            'type': 'telemetry',
            'data': data
        })
    
    async def log_event(self, level: str, message: str, **kwargs):
        """Send log event to OpenClaw cloud.
        
        Args:
            level: Log level (debug, info, warning, error)
            message: Log message
            **kwargs: Additional context
        """
        if not self._ws:
            return
        
        await self._ws.send_json({
            'type': 'log',
            'level': level,
            'message': message,
            'timestamp': asyncio.get_event_loop().time(),
            'context': kwargs
        })


class OpenClawSkill:
    """OpenClaw Skill interface for Agent ROS Bridge.
    
    This class wraps the bridge as an OpenClaw Skill for deployment
    on the ClawHub marketplace.
    
    Example:
        skill = OpenClawSkill(bridge)
        await skill.register_with_openclaw()
    """
    
    SKILL_MANIFEST = {
        'name': 'agent-ros-bridge',
        'version': '0.4.0',
        'description': 'Connect AI agents to ROS robots',
        'author': 'OpenClaw',
        'tags': ['ros', 'robotics', 'mcp', 'agents'],
        'protocols': ['websocket', 'grpc', 'mcp'],
        'pricing': {
            'model': 'free',  # or 'usage', 'subscription'
            'tiers': []
        }
    }
    
    def __init__(self, bridge):
        self.bridge = bridge
        self.integration = None
    
    async def register_with_openclaw(self, config: Optional[Dict] = None):
        """Register this bridge as an OpenClaw skill."""
        self.integration = OpenClawIntegration(self.bridge, config)
        await self.integration.connect()
        logger.info("Registered with OpenClaw as skill")
    
    def get_manifest(self) -> Dict[str, Any]:
        """Get skill manifest for ClawHub."""
        manifest = self.SKILL_MANIFEST.copy()
        manifest['capabilities'] = {
            'ros_version': self.bridge.ros_version,
            'actions': self.bridge.get_registered_actions(),
            'transports': [
                type(t).__name__ for t in self.bridge.transport_manager.get_transports()
            ]
        }
        return manifest


# Convenience function
async def connect_to_openclaw(bridge, **kwargs) -> OpenClawIntegration:
    """Quick connect to OpenClaw cloud.
    
    Args:
        bridge: ROSBridge instance
        **kwargs: Config options (endpoint, api_key, etc.)
        
    Returns:
        Connected OpenClawIntegration
    """
    integration = OpenClawIntegration(bridge, kwargs)
    await integration.connect()
    return integration


__all__ = [
    'OpenClawIntegration',
    'OpenClawConfig', 
    'OpenClawSkill',
    'connect_to_openclaw'
]
