"""Distributed tracing for Agent ROS Bridge.

OpenTelemetry integration for request tracing across services.

Example:
    from agent_ros_bridge.tracing import TracingManager
    
    tracing = TracingManager(bridge)
    await tracing.init(jaeger_endpoint="http://jaeger:14268/api/traces")
    
    # Automatic tracing on all actions
"""

import logging
from typing import Any, Dict, Optional
from contextlib import asynccontextmanager
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class TraceConfig:
    """Configuration for distributed tracing."""
    enabled: bool = True
    service_name: str = "agent-ros-bridge"
    jaeger_endpoint: Optional[str] = None
    otlp_endpoint: Optional[str] = None
    sample_rate: float = 1.0


class TracingManager:
    """Manage distributed tracing with OpenTelemetry.
    
    Traces:
    - Agent requests
    - Action execution
    - ROS operations
    - Transport messages
    """
    
    def __init__(self, bridge, config: Optional[TraceConfig] = None):
        """Initialize tracing manager.
        
        Args:
            bridge: ROSBridge instance
            config: Tracing configuration
        """
        self.bridge = bridge
        self.config = config or TraceConfig()
        self._tracer = None
        self._initialized = False
    
    async def init(self):
        """Initialize OpenTelemetry tracing."""
        if not self.config.enabled:
            logger.info("Tracing disabled")
            return
        
        try:
            from opentelemetry import trace
            from opentelemetry.sdk.trace import TracerProvider
            from opentelemetry.sdk.trace.export import BatchSpanProcessor
            from opentelemetry.sdk.resources import Resource, SERVICE_NAME
            
            # Create resource
            resource = Resource.create({SERVICE_NAME: self.config.service_name})
            
            # Create provider
            provider = TracerProvider(resource=resource)
            trace.set_tracer_provider(provider)
            
            # Configure exporter
            if self.config.jaeger_endpoint:
                from opentelemetry.exporter.jaeger.thrift import JaegerExporter
                exporter = JaegerExporter(
                    collector_endpoint=self.config.jaeger_endpoint
                )
                provider.add_span_processor(BatchSpanProcessor(exporter))
                logger.info(f"Jaeger exporter configured: {self.config.jaeger_endpoint}")
            
            elif self.config.otlp_endpoint:
                from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
                exporter = OTLPSpanExporter(endpoint=self.config.otlp_endpoint)
                provider.add_span_processor(BatchSpanProcessor(exporter))
                logger.info(f"OTLP exporter configured: {self.config.otlp_endpoint}")
            
            else:
                # Console exporter for debugging
                from opentelemetry.sdk.trace.export import ConsoleSpanExporter
                provider.add_span_processor(BatchSpanProcessor(ConsoleSpanExporter()))
                logger.info("Console exporter configured")
            
            self._tracer = trace.get_tracer(__name__)
            self._initialized = True
            
        except ImportError:
            logger.warning("OpenTelemetry not installed. Tracing disabled.")
            logger.warning("Install: pip install opentelemetry-api opentelemetry-sdk")
    
    @asynccontextmanager
    async def span(self, name: str, attributes: Optional[Dict] = None):
        """Create a tracing span.
        
        Args:
            name: Span name
            attributes: Span attributes
            
        Usage:
            async with tracing.span("navigate", {"x": 5, "y": 3}):
                await bridge.call_action("navigate", x=5, y=3)
        """
        if not self._initialized or not self._tracer:
            yield None
            return
        
        with self._tracer.start_as_current_span(name) as span:
            if attributes:
                for key, value in attributes.items():
                    span.set_attribute(key, value)
            yield span
    
    def trace_action(self, action_name: str):
        """Decorator to trace action execution.
        
        Usage:
            @tracing.trace_action("navigate")
            async def navigate(x, y):
                pass
        """
        def decorator(func):
            async def wrapper(*args, **kwargs):
                if not self._initialized:
                    return await func(*args, **kwargs)
                
                async with self.span(f"action:{action_name}", {
                    "action": action_name,
                    "args": str(args),
                    "kwargs": str(kwargs)
                }):
                    return await func(*args, **kwargs)
            
            return wrapper
        return decorator
    
    def add_event(self, span, name: str, attributes: Optional[Dict] = None):
        """Add event to current span.
        
        Args:
            span: Current span
            name: Event name
            attributes: Event attributes
        """
        if span and self._initialized:
            span.add_event(name, attributes or {})
    
    def set_error(self, span, error: Exception):
        """Record error on span.
        
        Args:
            span: Current span
            error: Exception that occurred
        """
        if span and self._initialized:
            from opentelemetry.trace import Status, StatusCode
            span.set_status(Status(StatusCode.ERROR, str(error)))
            span.record_exception(error)


class SpanContext:
    """Context manager for manual span handling."""
    
    def __init__(self, tracing: TracingManager, name: str, **kwargs):
        self.tracing = tracing
        self.name = name
        self.kwargs = kwargs
        self.span = None
    
    async def __aenter__(self):
        self.cm = self.tracing.span(self.name, self.kwargs)
        self.span = await self.cm.__aenter__()
        return self.span
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        return await self.cm.__aexit__(exc_type, exc_val, exc_tb)


# Convenience functions
def create_tracing(bridge, **kwargs) -> TracingManager:
    """Create tracing manager.
    
    Args:
        bridge: ROSBridge instance
        **kwargs: Config options
        
    Returns:
        TracingManager instance
    """
    config = TraceConfig(**kwargs)
    return TracingManager(bridge, config)


__all__ = [
    "TracingManager",
    "TraceConfig",
    "SpanContext",
    "create_tracing"
]
