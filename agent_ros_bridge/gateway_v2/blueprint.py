"""Blueprint pattern for Agent ROS Bridge.

Inspired by dimensionalOS/dimos blueprints, adapted for ROS compatibility.
Blueprints provide a declarative way to compose modules and define their connections.
"""

import asyncio
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any


@dataclass
class StreamDefinition:
    """Definition of a data stream between modules.

    Similar to dimos In[] and Out[] type annotations.
    """

    name: str
    msg_type: type
    direction: str = "bidirectional"  # "in", "out", "bidirectional"
    transport: str = "lcm"  # "lcm", "ros", "websocket", "shm"
    qos: int = 10  # Quality of Service (queue depth)

    def __post_init__(self):
        if self.direction not in ("in", "out", "bidirectional"):
            raise ValueError(f"Invalid direction: {self.direction}")


@dataclass
class RPCDefinition:
    """Definition of an RPC method exposed by a module."""

    name: str
    func: Callable
    input_type: type | None = None
    output_type: type | None = None
    is_skill: bool = False  # Can be called by AI agents


@dataclass
class ModuleBlueprint:
    """Blueprint for a module (inspired by dimos Module.blueprint()).

    A module blueprint defines:
    - The module class/factory
    - Input/output streams
    - RPC methods
    - Configuration
    """

    name: str
    factory: Callable
    streams: list[StreamDefinition] = field(default_factory=list)
    rpcs: list[RPCDefinition] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)
    dependencies: list[str] = field(default_factory=list)

    def with_stream(self, stream: StreamDefinition) -> "ModuleBlueprint":
        """Add a stream to the blueprint."""
        self.streams.append(stream)
        return self

    def with_rpc(self, rpc: RPCDefinition) -> "ModuleBlueprint":
        """Add an RPC to the blueprint."""
        self.rpcs.append(rpc)
        return self

    def with_config(self, **kwargs) -> "ModuleBlueprint":
        """Add configuration to the blueprint."""
        self.config.update(kwargs)
        return self

    def build(self, **override_config) -> Any:
        """Instantiate the module."""
        config = {**self.config, **override_config}
        return self.factory(**config)


@dataclass
class Connection:
    """Connection between two streams."""

    source_module: str
    source_stream: str
    target_module: str
    target_stream: str
    transform: Callable | None = None


class Blueprint:
    """Blueprint for composing multiple modules.

    Inspired by dimos autoconnect() and Blueprint pattern.

    Example:
        blueprint = Blueprint()
            .add_module("camera", camera_blueprint)
            .add_module("detector", detector_blueprint)
            .connect("camera", "image", "detector", "input")
            .transports({("image", Image): LCMTransport()})
    """

    def __init__(self):
        self.modules: dict[str, ModuleBlueprint] = {}
        self.connections: list[Connection] = []
        self.transports: dict[tuple[str, type], Any] = {}
        self._instances: dict[str, Any] = {}
        self._running = False

    def add_module(self, name: str, blueprint: ModuleBlueprint) -> "Blueprint":
        """Add a module to the blueprint."""
        self.modules[name] = blueprint
        return self

    def connect(
        self,
        source_module: str,
        source_stream: str,
        target_module: str,
        target_stream: str,
        transform: Callable | None = None,
    ) -> "Blueprint":
        """Connect two module streams."""
        conn = Connection(
            source_module=source_module,
            source_stream=source_stream,
            target_module=target_module,
            target_stream=target_stream,
            transform=transform,
        )
        self.connections.append(conn)
        return self

    def transports(self, transport_map: dict[tuple[str, type], Any]) -> "Blueprint":
        """Override transports for specific streams."""
        self.transports.update(transport_map)
        return self

    def autoconnect(self) -> "Blueprint":
        """Automatically connect modules by matching stream names and types.

        Inspired by dimos autoconnect() function.
        """
        # Build stream registry
        stream_registry: dict[tuple[str, type], list[tuple[str, str, str]]] = defaultdict(list)

        for module_name, blueprint in self.modules.items():
            for stream in blueprint.streams:
                key = (stream.name, stream.msg_type)
                stream_registry[key].append((module_name, stream.name, stream.direction))

        # Auto-connect matching streams
        for (_stream_name, _msg_type), endpoints in stream_registry.items():
            outputs = [(m, s) for m, s, d in endpoints if d in ("out", "bidirectional")]
            inputs = [(m, s) for m, s, d in endpoints if d in ("in", "bidirectional")]

            for out_module, out_stream in outputs:
                for in_module, in_stream in inputs:
                    if out_module != in_module:  # Don't self-connect
                        self.connect(out_module, out_stream, in_module, in_stream)

        return self

    def build(self) -> dict[str, Any]:
        """Build all modules and return instances."""
        for name, blueprint in self.modules.items():
            self._instances[name] = blueprint.build()

        return self._instances

    async def start(self) -> "Blueprint":
        """Start all modules and establish connections."""
        if self._running:
            return self

        # Build modules
        self.build()

        # Start modules
        for _name, instance in self._instances.items():
            if hasattr(instance, "start"):
                if asyncio.iscoroutinefunction(instance.start):
                    await instance.start()
                else:
                    instance.start()

        # Establish connections
        await self._establish_connections()

        self._running = True
        return self

    async def stop(self) -> "Blueprint":
        """Stop all modules."""
        if not self._running:
            return self

        for _name, instance in self._instances.items():
            if hasattr(instance, "stop"):
                if asyncio.iscoroutinefunction(instance.stop):
                    await instance.stop()
                else:
                    instance.stop()

        self._running = False
        return self

    async def _establish_connections(self) -> None:
        """Establish stream connections between modules."""
        for conn in self.connections:
            source = self._instances.get(conn.source_module)
            target = self._instances.get(conn.target_module)

            if not source or not target:
                continue

            # Set up pub/sub based on stream direction
            # This is a simplified version - real implementation would use transports
            pass

    def loop(self) -> None:
        """Run the blueprint (blocking)."""
        asyncio.run(self._run_loop())

    async def _run_loop(self) -> None:
        """Main loop for the blueprint."""
        await self.start()

        try:
            while self._running:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            await self.stop()


def autoconnect(*blueprints: ModuleBlueprint) -> Blueprint:
    """Automatically connect module blueprints.

    Inspired by dimos autoconnect() function.

    Args:
        *blueprints: Module blueprints to connect

    Returns:
        Blueprint with auto-connected modules
    """
    main_blueprint = Blueprint()

    # Add all modules
    for i, bp in enumerate(blueprints):
        name = bp.name or f"module_{i}"
        main_blueprint.add_module(name, bp)

    # Auto-connect
    main_blueprint.autoconnect()

    return main_blueprint


# Decorator for defining skills (AI-callable RPCs)
def skill(func: Callable) -> Callable:
    """Decorator to mark a function as an AI-callable skill.

    Similar to dimos @rpc decorator.
    """
    func._is_skill = True
    func._rpc_definition = RPCDefinition(name=func.__name__, func=func, is_skill=True)
    return func


# Decorator for defining RPC methods
def rpc(func: Callable) -> Callable:
    """Decorator to mark a function as an RPC method.

    Similar to dimos @rpc decorator.
    """
    func._is_rpc = True
    func._rpc_definition = RPCDefinition(name=func.__name__, func=func, is_skill=False)
    return func
