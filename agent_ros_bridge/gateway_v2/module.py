"""Module base class for Agent ROS Bridge.

Inspired by dimensionalOS/dimos Module class, adapted for ROS compatibility.
Modules are the primary units of deployment, running in parallel with streams and RPCs.
"""

import asyncio
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any, Generic, TypeVar

from .blueprint import ModuleBlueprint, RPCDefinition, StreamDefinition

T = TypeVar("T")


class Stream(Generic[T]):
    """Type-annotated stream for module communication.

    Similar to dimos In[] and Out[] annotations.

    Example:
        class CameraModule(Module):
            image: Out[Image]  # Output stream
            config: In[Config]  # Input stream
    """

    def __init__(self, msg_type: type[T], name: str | None = None):
        self.msg_type = msg_type
        self.name = name
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=100)
        self._subscribers: list[Callable[[T], None]] = []

    async def publish(self, msg: T) -> None:
        """Publish a message to the stream."""
        # Add to queue
        try:
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            # Drop oldest message
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(msg)
            except asyncio.QueueEmpty:
                pass

        # Notify subscribers
        for callback in self._subscribers:
            try:
                if asyncio.iscoroutinefunction(callback):
                    asyncio.create_task(callback(msg))
                else:
                    callback(msg)
            except Exception as e:
                print(f"Stream callback error: {e}")

    def subscribe(self, callback: Callable[[T], None]) -> None:
        """Subscribe to the stream."""
        self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[T], None]) -> None:
        """Unsubscribe from the stream."""
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    async def get(self) -> T:
        """Get next message from the stream (blocking)."""
        return await self._queue.get()

    def get_nowait(self) -> T | None:
        """Get next message if available (non-blocking)."""
        try:
            return self._queue.get_nowait()
        except asyncio.QueueEmpty:
            return None


# Type aliases for clarity
In = Stream  # Input stream
Out = Stream  # Output stream


class Module(ABC):
    """Base class for Agent ROS Bridge modules.

    Inspired by dimos Module class.

    Modules:
    - Run in parallel (async)
    - Have typed input/output streams
    - Expose RPC methods
    - Can define AI-callable skills

    Example:
        class CameraModule(Module):
            image: Out[Image]

            @rpc
            def start_capture(self):
                # Start capturing
                pass

            @skill
            def capture_photo(self) -> Image:
                # AI-callable skill
                return self._capture()
    """

    def __init__(self, name: str | None = None, **config):
        self.name = name or self.__class__.__name__
        self.config = config
        self._running = False
        self._tasks: list[asyncio.Task] = []
        self._streams: dict[str, Stream] = {}
        self._rpcs: dict[str, RPCDefinition] = {}

        # Initialize streams from type annotations
        self._init_streams()

        # Initialize RPCs from methods
        self._init_rpcs()

    def _init_streams(self) -> None:
        """Initialize streams from type annotations."""
        for attr_name, attr_type in self.__class__.__annotations__.items():
            # Check if it's a Stream type
            origin = getattr(attr_type, "__origin__", None)
            if origin is Stream or isinstance(attr_type, Stream):
                # Create stream instance
                if isinstance(attr_type, Stream):
                    stream = attr_type
                else:
                    # Get type parameter
                    args = getattr(attr_type, "__args__", (Any,))
                    msg_type = args[0] if args else Any
                    stream = Stream(msg_type, name=attr_name)

                stream.name = attr_name
                self._streams[attr_name] = stream
                setattr(self, attr_name, stream)

    def _init_rpcs(self) -> None:
        """Initialize RPCs from decorated methods."""
        for name in dir(self):
            if name.startswith("_"):
                continue

            attr = getattr(self, name)
            if callable(attr):
                # Check for decorators
                if hasattr(attr, "_rpc_definition"):
                    self._rpcs[name] = attr._rpc_definition
                elif hasattr(attr, "_is_rpc") or hasattr(attr, "_is_skill"):
                    self._rpcs[name] = RPCDefinition(
                        name=name, func=attr, is_skill=getattr(attr, "_is_skill", False)
                    )

    @classmethod
    def blueprint(cls, **default_config) -> ModuleBlueprint:
        """Create a blueprint for this module class.

        Similar to dimos Module.blueprint() class method.
        """
        # Collect streams from annotations
        streams = []
        for attr_name, attr_type in cls.__annotations__.items():
            origin = getattr(attr_type, "__origin__", None)
            if origin is Stream or isinstance(attr_type, Stream):
                if isinstance(attr_type, Stream):
                    msg_type = attr_type.msg_type
                else:
                    args = getattr(attr_type, "__args__", (Any,))
                    msg_type = args[0] if args else Any

                # Determine direction from attribute name convention
                direction = "bidirectional"
                if attr_name.startswith("in_"):
                    direction = "in"
                elif attr_name.startswith("out_"):
                    direction = "out"

                streams.append(
                    StreamDefinition(name=attr_name, msg_type=msg_type, direction=direction)
                )

        # Collect RPCs
        rpcs = []
        for name in dir(cls):
            if name.startswith("_"):
                continue
            attr = getattr(cls, name)
            if callable(attr) and hasattr(attr, "_rpc_definition"):
                rpcs.append(attr._rpc_definition)

        return ModuleBlueprint(
            name=cls.__name__,
            factory=lambda **config: cls(**{**default_config, **config}),
            streams=streams,
            rpcs=rpcs,
            config=default_config,
        )

    async def start(self) -> None:
        """Start the module."""
        if self._running:
            return

        self._running = True

        # Call user-defined on_start
        if hasattr(self, "on_start"):
            result = self.on_start()
            if asyncio.iscoroutine(result):
                await result

        # Start background tasks
        if hasattr(self, "run"):
            task = asyncio.create_task(self._run_wrapper())
            self._tasks.append(task)

    async def stop(self) -> None:
        """Stop the module."""
        if not self._running:
            return

        self._running = False

        # Cancel tasks
        for task in self._tasks:
            task.cancel()

        # Wait for cancellation
        await asyncio.gather(*self._tasks, return_exceptions=True)

        # Call user-defined on_stop
        if hasattr(self, "on_stop"):
            result = self.on_stop()
            if asyncio.iscoroutine(result):
                await result

    async def _run_wrapper(self) -> None:
        """Wrapper for user-defined run method."""
        try:
            await self.run()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Module {self.name} run error: {e}")

    @abstractmethod
    async def run(self) -> None:
        """Main module loop (override in subclass).

        This runs continuously while the module is active.
        """
        pass

    def call_rpc(self, name: str, *args, **kwargs) -> Any:
        """Call an RPC method on this module."""
        if name not in self._rpcs:
            raise ValueError(f"RPC '{name}' not found")

        rpc_def = self._rpcs[name]
        return rpc_def.func(self, *args, **kwargs)

    def get_skills(self) -> dict[str, RPCDefinition]:
        """Get all AI-callable skills."""
        return {name: rpc for name, rpc in self._rpcs.items() if rpc.is_skill}

    def get_streams(self) -> dict[str, Stream]:
        """Get all streams."""
        return self._streams.copy()


class CompositeModule(Module):
    """Module composed of multiple sub-modules.

    Useful for creating higher-level abstractions.
    """

    def __init__(self, name: str | None = None, **config):
        super().__init__(name, **config)
        self._submodules: dict[str, Module] = {}

    def add_module(self, name: str, module: Module) -> "CompositeModule":
        """Add a submodule."""
        self._submodules[name] = module
        return self

    async def start(self) -> None:
        """Start composite and all submodules."""
        await super().start()

        for module in self._submodules.values():
            await module.start()

    async def stop(self) -> None:
        """Stop composite and all submodules."""
        for module in self._submodules.values():
            await module.stop()

        await super().stop()

    async def run(self) -> None:
        """Composite modules typically don't have their own run loop."""
        # Just keep alive while submodules run
        while self._running:
            await asyncio.sleep(1)
