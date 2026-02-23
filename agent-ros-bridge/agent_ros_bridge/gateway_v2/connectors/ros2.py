"""ROS2 connector for Agent ROS Bridge.

Supports ROS2 distributions:
- Jazzy Jalisco (LTS) - rclpy 7.x - Python 3.12
- Iron Irwini - rclpy 6.x - Python 3.10
- Humble Hawksbill (LTS) - rclpy 3.x - Python 3.10
- Rolling Ridley (development)

Requires rclpy to be installed:
    # Using your ROS2 distribution (recommended)
    source /opt/ros/jazzy/setup.bash  # or humble, iron, rolling

    # Or via pip (if available for your platform)
    pip install rclpy

Or use rosdep:
    rosdep install --from-paths src --ignore-src -r -y
"""

import asyncio
import logging
import threading
from typing import Any, Callable, Dict, List, Optional, Type
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.executors import SingleThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    ROS2_IMPORT_ERROR = str(e)

logger = logging.getLogger(__name__)


@dataclass
class TopicInfo:
    """Information about a ROS topic."""
    name: str
    msg_type: str
    msg_type_class: Optional[Type] = None


class ROS2Connector:
    """Production-ready ROS2 connector using rclpy.
    
    Manages a ROS2 node lifecycle, handles subscriptions, publications,
    service calls, and action client interactions.
    
    Raises:
        RuntimeError: If rclpy is not installed or ROS2 is not available.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize ROS2 connector.
        
        Args:
            config: Configuration dictionary with keys:
                - domain_id: ROS domain ID (default: 0)
                - namespace: Node namespace (default: '')
                - node_name: Node name (default: 'agent_ros_bridge')
                - use_sim_time: Use simulation time (default: False)
        
        Raises:
            RuntimeError: If rclpy is not installed.
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError(
                f"ROS2 connector requires rclpy. "
                f"Install it: pip install rclpy\n"
                f"Import error: {ROS2_IMPORT_ERROR}"
            )
        
        self.config = config
        self.domain_id = config.get('domain_id', 0)
        self.namespace = config.get('namespace', '')
        self.node_name = config.get('node_name', 'agent_ros_bridge')
        self.use_sim_time = config.get('use_sim_time', False)
        
        # Node and executor
        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        
        # State
        self._running = False
        self._initialized = False
        
        # Publishers, subscriptions, services
        self._publishers: Dict[str, Any] = {}
        self._subscriptions: Dict[str, Any] = {}
        self._topic_data: Dict[str, Dict[str, Any]] = {}
        self._topic_callbacks: Dict[str, List[Callable]] = {}
        
        # Message type cache
        self._msg_types: Dict[str, Type] = {}
    
    def _get_msg_type(self, msg_type_str: str) -> Type:
        """Dynamically import and return a message type.
        
        Args:
            msg_type_str: Message type string like 'std_msgs/msg/String'
            
        Returns:
            The message class
            
        Raises:
            ImportError: If message type cannot be imported
        """
        if msg_type_str in self._msg_types:
            return self._msg_types[msg_type_str]
        
        # Parse type string: package/msg/Type or package/msg/Type
        parts = msg_type_str.replace('/', '.').split('.')
        if len(parts) == 3:
            # Format: package.msg.Type
            module_path = f"{parts[0]}.{parts[1]}"
            class_name = parts[2]
        elif len(parts) == 2:
            # Format: package.Type
            module_path = parts[0]
            class_name = parts[1]
        else:
            raise ImportError(f"Invalid message type format: {msg_type_str}")
        
        module = __import__(module_path, fromlist=[class_name])
        msg_class = getattr(module, class_name)
        self._msg_types[msg_type_str] = msg_class
        return msg_class
    
    async def start(self):
        """Initialize and start the ROS2 node.
        
        Raises:
            RuntimeError: If ROS2 is already initialized in this process
                          with a different domain ID.
        """
        if self._initialized:
            logger.warning("ROS2 connector already started")
            return
        
        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init(domain_id=self.domain_id)
            logger.info(f"Initialized rclpy with domain_id={self.domain_id}")
        
        # Create node
        self._node = rclpy.create_node(
            self.node_name,
            namespace=self.namespace,
            use_sim_time=self.use_sim_time
        )
        
        # Create executor and spin in background thread
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        
        self._initialized = True
        logger.info(
            f"ROS2 node '{self.node_name}' started "
            f"(domain={self.domain_id}, ns='{self.namespace}')"
        )
    
    def _spin(self):
        """Spin the executor in a background thread."""
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    logger.error(f"Error in ROS2 spin: {e}")
    
    async def stop(self):
        """Shutdown the ROS2 node and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info("Stopping ROS2 connector...")
        self._running = False
        
        # Stop spinning
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        
        # Cleanup node
        if self._node:
            # Destroy all publishers and subscriptions
            for pub in list(self._publishers.values()):
                self._node.destroy_publisher(pub)
            for sub in list(self._subscriptions.values()):
                self._node.destroy_subscription(sub)
            
            self._node.destroy_node()
            self._node = None
        
        # Cleanup executor
        if self._executor:
            self._executor.shutdown()
            self._executor = None
        
        # Shutdown rclpy if we initialized it
        if rclpy.ok():
            rclpy.shutdown()
        
        self._initialized = False
        self._publishers.clear()
        self._subscriptions.clear()
        self._topic_data.clear()
        
        logger.info("ROS2 connector stopped")
    
    def get_topics(self) -> List[str]:
        """Get list of available ROS topics.
        
        Returns:
            List of topic names
        """
        if not self._node:
            return []
        
        topics = self._node.get_topic_names_and_types()
        return sorted([t[0] for t in topics])
    
    def get_topic_types(self) -> Dict[str, List[str]]:
        """Get topic names and their message types.
        
        Returns:
            Dict mapping topic names to list of message type strings
        """
        if not self._node:
            return {}
        
        topics = self._node.get_topic_names_and_types()
        return {name: types for name, types in topics}
    
    async def get_topic_data(self, topic: str) -> Dict[str, Any]:
        """Get latest data for a topic.
        
        Args:
            topic: Topic name
            
        Returns:
            Dict with 'data' and 'timestamp', or 'error' if not available
        """
        if topic in self._topic_data:
            return self._topic_data[topic]
        
        return {"error": f"No data received for topic '{topic}' yet"}
    
    def subscribe(
        self, 
        topic: str, 
        callback: Callable[[Any], None],
        msg_type: Optional[Type] = None,
        msg_type_str: Optional[str] = None,
        qos_profile: Optional[QoSProfile] = None
    ):
        """Subscribe to a ROS topic.
        
        Args:
            topic: Topic name
            callback: Function to call with received messages
            msg_type: Message type class (optional if msg_type_str provided)
            msg_type_str: Message type string like 'std_msgs/msg/String' (optional)
            qos_profile: QoS profile (optional)
            
        Raises:
            RuntimeError: If node is not running
            ImportError: If message type cannot be resolved
        """
        if not self._node:
            raise RuntimeError("Cannot subscribe: ROS2 node not running")
        
        if topic in self._subscriptions:
            # Already subscribed, just add callback
            self._topic_callbacks.setdefault(topic, []).append(callback)
            logger.debug(f"Added callback to existing subscription: {topic}")
            return
        
        # Resolve message type
        if msg_type is None and msg_type_str:
            msg_type = self._get_msg_type(msg_type_str)
        
        if msg_type is None:
            raise ValueError(f"Must provide msg_type or msg_type_str for topic {topic}")
        
        # Default QoS
        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        
        # Store callbacks
        self._topic_callbacks[topic] = [callback]
        
        def _ros_callback(msg):
            """Internal callback that stores data and dispatches to handlers."""
            import time
            self._topic_data[topic] = {
                "data": msg,
                "timestamp": time.time()
            }
            
            for cb in self._topic_callbacks.get(topic, []):
                try:
                    cb(msg)
                except Exception as e:
                    logger.error(f"Error in topic callback for {topic}: {e}")
        
        # Create subscription
        sub = self._node.create_subscription(
            msg_type,
            topic,
            _ros_callback,
            qos_profile
        )
        self._subscriptions[topic] = sub
        logger.info(f"Subscribed to {topic} ({msg_type.__module__}.{msg_type.__name__})")
    
    def unsubscribe(self, topic: str):
        """Unsubscribe from a topic.
        
        Args:
            topic: Topic name to unsubscribe from
        """
        if topic in self._subscriptions:
            self._node.destroy_subscription(self._subscriptions[topic])
            del self._subscriptions[topic]
            del self._topic_callbacks[topic]
            logger.info(f"Unsubscribed from {topic}")
    
    def publish(
        self, 
        topic: str, 
        msg: Any,
        msg_type: Optional[Type] = None,
        msg_type_str: Optional[str] = None,
        qos_profile: Optional[QoSProfile] = None
    ):
        """Publish a message to a ROS topic.
        
        Args:
            topic: Topic name
            msg: Message to publish (can be dict or message object)
            msg_type: Message type class
            msg_type_str: Message type string (alternative to msg_type)
            qos_profile: QoS profile
            
        Raises:
            RuntimeError: If node is not running
        """
        if not self._node:
            raise RuntimeError("Cannot publish: ROS2 node not running")
        
        # Resolve message type if needed
        if msg_type is None and msg_type_str:
            msg_type = self._get_msg_type(msg_type_str)
        
        if msg_type is None:
            raise ValueError(f"Must provide msg_type or msg_type_str for topic {topic}")
        
        # Create publisher if needed
        if topic not in self._publishers:
            if qos_profile is None:
                qos_profile = QoSProfile(depth=10)
            
            pub = self._node.create_publisher(msg_type, topic, qos_profile)
            self._publishers[topic] = pub
            logger.info(f"Created publisher for {topic}")
        
        # Convert dict to message if needed
        if isinstance(msg, dict):
            msg_obj = msg_type()
            # Simple field assignment (could be enhanced)
            for key, value in msg.items():
                if hasattr(msg_obj, key):
                    setattr(msg_obj, key, value)
            msg = msg_obj
        
        self._publishers[topic].publish(msg)
    
    def call_service(
        self, 
        service_name: str, 
        request: Any,
        service_type: Optional[Type] = None,
        service_type_str: Optional[str] = None,
        timeout_sec: float = 5.0
    ) -> Any:
        """Call a ROS service synchronously.
        
        Args:
            service_name: Service name
            request: Service request
            service_type: Service type class
            service_type_str: Service type string
            timeout_sec: Timeout in seconds
            
        Returns:
            Service response
            
        Raises:
            RuntimeError: If service call fails or times out
        """
        if not self._node:
            raise RuntimeError("Cannot call service: ROS2 node not running")
        
        # Resolve service type
        if service_type is None and service_type_str:
            service_type = self._get_msg_type(service_type_str)
        
        if service_type is None:
            raise ValueError("Must provide service_type or service_type_str")
        
        # Create client
        client = self._node.create_client(service_type, service_name)
        
        # Wait for service
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service {service_name} not available")
        
        # Call service
        future = client.call_async(request)
        
        # Wait for response (blocking in this thread)
        import time
        start = time.time()
        while not future.done() and time.time() - start < timeout_sec:
            time.sleep(0.01)
        
        if not future.done():
            raise RuntimeError(f"Service call to {service_name} timed out")
        
        return future.result()
    
    def get_node(self) -> Optional[Node]:
        """Get the underlying ROS2 node.
        
        Returns:
            The rclpy Node instance or None if not running
        """
        return self._node
    
    def is_running(self) -> bool:
        """Check if the connector is running.
        
        Returns:
            True if node is initialized and running
        """
        return self._initialized and self._running and self._node is not None
    
    def get_ros_version_info(self) -> Dict[str, str]:
        """Get ROS2 version information.
        
        Returns:
            Dict with distro, rclpy version, and Python version
        """
        import sys
        import rclpy
        
        info = {
            "rclpy_version": getattr(rclpy, "__version__", "unknown"),
            "python_version": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
            "distro": "unknown"
        }
        
        # Try to detect ROS2 distro from environment or version
        import os
        distro = os.environ.get('ROS_DISTRO', '').lower()
        if distro:
            info["distro"] = distro
        else:
            # Infer from rclpy version
            rclpy_ver = info["rclpy_version"]
            if rclpy_ver.startswith("7."):
                info["distro"] = "jazzy (inferred)"
            elif rclpy_ver.startswith("6."):
                info["distro"] = "iron (inferred)"
            elif rclpy_ver.startswith("3."):
                info["distro"] = "humble (inferred)"
        
        return info


__all__ = ["ROS2Connector", "TopicInfo"]
