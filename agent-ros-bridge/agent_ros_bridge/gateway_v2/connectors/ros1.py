"""ROS1 connector for Agent ROS Bridge.

Requires rospy to be installed (comes with ROS1).

Example:
    source /opt/ros/noetic/setup.bash
    python3 your_script.py
"""

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Type
from dataclasses import dataclass

try:
    import rospy
    import rostopic
    import rosmsg
    ROS1_AVAILABLE = True
except ImportError as e:
    ROS1_AVAILABLE = False
    ROS1_IMPORT_ERROR = str(e)

logger = logging.getLogger(__name__)


@dataclass
class TopicInfo:
    """Information about a ROS topic."""
    name: str
    msg_type: str
    msg_type_class: Optional[Type] = None


class ROS1Connector:
    """Production-ready ROS1 connector using rospy.
    
    Manages a ROS1 node lifecycle, handles subscriptions, publications,
    service calls, and action client interactions.
    
    Raises:
        RuntimeError: If rospy is not installed or ROS1 is not available.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize ROS1 connector.
        
        Args:
            config: Configuration dictionary with keys:
                - node_name: Node name (default: 'agent_ros_bridge')
                - anonymous: Create anonymous node (default: True)
                - master_uri: ROS master URI (optional, uses env var)
                - hostname: ROS hostname (optional)
        
        Raises:
            RuntimeError: If rospy is not installed.
        """
        if not ROS1_AVAILABLE:
            raise RuntimeError(
                f"ROS1 connector requires rospy. "
                f"Source your ROS1 setup.bash first.\n"
                f"Import error: {ROS1_IMPORT_ERROR}"
            )
        
        self.config = config
        self.node_name = config.get('node_name', 'agent_ros_bridge')
        self.anonymous = config.get('anonymous', True)
        
        # Set environment variables if provided
        if 'master_uri' in config:
            import os
            os.environ['ROS_MASTER_URI'] = config['master_uri']
        if 'hostname' in config:
            import os
            os.environ['ROS_HOSTNAME'] = config['hostname']
        
        # State
        self._node: Optional[Any] = None  # rospy doesn't expose node as object
        self._initialized = False
        self._running = False
        
        # Publishers, subscriptions
        self._publishers: Dict[str, Any] = {}
        self._subscribers: Dict[str, Any] = {}
        self._topic_data: Dict[str, Dict[str, Any]] = {}
        self._topic_callbacks: Dict[str, List[Callable]] = {}
        
        # Message type cache
        self._msg_types: Dict[str, Type] = {}
    
    def _get_msg_type(self, msg_type_str: str) -> Type:
        """Dynamically import and return a message type.
        
        Args:
            msg_type_str: Message type string like 'std_msgs/String'
            
        Returns:
            The message class
        """
        if msg_type_str in self._msg_types:
            return self._msg_types[msg_type_str]
        
        # Parse type string: package/Type or package.Type
        if '/' in msg_type_str:
            parts = msg_type_str.split('/')
        else:
            parts = msg_type_str.split('.')
        
        if len(parts) != 2:
            raise ImportError(f"Invalid message type format: {msg_type_str}")
        
        package, msg_name = parts
        module = __import__(f"{package}.msg", fromlist=[msg_name])
        msg_class = getattr(module, msg_name)
        self._msg_types[msg_type_str] = msg_class
        return msg_class
    
    async def start(self):
        """Initialize and start the ROS1 node.
        
        Raises:
            RuntimeError: If ROS master is not available.
        """
        if self._initialized:
            logger.warning("ROS1 connector already started")
            return
        
        # Initialize node
        try:
            rospy.init_node(
                self.node_name,
                anonymous=self.anonymous,
                disable_signals=False
            )
            self._initialized = True
            self._running = True
            logger.info(f"ROS1 node '{self.node_name}' started")
        except rospy.exceptions.ROSException as e:
            raise RuntimeError(f"Failed to initialize ROS1 node: {e}")
    
    async def stop(self):
        """Shutdown the ROS1 node and cleanup resources."""
        if not self._initialized:
            return
        
        logger.info("Stopping ROS1 connector...")
        self._running = False
        
        # Unregister all publishers and subscribers
        for pub in list(self._publishers.values()):
            pub.unregister()
        for sub in list(self._subscribers.values()):
            sub.unregister()
        
        self._publishers.clear()
        self._subscribers.clear()
        self._topic_data.clear()
        self._topic_callbacks.clear()
        
        # Note: rospy doesn't have a clean shutdown method
        # The node will be cleaned up when the process exits
        
        self._initialized = False
        logger.info("ROS1 connector stopped")
    
    def get_topics(self) -> List[str]:
        """Get list of available ROS topics.
        
        Returns:
            List of topic names
        """
        if not self._initialized:
            return []
        
        return sorted(rospy.get_published_topics())
    
    def get_topic_types(self) -> Dict[str, str]:
        """Get topic names and their message types.
        
        Returns:
            Dict mapping topic names to message type strings
        """
        if not self._initialized:
            return {}
        
        topics = rospy.get_published_topics()
        return {name: msg_type for name, msg_type in topics}
    
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
        queue_size: int = 10
    ):
        """Subscribe to a ROS topic.
        
        Args:
            topic: Topic name
            callback: Function to call with received messages
            msg_type: Message type class (optional if msg_type_str provided)
            msg_type_str: Message type string like 'std_msgs/String' (optional)
            queue_size: Subscriber queue size
            
        Raises:
            RuntimeError: If node is not running
        """
        if not self._initialized:
            raise RuntimeError("Cannot subscribe: ROS1 node not running")
        
        if topic in self._subscribers:
            # Already subscribed, just add callback
            self._topic_callbacks.setdefault(topic, []).append(callback)
            logger.debug(f"Added callback to existing subscription: {topic}")
            return
        
        # Resolve message type
        if msg_type is None and msg_type_str:
            msg_type = self._get_msg_type(msg_type_str)
        
        if msg_type is None:
            # Try to auto-detect from master
            try:
                msg_type_str = rostopic.get_topic_type(topic)[0]
                if msg_type_str:
                    msg_type = self._get_msg_type(msg_type_str)
            except Exception as e:
                logger.warning(f"Could not auto-detect type for {topic}: {e}")
        
        if msg_type is None:
            raise ValueError(f"Could not determine message type for topic {topic}")
        
        # Store callbacks
        self._topic_callbacks[topic] = [callback]
        
        def _ros_callback(msg):
            """Internal callback that stores data and dispatches to handlers."""
            self._topic_data[topic] = {
                "data": msg,
                "timestamp": time.time()
            }
            
            for cb in self._topic_callbacks.get(topic, []):
                try:
                    cb(msg)
                except Exception as e:
                    logger.error(f"Error in topic callback for {topic}: {e}")
        
        # Create subscriber
        sub = rospy.Subscriber(topic, msg_type, _ros_callback, queue_size=queue_size)
        self._subscribers[topic] = sub
        logger.info(f"Subscribed to {topic} ({msg_type.__module__}.{msg_type.__name__})")
    
    def unsubscribe(self, topic: str):
        """Unsubscribe from a topic.
        
        Args:
            topic: Topic name to unsubscribe from
        """
        if topic in self._subscribers:
            self._subscribers[topic].unregister()
            del self._subscribers[topic]
            del self._topic_callbacks[topic]
            logger.info(f"Unsubscribed from {topic}")
    
    def publish(
        self, 
        topic: str, 
        msg: Any,
        msg_type: Optional[Type] = None,
        msg_type_str: Optional[str] = None,
        queue_size: int = 10
    ):
        """Publish a message to a ROS topic.
        
        Args:
            topic: Topic name
            msg: Message to publish (can be dict or message object)
            msg_type: Message type class
            msg_type_str: Message type string (alternative to msg_type)
            queue_size: Publisher queue size
            
        Raises:
            RuntimeError: If node is not running
        """
        if not self._initialized:
            raise RuntimeError("Cannot publish: ROS1 node not running")
        
        # Resolve message type if needed
        if msg_type is None and msg_type_str:
            msg_type = self._get_msg_type(msg_type_str)
        
        # Create publisher if needed
        if topic not in self._publishers:
            pub = rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=False)
            self._publishers[topic] = pub
            # Give time for connections to establish
            time.sleep(0.1)
            logger.info(f"Created publisher for {topic}")
        
        # Convert dict to message if needed
        if isinstance(msg, dict):
            msg_obj = msg_type()
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
        """Call a ROS service.
        
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
        if not self._initialized:
            raise RuntimeError("Cannot call service: ROS1 node not running")
        
        # Resolve service type
        if service_type is None and service_type_str:
            service_type = self._get_msg_type(service_type_str)
        
        if service_type is None:
            raise ValueError("Must provide service_type or service_type_str")
        
        # Wait for service
        try:
            rospy.wait_for_service(service_name, timeout=timeout_sec)
        except rospy.ROSException:
            raise RuntimeError(f"Service {service_name} not available")
        
        # Create service proxy and call
        proxy = rospy.ServiceProxy(service_name, service_type)
        try:
            response = proxy(request)
            return response
        except rospy.ServiceException as e:
            raise RuntimeError(f"Service call failed: {e}")
    
    def is_running(self) -> bool:
        """Check if the connector is running.
        
        Returns:
            True if node is initialized and running
        """
        return self._initialized and self._running and not rospy.is_shutdown()
    
    def is_shutdown(self) -> bool:
        """Check if ROS is shutting down.
        
        Returns:
            True if ROS is shutting down
        """
        return rospy.is_shutdown()


__all__ = ["ROS1Connector", "TopicInfo"]
