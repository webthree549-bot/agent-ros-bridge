#!/usr/bin/env python3
"""MCAP recording for ROS2 topics - JSON fallback version."""

import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, '/opt/ros/jazzy/local/lib/python3.12/dist-packages')

try:
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available")

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mcap_recorder")


class MCAPRecorder(Node):
    """Record ROS2 topics to JSONL file."""
    
    def __init__(self, output_dir: str = "/workspace/recordings"):
        super().__init__('mcap_recorder')
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create recording file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.json_path = self.output_dir / f"recording_{timestamp}.jsonl"
        
        # Topic subscriptions
        self.subscriptions_list = []
        
        # Recording stats
        self.message_count = 0
        self.start_time = time.time()
        
        # Open file for writing
        self.json_file = open(self.json_path, 'w')
        logger.info(f"Recording to: {self.json_path}")
        
        # Subscribe to topics
        self.subscribe_topic('/tf', TFMessage)
        self.subscribe_topic('/odom', Odometry)
        self.subscribe_topic('/scan', LaserScan)
        
        # Timer for stats
        self.create_timer(10.0, self.print_stats)
        
        logger.info("MCAP Recorder initialized")
    
    def subscribe_topic(self, topic_name: str, msg_type):
        """Subscribe to a topic."""
        sub = self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, name=topic_name: self.on_message(msg, name),
            10
        )
        self.subscriptions_list.append(sub)
        logger.info(f"Subscribed to {topic_name}")
    
    def on_message(self, msg, topic_name: str):
        """Handle incoming message."""
        try:
            timestamp = time.time()
            
            # Serialize message to dict
            data = self.msg_to_dict(msg)
            
            record = {
                'topic': topic_name,
                'timestamp': timestamp,
                'type': type(msg).__name__,
                'data': data
            }
            
            self.json_file.write(json.dumps(record) + '\n')
            self.json_file.flush()
            
            self.message_count += 1
            
        except Exception as e:
            logger.error(f"Error recording {topic_name}: {e}")
    
    def msg_to_dict(self, msg) -> dict:
        """Convert ROS message to dictionary."""
        if hasattr(msg, '__slots__'):
            result = {}
            for slot in msg.__slots__:
                value = getattr(msg, slot)
                if hasattr(value, '__slots__'):
                    result[slot] = self.msg_to_dict(value)
                elif isinstance(value, list):
                    result[slot] = [
                        self.msg_to_dict(item) if hasattr(item, '__slots__') else item
                        for item in value
                    ]
                else:
                    result[slot] = value
            return result
        return str(msg)
    
    def print_stats(self):
        """Print recording statistics."""
        elapsed = time.time() - self.start_time
        rate = self.message_count / elapsed if elapsed > 0 else 0
        logger.info(f"Recording: {self.message_count} messages, {rate:.1f} msg/s")
    
    def stop_recording(self):
        """Stop recording and close files."""
        self.json_file.close()
        logger.info(f"Recording saved: {self.json_path}")
        self.print_stats()


def main():
    if not ROS2_AVAILABLE:
        logger.error("ROS2 not available")
        return
    
    rclpy.init()
    recorder = MCAPRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
