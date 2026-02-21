"""
Painter Node - Real ROS2 Node
Publishes painting strokes to /art/strokes
Subscribes to /art/emotion for painting commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point
import json
import random


class PainterNode(Node):
    """ROS2 Node for painting robot"""
    
    def __init__(self, robot_id="painter_1", style="abstract"):
        super().__init__(f'painter_{robot_id}')
        self.robot_id = robot_id
        self.style = style
        
        # Publisher: Painting strokes
        self.stroke_pub = self.create_publisher(
            String, 
            '/art/strokes', 
            10
        )
        
        # Publisher: Robot status
        self.status_pub = self.create_publisher(
            String,
            '/art/robot_status',
            10
        )
        
        # Subscriber: Emotion commands
        self.emotion_sub = self.create_subscription(
            String,
            '/art/emotion',
            self.emotion_callback,
            10
        )
        
        self.get_logger().info(f'🎨 {self.robot_id} ({self.style}) initialized')
        
    def emotion_callback(self, msg):
        """Receive emotion command and paint"""
        emotion = msg.data
        self.get_logger().info(f'Received emotion: {emotion}')
        
        # Generate and publish stroke
        stroke = self.generate_stroke(emotion)
        stroke_msg = String()
        stroke_msg.data = json.dumps(stroke)
        self.stroke_pub.publish(stroke_msg)
        
        self.get_logger().info(f'Published stroke: {stroke["description"]}')
        
    def generate_stroke(self, emotion):
        """Generate painting stroke based on emotion"""
        colors = {
            "joy": "#FFD700", "sadness": "#4A5568",
            "anger": "#DC2626", "calm": "#48BB78"
        }
        
        return {
            "robot_id": self.robot_id,
            "style": self.style,
            "emotion": emotion,
            "color": colors.get(emotion, "#000000"),
            "description": f"{self.robot_id} painted with {emotion}",
            "timestamp": self.get_clock().now().to_msg()
        }


def main(args=None):
    import os
    rclpy.init(args=args)
    
    # Get robot ID from env or default
    robot_id = os.environ.get('ROBOT_ID', 'painter_1')
    style = os.environ.get('ROBOT_STYLE', 'abstract')
    
    node = PainterNode(robot_id=robot_id, style=style)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()