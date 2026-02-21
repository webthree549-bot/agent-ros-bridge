"""
Canvas Node - Real ROS2 Node
Aggregates strokes from all painters
Publishes canvas state
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class CanvasNode(Node):
    """ROS2 Node for canvas aggregation"""
    
    def __init__(self):
        super().__init__('canvas_aggregator')
        
        self.strokes = []
        
        # Subscriber: All painting strokes
        self.stroke_sub = self.create_subscription(
            String,
            '/art/strokes',
            self.stroke_callback,
            10
        )
        
        # Publisher: Canvas state
        self.canvas_pub = self.create_publisher(
            String,
            '/art/canvas',
            10
        )
        
        # Timer: Publish canvas state periodically
        self.timer = self.create_timer(2.0, self.publish_canvas)
        
        self.get_logger().info('🎨 Canvas aggregator initialized')
        
    def stroke_callback(self, msg):
        """Receive stroke from painter"""
        stroke = json.loads(msg.data)
        self.strokes.append(stroke)
        self.get_logger().info(f'Received stroke from {stroke["robot_id"]}')
        
    def publish_canvas(self):
        """Publish current canvas state"""
        canvas_state = {
            "generation": len(self.strokes),
            "strokes": self.strokes[-10:],  # Last 10 strokes
            "robot_count": len(set(s["robot_id"] for s in self.strokes))
        }
        
        msg = String()
        msg.data = json.dumps(canvas_state)
        self.canvas_pub.publish(msg)
        
        self.get_logger().info(f'Canvas: {canvas_state["generation"]} strokes')


def main(args=None):
    rclpy.init(args=args)
    node = CanvasNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()