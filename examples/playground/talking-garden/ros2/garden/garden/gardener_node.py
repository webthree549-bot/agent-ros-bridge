"""
Gardener Node - Real ROS2 Node
Monitors plants and sends care commands
Subscribes to /garden/sensors
Publishes to /garden/care_commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class GardenerNode(Node):
    """ROS2 Node for gardener bot that monitors and cares for plants"""
    
    def __init__(self, gardener_name="gardener_1"):
        super().__init__(f'gardener_{gardener_name}')
        self.gardener_name = gardener_name
        
        # Plant tracking
        self.plants = {}  # plant_id -> sensor data
        self.care_history = []
        
        # Publisher for care commands
        self.care_pub = self.create_publisher(String, '/garden/care_commands', 10)
        
        # Subscriber for sensor data
        self.sensor_sub = self.create_subscription(
            String, '/garden/sensors', self.sensor_callback, 10
        )
        
        # Subscriber for poetry (to react to it)
        self.poetry_sub = self.create_subscription(
            String, '/garden/poetry', self.poetry_callback, 10
        )
        
        # Timer for checking plant needs
        self.check_timer = self.create_timer(3.0, self.check_plant_needs)
        
        # Timer for periodic reports
        self.report_timer = self.create_timer(10.0, self.publish_report)
        
        self.get_logger().info(f'🧑‍🌾 Gardener {self.gardener_name} ready to tend the garden!')
        
    def sensor_callback(self, msg):
        """Receive sensor data from plants"""
        try:
            data = json.loads(msg.data)
            plant_id = data['plant_id']
            self.plants[plant_id] = data
            self.get_logger().debug(f"Received data from {plant_id}")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid sensor data received")
            
    def poetry_callback(self, msg):
        """React to poetry from oracle"""
        try:
            poetry = json.loads(msg.data)
            self.get_logger().info(f"📜 Oracle's words: '{poetry.get('verse', '...')[:50]}...'")
            # Happy plants might get extra care
            if poetry.get('mood') == 'joyful':
                self.get_logger().info("🎵 The poetry brings joy! Plants seem happier.")
        except json.JSONDecodeError:
            pass
            
    def check_plant_needs(self):
        """Check plants and send care commands as needed"""
        for plant_id, data in self.plants.items():
            moisture = data.get('moisture', 50)
            light = data.get('light', 50)
            happiness = data.get('happiness', 50)
            species = data.get('species', 'Unknown')
            
            # Decide care based on needs
            action = None
            
            if moisture < 30:
                action = 'water'
                reason = f"moisture low ({moisture:.1f})"
            elif light < 30:
                action = 'light'
                reason = f"light low ({light:.1f})"
            elif happiness < 40:
                action = 'fertilize'
                reason = f"happiness low ({happiness:.1f})"
            elif happiness > 90 and random.random() > 0.9:
                # Occasionally sing to very happy plants
                action = 'sing'
                reason = "celebrating happiness!"
                
            if action:
                self.send_care_command(plant_id, action, reason)
                
    def send_care_command(self, plant_id, action, reason):
        """Send a care command"""
        cmd = {
            'gardener': self.gardener_name,
            'plant': plant_id,
            'action': action,
            'reason': reason,
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.care_pub.publish(msg)
        
        self.care_history.append(cmd)
        emoji = {'water': '💧', 'light': '☀️', 'fertilize': '🌱', 'sing': '🎵'}.get(action, '✨')
        self.get_logger().info(f"{emoji} {action.capitalize()} {plant_id} ({reason})")
        
    def publish_report(self):
        """Publish garden status report"""
        if not self.plants:
            self.get_logger().warn("No plants detected in garden!")
            return
            
        total_happiness = sum(p.get('happiness', 50) for p in self.plants.values())
        avg_happiness = total_happiness / len(self.plants)
        needs_water = sum(1 for p in self.plants.values() if p.get('moisture', 100) < 30)
        
        self.get_logger().info(
            f"📊 Garden Report: {len(self.plants)} plants, "
            f"avg happiness: {avg_happiness:.1f}, "
            f"need water: {needs_water}, "
            f"care actions: {len(self.care_history)}"
        )


def main(args=None):
    import os
    rclpy.init(args=args)
    
    gardener_name = os.environ.get('GARDENER_NAME', '1')
    node = GardenerNode(gardener_name=gardener_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
