"""
Plant Node - Real ROS2 Node
Publishes sensor data to /garden/sensors
Subscribes to /garden/care_commands for watering/light
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json
import random
import math


class PlantNode(Node):
    """ROS2 Node for a plant with personality-based sensor data"""
    
    # Plant personalities with different base characteristics
    PERSONALITIES = {
        'fernando': {'species': 'Fern', 'moisture_love': 0.8, 'light_needs': 0.3, 'temp_pref': 22, 'emoji': '🌿'},
        'cactilda': {'species': 'Cactus', 'moisture_love': 0.1, 'light_needs': 0.9, 'temp_pref': 28, 'emoji': '🌵'},
        'rosalind': {'species': 'Rose', 'moisture_love': 0.6, 'light_needs': 0.7, 'temp_pref': 20, 'emoji': '🌹'},
        'basilion': {'species': 'Basil', 'moisture_love': 0.7, 'light_needs': 0.8, 'temp_pref': 24, 'emoji': '🌿'},
        'orchidelle': {'species': 'Orchid', 'moisture_love': 0.5, 'light_needs': 0.5, 'temp_pref': 21, 'emoji': '🌺'},
        'sunflower_sam': {'species': 'Sunflower', 'moisture_love': 0.6, 'light_needs': 0.95, 'temp_pref': 25, 'emoji': '🌻'},
    }
    
    def __init__(self, plant_id='fernando'):
        super().__init__(f'plant_{plant_id}')
        self.plant_id = plant_id
        self.personality = self.PERSONALITIES.get(plant_id, self.PERSONALITIES['fernando'])
        
        # Internal state
        self.moisture = random.uniform(40, 80)  # 0-100%
        self.light = random.uniform(30, 70)  # 0-100%
        self.temperature = self.personality['temp_pref'] + random.uniform(-3, 3)
        self.happiness = 50
        self.last_care = 'None'
        
        # Publishers
        self.sensor_pub = self.create_publisher(String, '/garden/sensors', 10)
        self.status_pub = self.create_publisher(String, f'/garden/{plant_id}/status', 10)
        
        # Individual sensor publishers
        self.moisture_pub = self.create_publisher(Float64, f'/garden/{plant_id}/moisture', 10)
        self.light_pub = self.create_publisher(Float64, f'/garden/{plant_id}/light', 10)
        self.temp_pub = self.create_publisher(Float64, f'/garden/{plant_id}/temperature', 10)
        
        # Subscriber for care commands
        self.care_sub = self.create_subscription(
            String, '/garden/care_commands', self.care_callback, 10
        )
        
        # Timer for sensor updates
        self.sensor_timer = self.create_timer(2.0, self.publish_sensors)
        # Timer for environmental changes
        self.env_timer = self.create_timer(5.0, self.update_environment)
        
        self.get_logger().info(f"{self.personality['emoji']} {self.plant_id} ({self.personality['species']}) planted!")
        
    def update_environment(self):
        """Simulate environmental changes"""
        # Moisture slowly decreases
        self.moisture = max(0, self.moisture - random.uniform(0.5, 2.0))
        
        # Light fluctuates based on time (simulated)
        time_factor = math.sin(self.get_clock().now().nanoseconds / 1e9 / 10) * 0.5 + 0.5
        base_light = self.personality['light_needs'] * 100
        self.light = base_light * time_factor + random.uniform(-10, 10)
        self.light = max(0, min(100, self.light))
        
        # Temperature drifts
        self.temperature += random.uniform(-0.5, 0.5)
        self.temperature = max(15, min(35, self.temperature))
        
        # Calculate happiness based on needs
        moisture_diff = abs(self.moisture / 100 - self.personality['moisture_love'])
        light_diff = abs(self.light / 100 - self.personality['light_needs'])
        temp_diff = abs(self.temperature - self.personality['temp_pref']) / 10
        
        self.happiness = max(0, min(100, 100 - (moisture_diff + light_diff + temp_diff) * 50))
        
    def publish_sensors(self):
        """Publish sensor data to ROS2 topics"""
        sensor_data = {
            'plant_id': self.plant_id,
            'species': self.personality['species'],
            'emoji': self.personality['emoji'],
            'moisture': round(self.moisture, 1),
            'light': round(self.light, 1),
            'temperature': round(self.temperature, 1),
            'happiness': round(self.happiness, 1),
            'last_care': self.last_care,
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        # Publish to main sensors topic
        msg = String()
        msg.data = json.dumps(sensor_data)
        self.sensor_pub.publish(msg)
        
        # Publish to individual topics
        moisture_msg = Float64()
        moisture_msg.data = self.moisture
        self.moisture_pub.publish(moisture_msg)
        
        light_msg = Float64()
        light_msg.data = self.light
        self.light_pub.publish(light_msg)
        
        temp_msg = Float64()
        temp_msg.data = self.temperature
        self.temp_pub.publish(temp_msg)
        
        self.get_logger().debug(f"Sensors: M={self.moisture:.1f} L={self.light:.1f} T={self.temperature:.1f}")
        
    def care_callback(self, msg):
        """Handle care commands"""
        try:
            cmd = json.loads(msg.data)
            if cmd.get('plant') == self.plant_id or cmd.get('plant') == 'all':
                action = cmd.get('action')
                if action == 'water':
                    self.moisture = min(100, self.moisture + random.uniform(20, 40))
                    self.last_care = 'Watered'
                    self.get_logger().info(f"💧 {self.plant_id} was watered! Moisture: {self.moisture:.1f}")
                elif action == 'light':
                    self.light = min(100, self.light + random.uniform(15, 30))
                    self.last_care = 'Light boost'
                    self.get_logger().info(f"☀️  {self.plant_id} got light boost! Light: {self.light:.1f}")
                elif action == 'fertilize':
                    self.happiness = min(100, self.happiness + 15)
                    self.last_care = 'Fertilized'
                    self.get_logger().info(f"🌱 {self.plant_id} was fertilized! Happiness: {self.happiness:.1f}")
                elif action == 'sing':
                    self.happiness = min(100, self.happiness + 10)
                    self.last_care = 'Sung to'
                    self.get_logger().info(f"🎵 {self.plant_id} enjoyed the song! Happiness: {self.happiness:.1f}")
                    
                # Publish status update
                status = {'plant': self.plant_id, 'action': action, 'happiness': self.happiness}
                status_msg = String()
                status_msg.data = json.dumps(status)
                self.status_pub.publish(status_msg)
                
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid care command: {msg.data}")


def main(args=None):
    import os
    rclpy.init(args=args)
    
    # Get plant ID from environment
    plant_id = os.environ.get('PLANT_ID', 'fernando')
    
    node = PlantNode(plant_id=plant_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
