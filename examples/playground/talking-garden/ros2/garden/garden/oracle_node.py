"""
Oracle Node - Real ROS2 Node
Generates poetry from garden sensor data
Subscribes to /garden/sensors
Publishes to /garden/poetry
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class OracleNode(Node):
    """ROS2 Node that generates poetry from plant sensor data"""
    
    # Poetry templates
    VERSES = {
        'moisture_high': [
            "Quenched and content, {plant} drinks deep",
            "Water's embrace, {plant} sighs with relief",
            "The roots of {plant} dance in liquid joy",
        ],
        'moisture_low': [
            "Thirsty {plant} dreams of rain",
            "Parched and patient, {plant} waits",
            "The soil whispers dryness to {plant}",
        ],
        'light_high': [
            "{plant} basks in golden rays",
            "Sun-kissed {plant} reaches skyward",
            "Light fills {plant} with radiant joy",
        ],
        'light_low': [
            "{plant} stretches toward distant light",
            "In shadows, {plant} dreams of sun",
            "Dimness wraps {plant} in mystery",
        ],
        'happy': [
            "{plant} sings the song of thriving life",
            "Joy radiates from every {plant} leaf",
            "{plant} blooms in contented peace",
        ],
        'sad': [
            "{plant} weeps silently in the night",
            "Aching {plant} calls for tender care",
            "{plant} wilts with longing",
        ],
        'general': [
            "The garden breathes in ancient rhythms",
            "Leaves whisper secrets to the wind",
            "Roots hold stories deep in earth",
            "Green life pulses with quiet wisdom",
            "Nature's poetry writes itself in growth",
        ]
    }
    
    def __init__(self):
        super().__init__('garden_oracle')
        
        self.plant_data = {}
        self.poem_history = []
        
        # Subscriber for sensor data
        self.sensor_sub = self.create_subscription(
            String, '/garden/sensors', self.sensor_callback, 10
        )
        
        # Publisher for poetry
        self.poetry_pub = self.create_publisher(String, '/garden/poetry', 10)
        
        # Timer for generating poetry
        self.poem_timer = self.create_timer(8.0, self.generate_poem)
        
        self.get_logger().info('🔮 Oracle awakened... watching the garden...')
        
    def sensor_callback(self, msg):
        """Collect sensor data for poetry inspiration"""
        try:
            data = json.loads(msg.data)
            plant_id = data['plant_id']
            self.plant_data[plant_id] = data
        except json.JSONDecodeError:
            pass
            
    def generate_poem(self):
        """Generate poetry based on garden state"""
        if not self.plant_data:
            self.get_logger().debug("No plant data yet, waiting...")
            return
            
        # Pick a random plant to feature
        plant_id = random.choice(list(self.plant_data.keys()))
        data = self.plant_data[plant_id]
        
        # Determine mood and select verses
        mood = self.determine_mood(data)
        verses = self.compose_verses(plant_id, data, mood)
        
        poem = {
            'oracle': 'garden_oracle',
            'featured_plant': plant_id,
            'species': data.get('species', 'Unknown'),
            'emoji': data.get('emoji', '🌱'),
            'mood': mood,
            'verses': verses,
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        # Publish poetry
        msg = String()
        msg.data = json.dumps(poem)
        self.poetry_pub.publish(msg)
        
        self.poem_history.append(poem)
        self.get_logger().info(f"📜 New poem for {plant_id}: '{verses[0]}...'")
        
    def determine_mood(self, data):
        """Determine the mood based on sensor data"""
        happiness = data.get('happiness', 50)
        moisture = data.get('moisture', 50)
        
        if happiness > 70:
            return 'joyful'
        elif happiness < 30:
            return 'melancholy'
        elif moisture < 20:
            return 'thirsty'
        elif moisture > 80:
            return 'content'
        return 'contemplative'
        
    def compose_verses(self, plant_id, data, mood):
        """Compose verses based on plant state"""
        verses = []
        
        moisture = data.get('moisture', 50)
        light = data.get('light', 50)
        happiness = data.get('happiness', 50)
        
        # Add moisture verse
        if moisture > 70:
            verses.append(random.choice(self.VERSES['moisture_high']).format(plant=plant_id))
        elif moisture < 30:
            verses.append(random.choice(self.VERSES['moisture_low']).format(plant=plant_id))
            
        # Add light verse
        if light > 80:
            verses.append(random.choice(self.VERSES['light_high']).format(plant=plant_id))
        elif light < 20:
            verses.append(random.choice(self.VERSES['light_low']).format(plant=plant_id))
            
        # Add happiness verse
        if happiness > 75:
            verses.append(random.choice(self.VERSES['happy']).format(plant=plant_id))
        elif happiness < 25:
            verses.append(random.choice(self.VERSES['sad']).format(plant=plant_id))
            
        # Ensure at least one verse
        if not verses:
            verses.append(random.choice(self.VERSES['general']))
            
        # Add a general verse for atmosphere
        if random.random() > 0.5:
            verses.append(random.choice(self.VERSES['general']))
            
        return verses


def main(args=None):
    rclpy.init(args=args)
    node = OracleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
