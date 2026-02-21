"""
Solar Drone Node - Real ROS2 Node
Provides power and aerial reconnaissance
Publishes to /mars/telemetry, /mars/mission
Subscribes to /mars/commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import json
import random
import math


class SolarDroneNode(Node):
    """ROS2 Node for solar-powered drone"""
    
    def __init__(self, robot_id="solar_drone_1"):
        super().__init__(f'solar_drone_{robot_id}')
        self.robot_id = robot_id
        
        # Drone state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 50.0}  # z is altitude
        self.battery = 100.0
        self.solar_charge_rate = random.uniform(3, 8)
        self.status = 'hovering'
        self.patrol_radius = 100.0
        self.patrol_angle = 0.0
        
        # Discovery tracking
        self.discoveries = []
        self.area_scanned = 0.0
        
        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/mars/telemetry', 10)
        self.mission_pub = self.create_publisher(String, '/mars/mission', 10)
        self.power_pub = self.create_publisher(String, f'/mars/{robot_id}/power', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mars/commands', self.command_callback, 10
        )
        
        # Timers
        self.telemetry_timer = self.create_timer(1.0, self.publish_telemetry)
        self.movement_timer = self.create_timer(2.0, self.update_position)
        self.solar_timer = self.create_timer(5.0, self.charge_solar)
        self.scan_timer = self.create_timer(3.0, self.scan_area)
        
        self.get_logger().info(f'🚁 Solar Drone {self.robot_id} airborne')
        
    def update_position(self):
        """Update drone position"""
        if self.status == 'patrol':
            # Circular patrol pattern
            self.patrol_angle += 0.1
            self.position['x'] = math.cos(self.patrol_angle) * self.patrol_radius
            self.position['y'] = math.sin(self.patrol_angle) * self.patrol_radius
            self.position['z'] = 50 + random.uniform(-5, 5)
        elif self.status == 'hovering':
            # Slight drift while hovering
            self.position['x'] += random.uniform(-2, 2)
            self.position['y'] += random.uniform(-2, 2)
            self.position['z'] = 50 + random.uniform(-3, 3)
            
    def scan_area(self):
        """Scan area below drone"""
        if self.status in ['patrol', 'hovering']:
            self.area_scanned += random.uniform(0.5, 2.0)
            
            # Chance to discover something
            if random.random() < 0.15:
                discovery = {
                    'type': random.choice(['mineral_deposit', 'anomaly', 'terrain_feature', 'resource_vein']),
                    'position': {
                        'x': round(self.position['x'] + random.uniform(-20, 20), 1),
                        'y': round(self.position['y'] + random.uniform(-20, 20), 1)
                    },
                    'confidence': random.uniform(0.5, 0.95)
                }
                self.discoveries.append(discovery)
                self.get_logger().info(f'🔍 Discovered {discovery["type"]} at ({discovery["position"]["x"]:.1f}, {discovery["position"]["y"]:.1f})')
                
                # Publish mission update
                mission_msg = {
                    'type': 'discovery',
                    'drone_id': self.robot_id,
                    'discovery': discovery,
                    'timestamp': self.get_clock().now().to_msg().__str__()
                }
                msg = String()
                msg.data = json.dumps(mission_msg)
                self.mission_pub.publish(msg)
                
    def charge_solar(self):
        """Charge from solar panels"""
        # More charge during "day" (simulated by random for now)
        sun_intensity = random.uniform(0.5, 1.0)
        charge = self.solar_charge_rate * sun_intensity
        self.battery = min(100, self.battery + charge)
        
        # Publish power status
        power_msg = {
            'robot_id': self.robot_id,
            'battery': round(self.battery, 1),
            'charge_rate': round(charge, 2),
            'solar_efficiency': round(self.solar_charge_rate, 2),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        msg = String()
        msg.data = json.dumps(power_msg)
        self.power_pub.publish(msg)
        
    def publish_telemetry(self):
        """Publish telemetry data"""
        telemetry = {
            'robot_id': self.robot_id,
            'type': 'solar_drone',
            'position': {
                'x': round(self.position['x'], 1),
                'y': round(self.position['y'], 1),
                'z': round(self.position['z'], 1)
            },
            'battery': round(self.battery, 1),
            'status': self.status,
            'area_scanned': round(self.area_scanned, 1),
            'discoveries': len(self.discoveries),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)
        
    def command_callback(self, msg):
        """Handle commands"""
        try:
            cmd = json.loads(msg.data)
            if cmd.get('target') == self.robot_id or cmd.get('target') == 'all_drones':
                action = cmd.get('action')
                
                if action == 'patrol':
                    self.status = 'patrol'
                    self.patrol_radius = cmd.get('radius', 100)
                    self.get_logger().info(f'🚁 Starting patrol (radius: {self.patrol_radius})')
                elif action == 'hover':
                    self.status = 'hovering'
                    self.get_logger().info('🚁 Hovering in place')
                elif action == 'goto':
                    self.position['x'] = cmd.get('x', self.position['x'])
                    self.position['y'] = cmd.get('y', self.position['y'])
                    self.get_logger().info(f'🚁 Moving to ({self.position["x"]:.1f}, {self.position["y"]:.1f})')
                elif action == 'land':
                    self.status = 'landed'
                    self.position['z'] = 0
                    self.get_logger().info('🚁 Landing...')
                    
        except json.JSONDecodeError:
            pass


def main(args=None):
    import os
    rclpy.init(args=args)
    
    robot_id = os.environ.get('ROBOT_ID', 'solar_drone_1')
    node = SolarDroneNode(robot_id=robot_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
