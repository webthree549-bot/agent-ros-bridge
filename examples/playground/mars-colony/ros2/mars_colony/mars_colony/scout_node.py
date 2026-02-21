"""
Scout Node - Real ROS2 Node
Explores terrain and identifies hazards
Publishes to /mars/telemetry, /mars/mission
Subscribes to /mars/commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import math


class ScoutNode(Node):
    """ROS2 Node for exploration scout robot"""
    
    def __init__(self, robot_id="scout_1"):
        super().__init__(f'scout_{robot_id}')
        self.robot_id = robot_id
        
        # Scout state
        self.position = {'x': 0.0, 'y': 0.0}
        self.battery = 100.0
        self.status = 'idle'
        self.speed = random.uniform(2.0, 4.0)
        self.exploration_range = 200.0
        
        # Exploration data
        self.terrain_mapped = 0.0  # sq km
        self.hazards_found = []
        self.points_of_interest = []
        self.exploration_path = []
        
        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/mars/telemetry', 10)
        self.mission_pub = self.create_publisher(String, '/mars/mission', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mars/commands', self.command_callback, 10
        )
        
        # Timers
        self.telemetry_timer = self.create_timer(2.0, self.publish_telemetry)
        self.explore_timer = self.create_timer(1.0, self.explore)
        self.battery_timer = self.create_timer(10.0, self.drain_battery)
        
        self.get_logger().info(f'🔭 Scout {self.robot_id} ready for exploration')
        
    def explore(self):
        """Exploration behavior"""
        if self.status != 'exploring':
            return
            
        if self.battery < 20:
            self.status = 'returning'
            self.get_logger().info('🔋 Low battery, returning to base')
            return
            
        # Random walk exploration
        angle = random.uniform(0, 2 * math.pi)
        distance = self.speed * random.uniform(0.5, 1.5)
        
        new_x = self.position['x'] + math.cos(angle) * distance
        new_y = self.position['y'] + math.sin(angle) * distance
        
        # Check exploration range
        if math.sqrt(new_x**2 + new_y**2) > self.exploration_range:
            # Turn back toward center
            angle = math.atan2(-self.position['y'], -self.position['x'])
            new_x = self.position['x'] + math.cos(angle) * distance
            new_y = self.position['y'] + math.sin(angle) * distance
            
        self.position['x'] = new_x
        self.position['y'] = new_y
        self.exploration_path.append(self.position.copy())
        
        # Map terrain
        self.terrain_mapped += random.uniform(0.01, 0.05)
        
        # Discover hazards
        if random.random() < 0.08:
            hazard = {
                'type': random.choice(['dust_devil', 'rock_field', 'crater', 'steep_slope', 'unstable_ground']),
                'position': self.position.copy(),
                'severity': random.choice(['low', 'medium', 'high']),
                'radius': random.uniform(5, 50)
            }
            self.hazards_found.append(hazard)
            self.get_logger().warn(f'⚠️  Hazard detected: {hazard["type"]} ({hazard["severity"]})')
            
            # Publish mission update
            mission_msg = {
                'type': 'hazard_detected',
                'robot_id': self.robot_id,
                'hazard': hazard,
                'timestamp': self.get_clock().now().to_msg().__str__()
            }
            msg = String()
            msg.data = json.dumps(mission_msg)
            self.mission_pub.publish(msg)
            
        # Discover points of interest
        if random.random() < 0.05:
            poi = {
                'type': random.choice(['cave', 'valley', 'ridge', 'ancient_riverbed', 'mineral_outcrop']),
                'position': self.position.copy(),
                'significance': random.uniform(0.3, 1.0)
            }
            self.points_of_interest.append(poi)
            self.get_logger().info(f'📍 Point of interest: {poi["type"]} (significance: {poi["significance"]:.2f})')
            
    def publish_telemetry(self):
        """Publish telemetry data"""
        # Calculate distance from base
        distance_from_base = math.sqrt(self.position['x']**2 + self.position['y']**2)
        
        telemetry = {
            'robot_id': self.robot_id,
            'type': 'scout',
            'position': {
                'x': round(self.position['x'], 1),
                'y': round(self.position['y'], 1)
            },
            'distance_from_base': round(distance_from_base, 1),
            'battery': round(self.battery, 1),
            'status': self.status,
            'terrain_mapped': round(self.terrain_mapped, 2),
            'hazards_found': len(self.hazards_found),
            'poi_found': len(self.points_of_interest),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)
        
    def drain_battery(self):
        """Simulate battery drain"""
        drain = 2.0
        if self.status == 'exploring':
            drain = 3.5
        self.battery = max(0, self.battery - drain)
        
        if self.status == 'returning':
            # Move toward base
            distance = math.sqrt(self.position['x']**2 + self.position['y']**2)
            if distance > 5:
                angle = math.atan2(-self.position['y'], -self.position['x'])
                self.position['x'] += math.cos(angle) * self.speed * 2
                self.position['y'] += math.sin(angle) * self.speed * 2
            else:
                self.status = 'idle'
                self.get_logger().info('🏠 Returned to base')
                
    def command_callback(self, msg):
        """Handle commands"""
        try:
            cmd = json.loads(msg.data)
            if cmd.get('target') == self.robot_id or cmd.get('target') == 'all_scouts':
                action = cmd.get('action')
                
                if action == 'explore':
                    self.status = 'exploring'
                    range_val = cmd.get('range', 200)
                    self.exploration_range = range_val
                    self.get_logger().info(f'🔭 Starting exploration (range: {range_val}m)')
                    
                elif action == 'goto':
                    self.position['x'] = cmd.get('x', self.position['x'])
                    self.position['y'] = cmd.get('y', self.position['y'])
                    self.status = 'moving'
                    self.get_logger().info(f'🔭 Moving to ({self.position["x"]:.1f}, {self.position["y"]:.1f})')
                    
                elif action == 'return':
                    self.status = 'returning'
                    self.get_logger().info('🔭 Returning to base')
                    
                elif action == 'charge':
                    if self.status == 'idle':
                        self.battery = min(100, self.battery + 30)
                        self.get_logger().info(f'🔋 Charging... Battery: {self.battery:.1f}%')
                    else:
                        self.get_logger().warn('Must be idle to charge')
                        
                elif action == 'map_hazards':
                    # Report all hazards found
                    self.get_logger().info(f'🗺️  Reporting {len(self.hazards_found)} hazards')
                    for hazard in self.hazards_found:
                        mission_msg = {
                            'type': 'hazard_report',
                            'robot_id': self.robot_id,
                            'hazard': hazard,
                            'timestamp': self.get_clock().now().to_msg().__str__()
                        }
                        msg = String()
                        msg.data = json.dumps(mission_msg)
                        self.mission_pub.publish(msg)
                        
        except json.JSONDecodeError:
            pass


def main(args=None):
    import os
    rclpy.init(args=args)
    
    robot_id = os.environ.get('ROBOT_ID', 'scout_1')
    node = ScoutNode(robot_id=robot_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
