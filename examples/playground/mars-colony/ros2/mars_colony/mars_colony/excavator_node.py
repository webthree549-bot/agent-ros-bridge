"""
Excavator Node - Real ROS2 Node
Mines regolith and extracts resources
Publishes to /mars/resources, /mars/telemetry
Subscribes to /mars/commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
import json
import random


class ExcavatorNode(Node):
    """ROS2 Node for excavation robot"""
    
    def __init__(self, robot_id="excavator_1"):
        super().__init__(f'excavator_{robot_id}')
        self.robot_id = robot_id
        
        # Robot state
        self.position = {'x': random.uniform(-50, 50), 'y': random.uniform(-50, 50), 'z': 0.0}
        self.battery = 100.0
        self.cargo = 0.0  # kg of regolith
        self.cargo_capacity = 500.0
        self.status = 'idle'  # idle, mining, returning
        self.mining_efficiency = random.uniform(0.8, 1.2)
        
        # Resources found
        self.resources = {'regolith': 0, 'water_ice': 0, 'minerals': 0, 'helium3': 0}
        
        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/mars/telemetry', 10)
        self.resource_pub = self.create_publisher(String, '/mars/resources', 10)
        self.status_pub = self.create_publisher(String, f'/mars/{robot_id}/status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/mars/commands', self.command_callback, 10
        )
        
        # Timers
        self.telemetry_timer = self.create_timer(2.0, self.publish_telemetry)
        self.work_timer = self.create_timer(5.0, self.do_work)
        self.battery_timer = self.create_timer(10.0, self.drain_battery)
        
        self.get_logger().info(f'🚜 Excavator {self.robot_id} initialized at ({self.position["x"]:.1f}, {self.position["y"]:.1f})')
        
    def do_work(self):
        """Main work loop - mining operations"""
        if self.battery < 10:
            self.status = 'low_battery'
            return
            
        if self.status == 'mining':
            # Mine regolith
            amount = random.uniform(5, 15) * self.mining_efficiency
            self.cargo = min(self.cargo_capacity, self.cargo + amount)
            
            # Discover resources
            if random.random() < 0.3:
                resource_type = random.choice(['water_ice', 'minerals', 'helium3'])
                found = random.uniform(0.1, 2.0)
                self.resources[resource_type] += found
                self.get_logger().info(f'💎 Found {found:.2f}kg of {resource_type}!')
                
            # Update resources
            self.resources['regolith'] = self.cargo
            
            if self.cargo >= self.cargo_capacity * 0.9:
                self.status = 'returning'
                self.get_logger().info('⛏️  Cargo full, returning to base')
                
        elif self.status == 'returning':
            # Return to base (simplified)
            self.position['x'] *= 0.5
            self.position['y'] *= 0.5
            
            # Drop off cargo near base
            if abs(self.position['x']) < 5 and abs(self.position['y']) < 5:
                self.publish_resources()
                self.cargo = 0
                self.resources['regolith'] = 0
                self.status = 'idle'
                self.get_logger().info('📦 Cargo delivered to base')
                
    def publish_resources(self):
        """Publish discovered resources"""
        resource_msg = {
            'robot_id': self.robot_id,
            'type': 'excavation',
            'resources': self.resources.copy(),
            'position': self.position.copy(),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(resource_msg)
        self.resource_pub.publish(msg)
        
    def publish_telemetry(self):
        """Publish telemetry data"""
        telemetry = {
            'robot_id': self.robot_id,
            'type': 'excavator',
            'position': self.position.copy(),
            'battery': round(self.battery, 1),
            'cargo': round(self.cargo, 1),
            'status': self.status,
            'efficiency': round(self.mining_efficiency, 2),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)
        self.status_pub.publish(msg)
        
    def drain_battery(self):
        """Simulate battery drain"""
        drain = random.uniform(2, 5)
        if self.status == 'mining':
            drain *= 2
        self.battery = max(0, self.battery - drain)
        
    def command_callback(self, msg):
        """Handle commands from command center"""
        try:
            cmd = json.loads(msg.data)
            if cmd.get('target') == self.robot_id or cmd.get('target') == 'all_excavators':
                action = cmd.get('action')
                
                if action == 'mine':
                    self.status = 'mining'
                    self.get_logger().info('⛏️  Starting mining operation')
                elif action == 'stop':
                    self.status = 'idle'
                    self.get_logger().info('🛑 Stopping operations')
                elif action == 'return':
                    self.status = 'returning'
                    self.get_logger().info('🏠 Returning to base')
                elif action == 'charge':
                    self.battery = min(100, self.battery + 20)
                    self.get_logger().info(f'🔋 Charging... Battery: {self.battery:.1f}%')
                    
        except json.JSONDecodeError:
            pass


def main(args=None):
    import os
    rclpy.init(args=args)
    
    robot_id = os.environ.get('ROBOT_ID', 'excavator_1')
    node = ExcavatorNode(robot_id=robot_id)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
