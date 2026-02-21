"""
Command Node - Real ROS2 Node
Coordinates the robot fleet
Subscribes to /mars/telemetry, /mars/resources, /mars/mission
Publishes to /mars/commands
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class CommandNode(Node):
    """ROS2 Node for mission command center"""
    
    def __init__(self):
        super().__init__('mars_command')
        
        # Fleet tracking
        self.robots = {}  # robot_id -> latest telemetry
        self.mission_log = []
        self.resource_inventory = {
            'regolith': 0,
            'water_ice': 0,
            'minerals': 0,
            'helium3': 0
        }
        
        # Mission state
        self.mission_status = 'standby'
        self.objectives = [
            {'name': 'Establish Base', 'completed': False},
            {'name': 'Map Surroundings', 'completed': False},
            {'name': 'Extract Resources', 'completed': False},
            {'name': 'Build Habitat', 'completed': False},
        ]
        
        # Publishers
        self.command_pub = self.create_publisher(String, '/mars/commands', 10)
        self.status_pub = self.create_publisher(String, '/mars/command_status', 10)
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(
            String, '/mars/telemetry', self.telemetry_callback, 10
        )
        self.resource_sub = self.create_subscription(
            String, '/mars/resources', self.resource_callback, 10
        )
        self.mission_sub = self.create_subscription(
            String, '/mars/mission', self.mission_callback, 10
        )
        
        # Timers
        self.status_timer = self.create_timer(5.0, self.publish_status)
        self.coordinate_timer = self.create_timer(10.0, self.coordinate_fleet)
        self.mission_timer = self.create_timer(30.0, self.check_objectives)
        
        self.get_logger().info('🎯 Mars Command Center online')
        self.get_logger().info(f'📋 Mission objectives: {len(self.objectives)}')
        
    def telemetry_callback(self, msg):
        """Receive telemetry from all robots"""
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            self.robots[robot_id] = data
        except (json.JSONDecodeError, KeyError):
            pass
            
    def resource_callback(self, msg):
        """Receive resource updates"""
        try:
            data = json.loads(msg.data)
            resources = data.get('resources', {})
            for resource, amount in resources.items():
                if resource in self.resource_inventory:
                    self.resource_inventory[resource] += amount
                    
            self.get_logger().info(
                f'📦 Resources: Regolith={self.resource_inventory["regolith"]:.0f}kg, '
                f'Ice={self.resource_inventory["water_ice"]:.1f}kg, '
                f'Minerals={self.resource_inventory["minerals"]:.1f}kg'
            )
        except json.JSONDecodeError:
            pass
            
    def mission_callback(self, msg):
        """Receive mission updates"""
        try:
            data = json.loads(msg.data)
            self.mission_log.append(data)
            
            event_type = data.get('type')
            if event_type == 'discovery':
                self.get_logger().info(f'🔍 New discovery by {data["drone_id"]}')
            elif event_type == 'hazard_detected':
                hazard = data.get('hazard', {})
                self.get_logger().warn(f'⚠️  Alert: {hazard.get("type")} detected!')
            elif event_type == 'construction_complete':
                self.get_logger().info(f'✅ Construction complete: {data["project"]["name"]}')
                
        except json.JSONDecodeError:
            pass
            
    def coordinate_fleet(self):
        """Coordinate robot activities"""
        if not self.robots:
            return
            
        # Count robot types
        excavators = [r for r in self.robots.values() if r.get('type') == 'excavator']
        drones = [r for r in self.robots.values() if r.get('type') == 'solar_drone']
        scouts = [r for r in self.robots.values() if r.get('type') == 'scout']
        builders = [r for r in self.robots.values() if r.get('type') == 'builder']
        
        # Auto-coordinate based on mission status
        if self.mission_status == 'exploration' and scouts:
            # Send scouts to explore
            idle_scouts = [s for s in scouts if s.get('status') == 'idle']
            if idle_scouts and len(self.mission_log) < 5:
                self.send_command('all_scouts', 'explore', {'range': 300})
                
        elif self.mission_status == 'resource_gathering' and excavators:
            # Start mining operations
            idle_excavators = [e for e in excavators if e.get('status') == 'idle']
            for _ in idle_excavators[:2]:  # Start up to 2
                self.send_command('all_excavators', 'mine')
                
    def check_objectives(self):
        """Check and update mission objectives"""
        # Check objective completion
        if not self.objectives[0]['completed'] and len(self.robots) >= 4:
            self.objectives[0]['completed'] = True
            self.get_logger().info('✅ Objective completed: Establish Base')
            
        if not self.objectives[1]['completed'] and len(self.mission_log) > 10:
            self.objectives[1]['completed'] = True
            self.get_logger().info('✅ Objective completed: Map Surroundings')
            
        if not self.objectives[2]['completed'] and self.resource_inventory['regolith'] > 1000:
            self.objectives[2]['completed'] = True
            self.get_logger().info('✅ Objective completed: Extract Resources')
            
        # Auto-advance mission status
        completed = sum(1 for o in self.objectives if o['completed'])
        if completed == 0:
            self.mission_status = 'establishing_base'
        elif completed == 1:
            self.mission_status = 'exploration'
        elif completed == 2:
            self.mission_status = 'resource_gathering'
        elif completed == 3:
            self.mission_status = 'construction'
        else:
            self.mission_status = 'operational'
            
    def send_command(self, target, action, params=None):
        """Send command to robots"""
        cmd = {
            'from': 'mars_command',
            'target': target,
            'action': action,
            'params': params or {},
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.command_pub.publish(msg)
        self.get_logger().debug(f'📤 Command: {action} -> {target}')
        
    def publish_status(self):
        """Publish command center status"""
        active_robots = sum(1 for r in self.robots.values() 
                          if r.get('status') not in ['idle', 'low_battery'])
        
        status = {
            'command_center': 'online',
            'mission_status': self.mission_status,
            'robots_online': len(self.robots),
            'robots_active': active_robots,
            'objectives_completed': sum(1 for o in self.objectives if o['completed']),
            'total_objectives': len(self.objectives),
            'resources': self.resource_inventory.copy(),
            'timestamp': self.get_clock().now().to_msg().__str__()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
        self.get_logger().info(
            f'🎯 Status: {self.mission_status}, '
            f'Robots: {len(self.robots)} ({active_robots} active), '
            f'Objectives: {status["objectives_completed"]}/{status["total_objectives"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
